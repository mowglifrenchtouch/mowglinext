// Copyright (C) 2024 Cedric
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * @file coverage_planner_node.cpp
 * @brief Implementation of the CoveragePlannerNode.
 *
 * Algorithm overview:
 *   1. Validate the input boundary polygon (must have >= 3 vertices).
 *   2. Generate headland passes: shrink the outer boundary by
 *      (headland_passes_ * headland_width_) and trace a path around each
 *      successive contraction ring.
 *   3. Determine the mowing angle (auto or user-specified).
 *   4. Generate parallel swaths inside the inner polygon at path_spacing_
 *      intervals, alternating direction (boustrophedon).
 *   5. Convert swaths and headland rings to nav_msgs::msg::Path with poses
 *      oriented along the direction of travel.
 *   6. Publish both paths for RViz and populate the service response.
 */

#include "mowgli_coverage_planner/coverage_planner_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "mowgli_coverage_planner/polygon_utils.hpp"

namespace mowgli_coverage_planner
{

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

CoveragePlannerNode::CoveragePlannerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("coverage_planner_node", options)
{
  // Declare parameters with defaults.
  tool_width_ = declare_parameter<double>("tool_width", 0.18);
  headland_passes_ = declare_parameter<int>("headland_passes", 2);
  headland_width_ = declare_parameter<double>("headland_width", 0.18);
  default_mow_angle_ = declare_parameter<double>("default_mow_angle", -1.0);
  path_spacing_ = declare_parameter<double>("path_spacing", 0.18);
  map_frame_ = declare_parameter<std::string>("map_frame", "map");

  // Validate parameters at startup.
  if (tool_width_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "tool_width must be positive; clamping to 0.01 m");
    tool_width_ = 0.01;
  }
  if (headland_passes_ < 0) {
    RCLCPP_WARN(get_logger(), "headland_passes must be >= 0; setting to 0");
    headland_passes_ = 0;
  }
  if (headland_width_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "headland_width must be positive; clamping to tool_width");
    headland_width_ = tool_width_;
  }
  if (path_spacing_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "path_spacing must be positive; clamping to tool_width");
    path_spacing_ = tool_width_;
  }

  RCLCPP_INFO(
    get_logger(),
    "CoveragePlannerNode: tool_width=%.3f m, headland_passes=%d, "
    "headland_width=%.3f m, path_spacing=%.3f m, frame='%s'",
    tool_width_, headland_passes_, headland_width_, path_spacing_, map_frame_.c_str());

  // Publishers (latched-like via transient-local QoS).
  const auto qos = rclcpp::QoS(1).transient_local();
  path_pub_ = create_publisher<nav_msgs::msg::Path>("~/coverage_path", qos);
  outline_pub_ = create_publisher<nav_msgs::msg::Path>("~/coverage_outline", qos);

  // Service.
  service_ = create_service<mowgli_interfaces::srv::PlanCoveragePath>(
    "~/plan_coverage",
    [this](
      const std::shared_ptr<mowgli_interfaces::srv::PlanCoveragePath::Request> req,
      std::shared_ptr<mowgli_interfaces::srv::PlanCoveragePath::Response> res) {
      handle_plan_coverage(req, res);
    });

  RCLCPP_INFO(get_logger(), "Coverage planner service ready at ~/plan_coverage");
}

// ---------------------------------------------------------------------------
// Service callback
// ---------------------------------------------------------------------------

void CoveragePlannerNode::handle_plan_coverage(
  const std::shared_ptr<mowgli_interfaces::srv::PlanCoveragePath::Request> request,
  std::shared_ptr<mowgli_interfaces::srv::PlanCoveragePath::Response> response)
{
  RCLCPP_INFO(get_logger(), "Received coverage planning request");

  // Convert ROS polygon to internal representation.
  const Polygon2D outer = geometry_polygon_to_points(request->outer_boundary);

  if (outer.size() < 3) {
    response->success = false;
    response->message = "outer_boundary must have at least 3 vertices";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }

  const double outer_area = std::abs(polygon_area(outer));
  if (outer_area < 1e-6) {
    response->success = false;
    response->message = "outer_boundary has zero or near-zero area";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }

  // ---- Headland outline path ------------------------------------------
  nav_msgs::msg::Path outline_path;
  if (!request->skip_outline) {
    outline_path = generate_outline_path(outer, map_frame_);
  }

  // ---- Determine the inner polygon (after headland contraction) ---------
  const double total_inset = static_cast<double>(headland_passes_) * headland_width_;
  Polygon2D inner = (total_inset > 1e-9) ? offset_polygon(outer, total_inset) : outer;

  if (inner.size() < 3) {
    // Headland completely consumed the area.
    response->success = false;
    response->message =
      "Headland contraction collapsed the inner area. "
      "Reduce headland_passes or headland_width.";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }

  // ---- Determine mowing angle ------------------------------------------
  double angle_rad = 0.0;
  if (request->mow_angle_deg < 0.0 && default_mow_angle_ < 0.0) {
    angle_rad = optimal_mowing_angle(inner);
    RCLCPP_INFO(get_logger(), "Auto-selected mowing angle: %.1f deg", angle_rad * 180.0 / M_PI);
  } else {
    const double deg =
      (request->mow_angle_deg >= 0.0) ? request->mow_angle_deg : default_mow_angle_;
    angle_rad = deg * M_PI / 180.0;
    RCLCPP_INFO(get_logger(), "Using mowing angle: %.1f deg", deg);
  }

  // ---- Generate swath path ---------------------------------------------
  const nav_msgs::msg::Path raw_path = generate_swath_path(inner, angle_rad, map_frame_);

  // Densify the path so consecutive waypoints are at most ~0.1 m apart.
  // This prevents RPP from skipping to the wrong swath via nearest-point matching.
  nav_msgs::msg::Path swath_path;
  swath_path.header = raw_path.header;
  constexpr double kMaxGap = 0.1;
  for (size_t i = 0; i < raw_path.poses.size(); ++i) {
    if (i > 0) {
      const auto& prev = raw_path.poses[i - 1].pose.position;
      const auto& curr = raw_path.poses[i].pose.position;
      const double dx = curr.x - prev.x;
      const double dy = curr.y - prev.y;
      const double dist = std::hypot(dx, dy);
      if (dist > kMaxGap) {
        const int n = static_cast<int>(std::ceil(dist / kMaxGap));
        for (int j = 1; j < n; ++j) {
          const double t = static_cast<double>(j) / n;
          geometry_msgs::msg::PoseStamped ps;
          ps.header = raw_path.poses[i].header;
          ps.pose.position.x = prev.x + t * dx;
          ps.pose.position.y = prev.y + t * dy;
          ps.pose.position.z = 0.0;
          ps.pose.orientation = raw_path.poses[i - 1].pose.orientation;
          swath_path.poses.push_back(ps);
        }
      }
    }
    swath_path.poses.push_back(raw_path.poses[i]);
  }
  RCLCPP_INFO(get_logger(),
    "Path densified: %zu -> %zu poses (max gap %.2f m)",
    raw_path.poses.size(), swath_path.poses.size(), kMaxGap);

  // ---- Populate response -----------------------------------------------
  response->success = true;
  response->message = "Coverage path generated successfully";
  response->path = swath_path;
  response->outline_path = outline_path;
  response->total_distance =
    compute_path_length(swath_path) + compute_path_length(outline_path);
  response->coverage_area = std::abs(polygon_area(inner));

  // Publish for visualisation.
  path_pub_->publish(swath_path);
  outline_pub_->publish(outline_path);

  RCLCPP_INFO(
    get_logger(),
    "Coverage plan: %zu swath poses, %.1f m total distance, %.2f m² inner area",
    swath_path.poses.size(),
    response->total_distance,
    response->coverage_area);
}

// ---------------------------------------------------------------------------
// generate_outline_path
// ---------------------------------------------------------------------------

nav_msgs::msg::Path CoveragePlannerNode::generate_outline_path(
  const Polygon2D & outer,
  const std::string & frame) const
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame;
  path.header.stamp = rclcpp::Clock().now();

  // Generate each successive headland ring by shrinking inward.
  for (int pass = 1; pass <= headland_passes_; ++pass) {
    const double inset = static_cast<double>(pass) * headland_width_;
    const Polygon2D ring = (pass == 1) ? offset_polygon(outer, headland_width_ * 0.5)
                                       : offset_polygon(outer, inset - headland_width_ * 0.5);

    if (ring.size() < 3) {
      break;  // contraction collapsed
    }

    // Trace the ring: add all vertices plus close back to the start.
    for (std::size_t i = 0; i < ring.size(); ++i) {
      const Point2D & curr = ring[i];
      const Point2D & next = ring[(i + 1) % ring.size()];
      const double yaw = std::atan2(next.second - curr.second, next.first - curr.first);
      path.poses.push_back(make_pose(curr.first, curr.second, yaw, frame));
    }
    // Close the ring.
    if (!ring.empty()) {
      const Point2D & last = ring.back();
      const Point2D & first = ring.front();
      const double yaw = std::atan2(first.second - last.second, first.first - last.first);
      path.poses.push_back(make_pose(last.first, last.second, yaw, frame));
    }
  }

  return path;
}

// ---------------------------------------------------------------------------
// generate_swath_path
// ---------------------------------------------------------------------------

nav_msgs::msg::Path CoveragePlannerNode::generate_swath_path(
  const Polygon2D & inner,
  double angle_rad,
  const std::string & frame) const
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame;
  path.header.stamp = rclcpp::Clock().now();

  if (inner.size() < 3) {
    return path;
  }

  // Rotate the polygon by -angle_rad so that swaths become horizontal (along X).
  const Point2D centroid = polygon_centroid(inner);
  const Polygon2D rotated_poly = rotate_polygon(inner, -angle_rad, centroid);

  // Find the bounding box of the rotated polygon.
  double y_min = std::numeric_limits<double>::max();
  double y_max = std::numeric_limits<double>::lowest();
  double x_min = std::numeric_limits<double>::max();
  double x_max = std::numeric_limits<double>::lowest();

  for (const auto & v : rotated_poly) {
    y_min = std::min(y_min, v.second);
    y_max = std::max(y_max, v.second);
    x_min = std::min(x_min, v.first);
    x_max = std::max(x_max, v.first);
  }

  // Extend the horizontal sweep lines slightly beyond the bounding box.
  const double x_margin = (x_max - x_min) * 0.1 + 1.0;

  // Generate horizontal sweep lines at path_spacing_ intervals.
  struct SwathLine
  {
    std::vector<Segment2D> segments;
    bool forward;  // direction of travel
  };

  std::vector<SwathLine> swaths;

  // Offset the first swath by half-spacing from y_min so that the outer edge
  // of the first swath aligns with the polygon boundary.
  double y_current = y_min + path_spacing_ * 0.5;
  bool forward = true;

  while (y_current <= y_max - path_spacing_ * 0.5 + 1e-9) {
    const Point2D sweep_start = {x_min - x_margin, y_current};
    const Point2D sweep_end = {x_max + x_margin, y_current};

    auto segments = clip_line_to_polygon(sweep_start, sweep_end, rotated_poly);

    if (!segments.empty()) {
      // Sort segments left-to-right so boustrophedon logic is deterministic.
      std::sort(segments.begin(), segments.end(), [](const Segment2D & a, const Segment2D & b) {
        return a.start.first < b.start.first;
      });
      swaths.push_back({std::move(segments), forward});
      forward = !forward;
    }

    y_current += path_spacing_;
  }

  // Rotate swath segments back by +angle_rad and build pose list.
  const double cos_a = std::cos(angle_rad);
  const double sin_a = std::sin(angle_rad);

  auto rotate_point_back = [&](const Point2D & p) -> Point2D {
      const double tx = p.first - centroid.first;
      const double ty = p.second - centroid.second;
      return {
        centroid.first + cos_a * tx - sin_a * ty,
        centroid.second + sin_a * tx + cos_a * ty
      };
    };

  // Number of intermediate waypoints for U-turns between swaths.
  // A semicircle is divided into this many steps.
  constexpr int turn_steps = 6;
  // Turn radius: half the spacing between adjacent swaths.
  const double turn_radius = path_spacing_ * 0.5;

  for (std::size_t swath_idx = 0; swath_idx < swaths.size(); ++swath_idx) {
    const auto & swath = swaths[swath_idx];
    const std::size_t seg_count = swath.segments.size();

    for (std::size_t si = 0; si < seg_count; ++si) {
      const std::size_t idx = swath.forward ? si : (seg_count - 1 - si);
      const Segment2D & seg = swath.segments[idx];

      Point2D world_start, world_end;
      if (swath.forward) {
        world_start = rotate_point_back(seg.start);
        world_end = rotate_point_back(seg.end);
      } else {
        world_start = rotate_point_back(seg.end);
        world_end = rotate_point_back(seg.start);
      }

      const double yaw_fwd = angle_rad;
      const double yaw_bwd = angle_rad + M_PI;
      const double yaw = swath.forward ? yaw_fwd : yaw_bwd;

      path.poses.push_back(make_pose(world_start.first, world_start.second, yaw, frame));
      path.poses.push_back(make_pose(world_end.first, world_end.second, yaw, frame));
    }

    // Insert semicircular U-turn waypoints between this swath and the next.
    if (swath_idx + 1 < swaths.size()) {
      const auto & prev_pose = path.poses.back().pose;
      const double end_x = prev_pose.position.x;
      const double end_y = prev_pose.position.y;

      // Determine direction: which side does the next swath start?
      const auto & next_swath = swaths[swath_idx + 1];
      const auto & first_seg = next_swath.forward
        ? next_swath.segments.front()
        : next_swath.segments.back();
      const Point2D next_start_rot = next_swath.forward
        ? first_seg.start : first_seg.end;
      const Point2D next_start_world = rotate_point_back(next_start_rot);

      // Center of the semicircle is midpoint between end of current and start of next.
      const double cx = (end_x + next_start_world.first) * 0.5;
      const double cy = (end_y + next_start_world.second) * 0.5;

      // Angle from center to end of current swath.
      const double start_angle = std::atan2(end_y - cy, end_x - cx);
      const double actual_radius = std::hypot(end_x - cx, end_y - cy);

      // Generate semicircular arc from current end to next start.
      // Direction of arc: away from the mowing area (outward turn).
      for (int step = 1; step < turn_steps; ++step) {
        const double t = static_cast<double>(step) / turn_steps;
        const double theta = start_angle + M_PI * t;  // semicircle
        const double wx = cx + actual_radius * std::cos(theta);
        const double wy = cy + actual_radius * std::sin(theta);
        // Tangent direction along the arc
        const double tangent_yaw = theta + M_PI * 0.5;
        path.poses.push_back(make_pose(wx, wy, tangent_yaw, frame));
      }
    }
  }

  return path;
}

// ---------------------------------------------------------------------------
// compute_path_length
// ---------------------------------------------------------------------------

double CoveragePlannerNode::compute_path_length(const nav_msgs::msg::Path & path)
{
  double total = 0.0;
  for (std::size_t i = 1; i < path.poses.size(); ++i) {
    const auto & a = path.poses[i - 1].pose.position;
    const auto & b = path.poses[i].pose.position;
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    total += std::hypot(dx, dy);
  }
  return total;
}

// ---------------------------------------------------------------------------
// make_pose
// ---------------------------------------------------------------------------

geometry_msgs::msg::PoseStamped CoveragePlannerNode::make_pose(
  double x, double y, double yaw, const std::string & frame)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame;
  pose.header.stamp = rclcpp::Clock().now();
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  pose.pose.orientation = tf2::toMsg(q);

  return pose;
}

}  // namespace mowgli_coverage_planner
