// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
/**
 * @file slam_heading_node.hpp
 * @brief Extracts heading from SLAM's /pose topic for EKF fusion.
 *
 * SLAM publishes /pose independently of TF (even with transform_publish_period=0).
 * This node extracts yaw and republishes for ekf_map heading correction.
 */

#pragma once

#include <memory>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mowgli_localization
{

class SlamHeadingNode : public rclcpp::Node
{
public:
  explicit SlamHeadingNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~SlamHeadingNode() override = default;

private:
  void on_slam_pose(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);

  double yaw_variance_{0.05};
  double stale_timeout_{5.0};
  double stale_yaw_variance_{1e6};

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr heading_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr slam_pose_sub_;
};

}  // namespace mowgli_localization
