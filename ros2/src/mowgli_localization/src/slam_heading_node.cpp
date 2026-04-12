// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
/**
 * @file slam_heading_node.cpp
 * @brief Extracts heading from SLAM's slam_map→odom TF for EKF fusion.
 *
 * SLAM publishes slam_map→odom TF (separate from ekf_map's map→odom).
 * This node reads that TF and publishes the yaw as a heading measurement
 * for ekf_map to fuse, providing absolute heading from LiDAR features.
 */

#include "mowgli_localization/slam_heading_node.hpp"

#include <cmath>

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace mowgli_localization
{

SlamHeadingNode::SlamHeadingNode(const rclcpp::NodeOptions& options) : Node("slam_heading", options)
{
  publish_rate_ = declare_parameter<double>("publish_rate", 5.0);
  yaw_variance_ = declare_parameter<double>("yaw_variance", 0.05);
  stale_timeout_ = declare_parameter<double>("stale_timeout", 5.0);
  stale_yaw_variance_ = declare_parameter<double>("stale_yaw_variance", 1e6);
  slam_frame_ = declare_parameter<std::string>("slam_frame", "slam_map");

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  heading_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/slam/heading", rclcpp::QoS(10));

  timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / publish_rate_),
                             std::bind(&SlamHeadingNode::timer_callback, this));

  RCLCPP_INFO(get_logger(),
              "SlamHeadingNode started (reading %s→odom TF, rate=%.1f Hz, yaw_var=%.3f)",
              slam_frame_.c_str(),
              publish_rate_,
              yaw_variance_);
}

void SlamHeadingNode::timer_callback()
{
  geometry_msgs::msg::TransformStamped tf;
  try
  {
    tf = tf_buffer_->lookupTransform(slam_frame_, "odom", tf2::TimePointZero);
  }
  catch (const tf2::TransformException&)
  {
    return;
  }

  const rclcpp::Time tf_stamp(tf.header.stamp);
  const double age = (now() - tf_stamp).seconds();
  const double yaw = tf2::getYaw(tf.transform.rotation);

  double variance = yaw_variance_;
  if (age > stale_timeout_)
  {
    variance = stale_yaw_variance_;
  }
  else if (age > stale_timeout_ * 0.5)
  {
    const double alpha = (age - stale_timeout_ * 0.5) / (stale_timeout_ * 0.5);
    variance = yaw_variance_ + alpha * (stale_yaw_variance_ - yaw_variance_);
  }

  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.stamp = now();
  msg.header.frame_id = "map";

  msg.pose.pose.orientation.w = std::cos(yaw / 2.0);
  msg.pose.pose.orientation.z = std::sin(yaw / 2.0);

  msg.pose.covariance[0] = 1e6;
  msg.pose.covariance[7] = 1e6;
  msg.pose.covariance[14] = 1e6;
  msg.pose.covariance[21] = 1e6;
  msg.pose.covariance[28] = 1e6;
  msg.pose.covariance[35] = variance;

  heading_pub_->publish(msg);
}

}  // namespace mowgli_localization

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mowgli_localization::SlamHeadingNode>());
  rclcpp::shutdown();
  return 0;
}
