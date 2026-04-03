// Copyright (C) 2024 Cedric
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * @file main.cpp
 * @brief Entry point for the coverage_planner_node executable.
 */

#include "mowgli_coverage_planner/coverage_planner_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  const auto node = std::make_shared<mowgli_coverage_planner::CoveragePlannerNode>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
