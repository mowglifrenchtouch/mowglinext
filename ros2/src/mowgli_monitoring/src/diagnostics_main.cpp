// SPDX-License-Identifier: GPL-3.0
/**
 * @file diagnostics_main.cpp
 * @brief Entry point for the diagnostics_node executable.
 */

#include "mowgli_monitoring/diagnostics_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mowgli_monitoring::DiagnosticsNode>());
  rclcpp::shutdown();
  return 0;
}
