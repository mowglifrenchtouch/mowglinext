// SPDX-License-Identifier: GPL-3.0
/**
 * @file mqtt_bridge_main.cpp
 * @brief Entry point for the mqtt_bridge_node executable.
 */

#include "mowgli_monitoring/mqtt_bridge_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mowgli_monitoring::MqttBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
