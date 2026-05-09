#!/usr/bin/env python3
# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0-or-later

"""
sim_cmd_vel_unstamp.py — SIMULATION ONLY

TwistStamped → Twist relay for the Gazebo Ignition cmd_vel bridge.

Since the Kilted migration (commit c687b048), the entire Mowgli stack
publishes TwistStamped on the cmd_vel chain (Nav2 controllers,
behavior_server, twist_mux output). Gazebo's `gz.msgs.Twist` is
unstamped, and `ros_gz_bridge` only maps `geometry_msgs/Twist` ↔
`gz.msgs.Twist`, so the stamped messages cannot reach the diff-drive
plugin directly. This relay bridges the two.

Wiring (sim only)
-----------------
  twist_mux output  -> /cmd_vel_stamped (TwistStamped, this node's input)
  this node         -> /cmd_vel         (Twist, gz_ros2_bridge's input)
  gz_ros2_bridge    -> Gazebo /cmd_vel  (gz.msgs.Twist)

In production, twist_mux publishes /cmd_vel directly as TwistStamped to
hardware_bridge — no relay needed.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class CmdVelUnstamp(Node):
    def __init__(self) -> None:
        super().__init__("sim_cmd_vel_unstamp")
        self.declare_parameter("input_topic", "/cmd_vel_stamped")
        self.declare_parameter("output_topic", "/cmd_vel")
        in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value

        self._pub = self.create_publisher(Twist, out_topic, 10)
        self._sub = self.create_subscription(
            TwistStamped, in_topic, self._on_stamped, 10
        )
        self.get_logger().info(
            f"sim_cmd_vel_unstamp ready: {in_topic} (TwistStamped) -> {out_topic} (Twist)"
        )

    def _on_stamped(self, msg: TwistStamped) -> None:
        self._pub.publish(msg.twist)


def main() -> None:
    rclpy.init()
    node = CmdVelUnstamp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
