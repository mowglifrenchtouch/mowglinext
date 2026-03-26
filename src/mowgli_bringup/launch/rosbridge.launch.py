"""
rosbridge.launch.py

Starts the rosbridge_server WebSocket bridge for the openmower-gui.

Brings up:
  1. rosbridge_websocket — WebSocket server exposing ROS2 topics/services/params
  2. rosapi             — Required companion node for service/param introspection
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # ------------------------------------------------------------------
    # Package directories
    # ------------------------------------------------------------------
    bringup_dir = get_package_share_directory("mowgli_bringup")

    # ------------------------------------------------------------------
    # Declared arguments
    # ------------------------------------------------------------------
    port_arg = DeclareLaunchArgument(
        "port",
        default_value="9090",
        description="Port number for the rosbridge WebSocket server.",
    )

    max_message_size_arg = DeclareLaunchArgument(
        "max_message_size",
        default_value="10000000",
        description="Maximum size in bytes of a single WebSocket message.",
    )

    authenticate_arg = DeclareLaunchArgument(
        "authenticate",
        default_value="false",
        description="Require authentication for rosbridge connections when true.",
    )

    # ------------------------------------------------------------------
    # Resolved substitutions
    # ------------------------------------------------------------------
    port = LaunchConfiguration("port")
    max_message_size = LaunchConfiguration("max_message_size")
    authenticate = LaunchConfiguration("authenticate")

    # ------------------------------------------------------------------
    # Config path
    # ------------------------------------------------------------------
    rosbridge_params = os.path.join(bringup_dir, "config", "rosbridge.yaml")

    # ------------------------------------------------------------------
    # 1. rosbridge_websocket
    # ------------------------------------------------------------------
    rosbridge_node = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        output="screen",
        parameters=[
            rosbridge_params,
            {
                "port": port,
                "max_message_size": max_message_size,
                "authenticate": authenticate,
            },
        ],
    )

    # ------------------------------------------------------------------
    # 2. rosapi — required companion for service/param introspection
    # ------------------------------------------------------------------
    rosapi_node = Node(
        package="rosapi",
        executable="rosapi_node",
        name="rosapi",
        output="screen",
    )

    # ------------------------------------------------------------------
    # LaunchDescription
    # ------------------------------------------------------------------
    return LaunchDescription(
        [
            port_arg,
            max_message_size_arg,
            authenticate_arg,
            rosbridge_node,
            rosapi_node,
        ]
    )
