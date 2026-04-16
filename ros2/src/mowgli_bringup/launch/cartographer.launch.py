# Copyright 2026 Mowgli Project
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

"""
cartographer.launch.py

Launches Google Cartographer as an alternative SLAM backend.

Publishes map→odom TF (replacing slam_toolbox's slam_map→odom).
FusionCore publishes odom→base_footprint with GPS enabled.
No GPS-SLAM corrector is needed — GPS is fused directly in FusionCore.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup_dir = get_package_share_directory("mowgli_bringup")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
    )

    resolution_arg = DeclareLaunchArgument(
        "resolution",
        default_value="0.05",
        description="Resolution of the occupancy grid published by Cartographer.",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    resolution = LaunchConfiguration("resolution")

    cartographer_config_dir = os.path.join(bringup_dir, "config")

    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "-configuration_directory", cartographer_config_dir,
            "-configuration_basename", "cartographer.lua",
        ],
        remappings=[
            ("scan", "/scan"),
            ("imu", "/imu/data"),
            ("odom", "/fusion/odom"),
        ],
    )

    occupancy_grid_node = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        name="cartographer_occupancy_grid_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"resolution": resolution},
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        resolution_arg,
        cartographer_node,
        occupancy_grid_node,
    ])
