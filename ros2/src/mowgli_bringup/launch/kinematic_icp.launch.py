# Copyright 2026 Mowgli Project
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.


"""
kinematic_icp.launch.py

Starts the Kinematic-ICP LiDAR odometry pipeline for a 2D LD19 LiDAR:

  1. wheel_odom_tf_node (mowgli_localization)
     - /wheel_odom (nav_msgs/Odometry, twist-only)
     -> TF: wheel_odom_raw -> base_footprint_wheels
     Publishes a raw-wheels TF on an ISOLATED frame pair to serve as
     Kinematic-ICP's motion prior — independent of FusionCore's
     odom -> base_footprint TF, so the encoder2 feedback path can't
     form a loop.

  2. kinematic_icp_online_node (kinematic_icp pkg, upstream)
     - /scan (sensor_msgs/LaserScan) — native 2D input, no external
       laserscan_to_pointcloud converter needed (use_2d_lidar: true).
     - looks up wheel_odom_raw -> base_footprint_wheels as motion prior.
     -> /kinematic_icp/lidar_odometry (nav_msgs/Odometry)
     - publish_odom_tf: False    (FusionCore owns odom -> base_footprint;
                                  Kinematic-ICP MUST NOT publish TF here)

  3. kinematic_icp_encoder_adapter (mowgli_localization)
     - /kinematic_icp/lidar_odometry  ->  finite-difference body-frame twist
     -> /encoder2/odom                     (FusionCore's encoder2 slot)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup_dir = get_package_share_directory("mowgli_bringup")
    kicp_config = os.path.join(bringup_dir, "config", "kinematic_icp.yaml")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock when true.",
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    # ------------------------------------------------------------------
    # 1. Raw-wheel-only TF (isolates Kinematic-ICP's motion prior from
    #    FusionCore's fused odom TF, breaking the feedback loop).
    # ------------------------------------------------------------------
    wheel_odom_tf = Node(
        package="mowgli_localization",
        executable="wheel_odom_tf_node.py",
        name="wheel_odom_tf_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "input_topic": "/wheel_odom",
                "parent_frame": "wheel_odom_raw",
                "child_frame": "base_footprint_wheels",
            }
        ],
    )

    # ------------------------------------------------------------------
    # 2. Kinematic-ICP online node.
    #    - input 'lidar_topic' = /scan (LaserScan, use_2d_lidar=true)
    #    - output remapped to /kinematic_icp/lidar_odometry
    #    - publish_odom_tf: False (FusionCore owns odom -> base_footprint)
    # ------------------------------------------------------------------
    kinematic_icp_node = Node(
        package="kinematic_icp",
        executable="kinematic_icp_online_node",
        name="kinematic_icp_online_node",
        namespace="kinematic_icp",
        output="screen",
        parameters=[
            kicp_config,
            {
                "use_sim_time": use_sim_time,
                "lidar_topic": "/scan",
                "use_2d_lidar": True,
                # Motion prior frames come from wheel_odom_tf_node above.
                "wheel_odom_frame": "wheel_odom_raw",
                "base_frame": "base_footprint_wheels",
                # Kinematic-ICP publishes lidar_odom_kicp -> base_footprint_wheels
                # internally only; this frame is not consumed by anything else.
                "lidar_odom_frame": "lidar_odom_kicp",
                "publish_odom_tf": False,
                "invert_odom_tf": False,
                "tf_timeout": 0.0,
            },
        ],
        remappings=[
            ("lidar_odometry", "/kinematic_icp/lidar_odometry"),
        ],
    )

    # ------------------------------------------------------------------
    # 3. Encoder-twist adapter: pose-only KICP odometry -> FusionCore
    #    encoder2 twist.
    # ------------------------------------------------------------------
    kicp_encoder_adapter = Node(
        package="mowgli_localization",
        executable="kinematic_icp_encoder_adapter.py",
        name="kinematic_icp_encoder_adapter",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "input_topic": "/kinematic_icp/lidar_odometry",
                "output_topic": "/encoder2/odom",
            }
        ],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            wheel_odom_tf,
            kinematic_icp_node,
            kicp_encoder_adapter,
        ]
    )
