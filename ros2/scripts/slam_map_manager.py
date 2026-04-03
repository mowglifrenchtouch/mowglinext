#!/usr/bin/env python3
"""
slam_map_manager.py

CLI utility for saving and loading slam_toolbox pose graphs.

Usage:
    python3 slam_map_manager.py save [--path /ros2_ws/maps/garden_map]
    python3 slam_map_manager.py load [--path /ros2_ws/maps/garden_map]

Services used:
    /slam_toolbox/serialize_map    (slam_toolbox/srv/SerializePoseGraph)
    /slam_toolbox/deserialize_map  (slam_toolbox/srv/DeserializePoseGraph)

The slam_toolbox service stores two files on disk:
    <path>.posegraph
    <path>.data

For localization mode the match_type must be
LOCALIZATION_POSE_GRAPH = 2 so that slam_toolbox switches its internal
pose graph to read-only / localisation behaviour.
"""

import argparse
import os
import sys

import rclpy
from rclpy.node import Node
from slam_toolbox.srv import DeserializePoseGraph, SerializePoseGraph


# ---------------------------------------------------------------------------
# Constants mirroring slam_toolbox/DeserializePoseGraph.srv
# ---------------------------------------------------------------------------
START_AT_FIRST_NODE = 0
START_AT_GIVEN_POSE = 1
LOCALIZATION_POSE_GRAPH = 2

DEFAULT_MAP_PATH = "/ros2_ws/maps/garden_map"
SERVICE_TIMEOUT_SEC = 10.0


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------


class SlamMapManager(Node):
    """Thin ROS 2 node that wraps serialize/deserialize service calls."""

    def __init__(self) -> None:
        super().__init__("slam_map_manager")

        self._save_client = self.create_client(
            SerializePoseGraph, "/slam_toolbox/serialize_map"
        )
        self._load_client = self.create_client(
            DeserializePoseGraph, "/slam_toolbox/deserialize_map"
        )

    # ------------------------------------------------------------------
    # Public helpers
    # ------------------------------------------------------------------

    def save(self, path: str) -> bool:
        """Serialize the current pose graph to *path* (no extension).

        Returns True on success, False on failure.
        """
        self._ensure_directory(path)

        self.get_logger().info(f"Waiting for serialize_map service …")
        if not self._save_client.wait_for_service(timeout_sec=SERVICE_TIMEOUT_SEC):
            self.get_logger().error(
                "/slam_toolbox/serialize_map not available after "
                f"{SERVICE_TIMEOUT_SEC:.0f} s"
            )
            return False

        request = SerializePoseGraph.Request()
        request.filename = path

        self.get_logger().info(f"Saving SLAM map to '{path}' …")
        future = self._save_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=SERVICE_TIMEOUT_SEC)

        if not future.done():
            self.get_logger().error("serialize_map call timed out")
            return False

        response = future.result()
        if response is None:
            self.get_logger().error("serialize_map returned None response")
            return False

        # The service has a `result` field: 0 = success (RESULT_SUCCESS)
        if response.result != 0:
            self.get_logger().error(
                f"serialize_map failed with result code {response.result}"
            )
            return False

        self.get_logger().info(f"SLAM map saved successfully to '{path}'")
        return True

    def load(self, path: str) -> bool:
        """Deserialize a saved pose graph from *path* in localization mode.

        Returns True on success, False on failure.
        """
        posegraph_file = path + ".posegraph"
        if not os.path.exists(posegraph_file):
            self.get_logger().error(
                f"Map file not found: '{posegraph_file}'. "
                "Run 'save' first or verify the path."
            )
            return False

        self.get_logger().info("Waiting for deserialize_map service …")
        if not self._load_client.wait_for_service(timeout_sec=SERVICE_TIMEOUT_SEC):
            self.get_logger().error(
                "/slam_toolbox/deserialize_map not available after "
                f"{SERVICE_TIMEOUT_SEC:.0f} s"
            )
            return False

        request = DeserializePoseGraph.Request()
        request.filename = path
        request.match_type = LOCALIZATION_POSE_GRAPH
        # initial_pose is ignored when match_type = LOCALIZATION_POSE_GRAPH
        request.initial_pose.x = 0.0
        request.initial_pose.y = 0.0
        request.initial_pose.theta = 0.0

        self.get_logger().info(
            f"Loading SLAM map from '{path}' (match_type=LOCALIZATION_POSE_GRAPH) …"
        )
        future = self._load_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=SERVICE_TIMEOUT_SEC)

        if not future.done():
            self.get_logger().error("deserialize_map call timed out")
            return False

        response = future.result()
        if response is None:
            self.get_logger().error("deserialize_map returned None response")
            return False

        if response.result != 0:
            self.get_logger().error(
                f"deserialize_map failed with result code {response.result}"
            )
            return False

        self.get_logger().info(f"SLAM map loaded successfully from '{path}'")
        return True

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _ensure_directory(path: str) -> None:
        """Create the parent directory of *path* if it does not exist."""
        directory = os.path.dirname(path)
        if directory and not os.path.exists(directory):
            os.makedirs(directory, exist_ok=True)


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Save or load a slam_toolbox pose graph."
    )
    parser.add_argument(
        "command",
        choices=["save", "load"],
        help="'save' serializes the running map; 'load' deserializes into localization mode.",
    )
    parser.add_argument(
        "--path",
        default=DEFAULT_MAP_PATH,
        help=f"Base path without extension (default: {DEFAULT_MAP_PATH}).",
    )
    args = parser.parse_args()

    rclpy.init()
    node = SlamMapManager()

    try:
        if args.command == "save":
            success = node.save(args.path)
        else:
            success = node.load(args.path)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
