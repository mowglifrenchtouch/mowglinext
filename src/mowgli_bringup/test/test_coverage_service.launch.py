"""
test_coverage_service.launch.py

launch_testing integration test for the CoveragePlannerNode service.

Test cases:
  - test_plan_coverage_success    – call ~/plan_coverage with a 10 x 10 m
                                    square polygon; verify success=True,
                                    path contains poses, total_distance > 0.
  - test_skip_outline             – call with skip_outline=True; verify the
                                    response carries an empty outline_path.
"""

import math
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from geometry_msgs.msg import Point32, Polygon
from mowgli_interfaces.srv import PlanCoveragePath


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _square_polygon(side_m: float = 10.0) -> Polygon:
    """Return a closed CCW square polygon with the given side length."""
    half = side_m / 2.0
    poly = Polygon()
    poly.points = [
        Point32(x=float(-half), y=float(-half), z=0.0),
        Point32(x=float(half),  y=float(-half), z=0.0),
        Point32(x=float(half),  y=float(half),  z=0.0),
        Point32(x=float(-half), y=float(half),  z=0.0),
    ]
    return poly


def _call_service(
    node: rclpy.node.Node,
    client: rclpy.client.Client,
    request: PlanCoveragePath.Request,
    timeout_sec: float = 15.0,
) -> PlanCoveragePath.Response:
    """Synchronously call the service and return the response."""
    if not client.wait_for_service(timeout_sec=timeout_sec):
        raise RuntimeError(
            f"Service {client.srv_name!r} not available after {timeout_sec} s"
        )

    future = client.call_async(request)
    deadline = time.monotonic() + timeout_sec

    while not future.done():
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.monotonic() > deadline:
            raise RuntimeError(
                f"Service call to {client.srv_name!r} did not complete within "
                f"{timeout_sec} s"
            )

    return future.result()


# ---------------------------------------------------------------------------
# Launch description
# ---------------------------------------------------------------------------

@pytest.mark.launch_test
def generate_test_description():
    coverage_planner = launch_ros.actions.Node(
        package="mowgli_coverage_planner",
        executable="coverage_planner_node",
        name="coverage_planner_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "tool_width": 0.18,
                "headland_passes": 1,
                "headland_width": 0.18,
                "path_spacing": 0.18,
                "map_frame": "map",
            }
        ],
    )

    return (
        launch.LaunchDescription(
            [
                coverage_planner,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {"coverage_planner": coverage_planner},
    )


# ---------------------------------------------------------------------------
# Active-state tests
# ---------------------------------------------------------------------------

class TestCoverageService(unittest.TestCase):
    """Integration tests for the PlanCoveragePath service."""

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()
        cls.node = rclpy.create_node("test_coverage_service_helper")
        cls.client = cls.node.create_client(
            PlanCoveragePath,
            "/coverage_planner_node/plan_coverage",
        )

    @classmethod
    def tearDownClass(cls) -> None:
        cls.node.destroy_node()
        rclpy.shutdown()

    # ------------------------------------------------------------------
    # test_plan_coverage_success
    # ------------------------------------------------------------------

    def test_plan_coverage_success(self) -> None:
        """
        A 10 x 10 m square polygon must produce a successful response with
        at least one pose in the coverage path and a positive total distance.
        """
        request = PlanCoveragePath.Request()
        request.outer_boundary = _square_polygon(side_m=10.0)
        request.obstacles = []
        request.mow_angle_deg = -1.0   # auto-optimise
        request.skip_outline = False

        response = _call_service(self.node, self.client, request)

        self.assertTrue(
            response.success,
            msg=f"service returned success=False: {response.message}",
        )
        self.assertGreater(
            len(response.path.poses),
            0,
            msg="coverage path must contain at least one pose",
        )
        self.assertGreater(
            response.total_distance,
            0.0,
            msg="total_distance must be positive",
        )

    # ------------------------------------------------------------------
    # test_skip_outline
    # ------------------------------------------------------------------

    def test_skip_outline(self) -> None:
        """
        When skip_outline=True the response must succeed and the
        outline_path must be empty (zero poses).
        """
        request = PlanCoveragePath.Request()
        request.outer_boundary = _square_polygon(side_m=10.0)
        request.obstacles = []
        request.mow_angle_deg = -1.0
        request.skip_outline = True

        response = _call_service(self.node, self.client, request)

        self.assertTrue(
            response.success,
            msg=f"service returned success=False with skip_outline=True: "
                f"{response.message}",
        )
        self.assertEqual(
            len(response.outline_path.poses),
            0,
            msg="outline_path must be empty when skip_outline=True",
        )


# ---------------------------------------------------------------------------
# Post-shutdown tests
# ---------------------------------------------------------------------------

@launch_testing.post_shutdown_test()
class TestCoverageServiceShutdown(unittest.TestCase):
    """Verify the coverage_planner_node process exited cleanly."""

    def test_exit_code(self, proc_info, coverage_planner) -> None:
        launch_testing.asserts.assertExitCodes(
            proc_info,
            process=coverage_planner,
            allowable_exit_codes=[0],
        )
