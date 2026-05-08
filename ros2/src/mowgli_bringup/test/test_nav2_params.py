# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0
#
# Regression tests for nav2_params.yaml configuration. These guard
# against the class of bug where a goal_checker tolerance is set so
# loose that the controller silently reports SUCCEEDED on the first
# tick, the BT loops GetNextSegment forever, and the robot never moves.
# See: 2026-05-08 field incident — coverage_goal_checker.xy_goal_tolerance
# was 0.5 m, which made every <0.5 m strip "already done" the moment
# FTC started.
"""Regression tests for the nav2_params.yaml goal-checker tolerances."""
import os

import pytest
import yaml


def _load_params() -> dict:
    here = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(here, "..", "config", "nav2_params.yaml")
    with open(path, "r", encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def _controller_section(params: dict) -> dict:
    return params["controller_server"]["ros__parameters"]


def test_coverage_goal_checker_xy_tolerance_is_tight() -> None:
    """coverage_goal_checker.xy_goal_tolerance must be <= mower_width.

    SimpleGoalChecker with stateful=true returns SUCCEEDED on the first
    tick where the robot is within xy_goal_tolerance of the goal pose.
    For cell-based mowing the strips are short (<3 m, often <0.5 m near
    the robot's current pose). If the tolerance is 0.5 m, every strip is
    "already done" before FTC publishes any cmd_vel, the controller
    returns SUCCEEDED instantly, and the BT loops forever without
    moving the robot.

    Anything above mower_width (0.18 m) means the controller can decide
    a strip is finished before driving even one full cell — regressions
    bumping this back fail the build.
    """
    cfg = _controller_section(_load_params())
    tol = cfg["coverage_goal_checker"]["xy_goal_tolerance"]
    # Use 0.20 m as the upper bound: 0.18 m mower_width + a bit of slack.
    assert tol <= 0.20, (
        f"coverage_goal_checker.xy_goal_tolerance={tol} m is too loose. "
        "FTC will report SUCCEEDED on the first tick before the robot moves; "
        "the BT will loop GetNextSegment forever. Tighten to <= mower_width."
    )


def test_coverage_goal_checker_is_stateful() -> None:
    """stateful=false would re-check tolerance every tick, defeating
    the 'commit to the goal once reached' semantic FTC needs. If the
    robot drifts during the post-success blade engagement, a non-stateful
    checker would re-fire failure. Pin it true.
    """
    cfg = _controller_section(_load_params())
    assert cfg["coverage_goal_checker"]["stateful"] is True


def test_stopped_goal_checker_velocity_threshold_is_set() -> None:
    """StoppedGoalChecker without trans_stopped_velocity defaults to
    a permissive threshold; pin a value so the check is deterministic.
    """
    cfg = _controller_section(_load_params())
    assert "trans_stopped_velocity" in cfg["stopped_goal_checker"]
    assert cfg["stopped_goal_checker"]["trans_stopped_velocity"] > 0.0


def test_followpath_uses_rotation_shim() -> None:
    """The transit/dock controller wraps RPP in RotationShimController so
    big heading errors get an in-place rotate before driving. If this
    pin slips, RPP starts driving an arc immediately and the robot
    spirals away from its first carrot.
    """
    cfg = _controller_section(_load_params())
    assert (
        cfg["FollowPath"]["plugin"]
        == "nav2_rotation_shim_controller::RotationShimController"
    )


def test_followcoveragepath_uses_ftc() -> None:
    """The coverage controller MUST be FTCController. Switching back to
    RPP/MPPI breaks adjacent-row spacing accuracy (CLAUDE.md invariant
    #8) — pin it explicitly.
    """
    cfg = _controller_section(_load_params())
    assert cfg["FollowCoveragePath"]["plugin"] == "mowgli_nav2_plugins/FTCController"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
