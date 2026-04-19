#!/usr/bin/env python3
# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0-or-later

"""
calibrate_imu_yaw_node.py

Autonomously estimates the IMU chip's mounting yaw relative to base_link.

The WT901 is physically mounted rotated around Z by `imu_yaw` relative to
base_link. Pure rotation does not reveal this angle (gyro_z is invariant
to yaw mount rotation around the common Z axis). Pure linear acceleration
in base_link +X DOES reveal it: accel observed in the chip frame is
    (a * cos(imu_yaw), -a * sin(imu_yaw), g)
so for a non-zero body acceleration a,
    imu_yaw = atan2(-ay_chip, ax_chip)

When the /calibrate service is invoked, this node DRIVES THE ROBOT itself:
forward with a ramp-cruise-ramp profile, pause, then back to start. Users
need only call the service with the robot undocked; no manual teleop.

Safety: the service refuses to run when the robot is charging (on dock)
or when an emergency is active. The collision_monitor stays in the
pipeline, so physical obstacles stop the motion regardless.
"""

import math
import threading
import time

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

from mowgli_interfaces.msg import Emergency
from mowgli_interfaces.msg import Status as HwStatus
from mowgli_interfaces.srv import CalibrateImuYaw


def _stamp_to_float(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class ImuYawCalibrator(Node):
    """Collects IMU + wheel odom samples during an autonomous drive profile
    and computes imu_yaw from the horizontal accel direction in chip frame."""

    # --- Sample filter thresholds ---
    WZ_STRAIGHT_THRESHOLD = 0.05        # rad/s — |wz| below ⇒ straight motion
    ACCEL_BODY_THRESHOLD = 0.1          # m/s² — |a_body| above ⇒ useful sample
    MIN_SAMPLES = 50

    # --- Motion profile ---
    # Forward 0.6 m + back 0.6 m = robot returns near its start.
    # Ramp 0→cruise at 0.2 m/s² (well above the 0.1 m/s² accel threshold).
    CRUISE_SPEED = 0.2                  # m/s
    RAMP_SEC = 1.0                      # acceleration and deceleration durations
    CRUISE_SEC = 2.0                    # constant-speed segment
    PAUSE_SEC = 1.0                     # between forward and backward
    CMD_RATE_HZ = 20.0                  # cmd_vel publish rate (firmware 200 ms watchdog)
    SETTLE_SEC = 1.0                    # post-motion settle before closing window

    def __init__(self) -> None:
        super().__init__("calibrate_imu_yaw_node")

        self._cb_group = ReentrantCallbackGroup()

        imu_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )
        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._lock = threading.Lock()
        self._collecting = False
        self._imu_samples: list[tuple[float, float, float]] = []
        self._odom_samples: list[tuple[float, float, float]] = []

        # Preflight state (latest values from safety topics).
        self._is_charging = False
        self._emergency_active = False

        self._imu_sub = self.create_subscription(
            Imu, "/imu/data", self._imu_cb, imu_qos, callback_group=self._cb_group
        )
        self._odom_sub = self.create_subscription(
            Odometry, "/wheel_odom", self._odom_cb, odom_qos,
            callback_group=self._cb_group
        )
        self._status_sub = self.create_subscription(
            HwStatus, "/hardware_bridge/status", self._status_cb, state_qos,
            callback_group=self._cb_group,
        )
        self._emergency_sub = self.create_subscription(
            Emergency, "/hardware_bridge/emergency", self._emergency_cb, state_qos,
            callback_group=self._cb_group,
        )

        # Teleop-priority velocity command. Matches the GUI joystick path so
        # the twist_mux gives it the right priority and all downstream
        # safety (collision_monitor, velocity_smoother) still applies.
        self._cmd_pub = self.create_publisher(
            TwistStamped, "/cmd_vel_teleop", state_qos
        )

        self._srv = self.create_service(
            CalibrateImuYaw,
            "~/calibrate",
            self._calibrate_cb,
            callback_group=self._cb_group,
        )

        self.get_logger().info(
            "IMU yaw calibration node ready. Ensure robot is undocked with "
            "~1 m of clear space in front and behind, then call ~/calibrate."
        )

    # ------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------

    def _imu_cb(self, msg: Imu) -> None:
        if not self._collecting:
            return
        t = _stamp_to_float(msg.header.stamp)
        with self._lock:
            self._imu_samples.append(
                (t, float(msg.linear_acceleration.x), float(msg.linear_acceleration.y))
            )

    def _odom_cb(self, msg: Odometry) -> None:
        if not self._collecting:
            return
        t = _stamp_to_float(msg.header.stamp)
        with self._lock:
            self._odom_samples.append(
                (t, float(msg.twist.twist.linear.x), float(msg.twist.twist.angular.z))
            )

    def _status_cb(self, msg: HwStatus) -> None:
        self._is_charging = bool(msg.is_charging)

    def _emergency_cb(self, msg: Emergency) -> None:
        self._emergency_active = bool(msg.active_emergency or msg.latched_emergency)

    # ------------------------------------------------------------------
    # Autonomous drive
    # ------------------------------------------------------------------

    def _publish_vx(self, vx: float) -> None:
        """Publish a stamped Twist with the given forward velocity."""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_footprint"
        msg.twist.linear.x = float(vx)
        # All other components stay at 0.
        self._cmd_pub.publish(msg)

    def _drive_profile(self, signed_cruise_speed: float) -> None:
        """Run a ramp-cruise-ramp velocity profile at ±CRUISE_SPEED.

        signed_cruise_speed: +CRUISE_SPEED for forward, -CRUISE_SPEED for
        backward. Commands are published at CMD_RATE_HZ throughout so the
        firmware's 200 ms command-watchdog never trips.
        """
        period = 1.0 / self.CMD_RATE_HZ
        # Acceleration ramp
        n = max(1, int(self.RAMP_SEC * self.CMD_RATE_HZ))
        for i in range(n):
            v = signed_cruise_speed * (i + 1) / n
            self._publish_vx(v)
            time.sleep(period)
        # Cruise
        n = max(1, int(self.CRUISE_SEC * self.CMD_RATE_HZ))
        for _ in range(n):
            self._publish_vx(signed_cruise_speed)
            time.sleep(period)
        # Deceleration ramp
        n = max(1, int(self.RAMP_SEC * self.CMD_RATE_HZ))
        for i in range(n):
            v = signed_cruise_speed * (n - i - 1) / n
            self._publish_vx(v)
            time.sleep(period)
        self._publish_vx(0.0)

    def _pause(self, seconds: float) -> None:
        """Publish zero-velocity for `seconds` so the firmware stays fed."""
        period = 1.0 / self.CMD_RATE_HZ
        n = max(1, int(seconds * self.CMD_RATE_HZ))
        for _ in range(n):
            self._publish_vx(0.0)
            time.sleep(period)

    # ------------------------------------------------------------------
    # Service handler
    # ------------------------------------------------------------------

    def _calibrate_cb(
        self,
        request: CalibrateImuYaw.Request,
        response: CalibrateImuYaw.Response,
    ) -> CalibrateImuYaw.Response:
        # The request's duration_sec is now ignored (the motion profile
        # dictates how long collection lasts). Kept in the .srv for
        # backwards compatibility; a future caller that wants to override
        # the total length can still drive that externally.

        # --- Preflight checks ---
        if self._is_charging:
            response.success = False
            response.message = (
                "Refusing to calibrate while charging — undock the robot first. "
                "Calibration drives the robot ~0.6 m forward then back."
            )
            return response
        if self._emergency_active:
            response.success = False
            response.message = (
                "Refusing to calibrate while an emergency is active/latched. "
                "Clear the emergency first."
            )
            return response

        with self._lock:
            self._imu_samples.clear()
            self._odom_samples.clear()
            self._collecting = True

        total_motion_sec = (
            2.0 * (self.RAMP_SEC + self.CRUISE_SEC + self.RAMP_SEC)
            + self.PAUSE_SEC
        )
        self.get_logger().info(
            f"Autonomous calibration drive starting — forward "
            f"{self.CRUISE_SPEED:.2f} m/s then back, ~{total_motion_sec:.0f}s total."
        )

        try:
            # Forward leg: ramp up, cruise, ramp down to 0.
            self._drive_profile(+self.CRUISE_SPEED)
            # Pause between directions.
            self._pause(self.PAUSE_SEC)
            # Backward leg: same profile, negative speed.
            self._drive_profile(-self.CRUISE_SPEED)
            # Let the filter / sensors settle a moment before closing.
            self._pause(self.SETTLE_SEC)
        except Exception as exc:
            # Stop the robot no matter what, then surface the failure.
            for _ in range(5):
                self._publish_vx(0.0)
                time.sleep(0.05)
            with self._lock:
                self._collecting = False
            response.success = False
            response.message = f"Drive profile errored: {exc}"
            return response
        finally:
            # Belt-and-braces stop: one last zero command after the profile.
            self._publish_vx(0.0)

        with self._lock:
            self._collecting = False
            imu_samples = list(self._imu_samples)
            odom_samples = list(self._odom_samples)

        self.get_logger().info(
            f"Drive complete — {len(imu_samples)} IMU / "
            f"{len(odom_samples)} odom samples collected."
        )

        result = self._compute_imu_yaw(imu_samples, odom_samples)
        response.success = result["success"]
        response.message = result["message"]
        response.imu_yaw_rad = result["imu_yaw_rad"]
        response.imu_yaw_deg = result["imu_yaw_deg"]
        response.samples_used = int(result["samples_used"])
        response.std_dev_deg = result["std_dev_deg"]

        if response.success:
            self.get_logger().info(
                f"imu_yaw = {response.imu_yaw_rad:+.4f} rad "
                f"({response.imu_yaw_deg:+.2f}°) from {response.samples_used} "
                f"samples, stddev {response.std_dev_deg:.2f}°"
            )
        else:
            self.get_logger().warn(f"Calibration failed: {response.message}")

        return response

    # ------------------------------------------------------------------
    # Numerical core — pure function over collected samples
    # ------------------------------------------------------------------

    def _compute_imu_yaw(
        self,
        imu_samples: list[tuple[float, float, float]],
        odom_samples: list[tuple[float, float, float]],
    ) -> dict:
        empty = {
            "success": False,
            "message": "",
            "imu_yaw_rad": 0.0,
            "imu_yaw_deg": 0.0,
            "samples_used": 0,
            "std_dev_deg": 0.0,
        }

        if len(odom_samples) < 3:
            empty["message"] = (
                f"Not enough wheel_odom samples ({len(odom_samples)}). "
                "Is /wheel_odom being published?"
            )
            return empty
        if len(imu_samples) < self.MIN_SAMPLES:
            empty["message"] = (
                f"Not enough /imu/data samples ({len(imu_samples)}). "
                "Is the IMU running?"
            )
            return empty

        odom = np.asarray(odom_samples, dtype=np.float64)
        imu = np.asarray(imu_samples, dtype=np.float64)

        odom_t = odom[:, 0]
        odom_vx = odom[:, 1]
        odom_wz = odom[:, 2]

        order_o = np.argsort(odom_t)
        odom_t = odom_t[order_o]
        odom_vx = odom_vx[order_o]
        odom_wz = odom_wz[order_o]

        order_i = np.argsort(imu[:, 0])
        imu = imu[order_i]

        if len(odom_t) < 3:
            empty["message"] = "Not enough odom samples for central difference."
            return empty
        a_body = np.zeros_like(odom_vx)
        a_body[1:-1] = (odom_vx[2:] - odom_vx[:-2]) / np.maximum(
            odom_t[2:] - odom_t[:-2], 1e-6
        )
        a_body[0] = a_body[1]
        a_body[-1] = a_body[-2]

        imu_t = imu[:, 0]
        idx = np.searchsorted(odom_t, imu_t)
        idx = np.clip(idx, 1, len(odom_t) - 1)
        left = idx - 1
        right = idx
        dt = np.maximum(odom_t[right] - odom_t[left], 1e-9)
        frac = (imu_t - odom_t[left]) / dt
        frac = np.clip(frac, 0.0, 1.0)
        wz_interp = odom_wz[left] + (odom_wz[right] - odom_wz[left]) * frac
        a_interp = a_body[left] + (a_body[right] - a_body[left]) * frac

        straight = np.abs(wz_interp) < self.WZ_STRAIGHT_THRESHOLD
        moving = np.abs(a_interp) > self.ACCEL_BODY_THRESHOLD
        mask = straight & moving

        valid_count = int(np.count_nonzero(mask))
        if valid_count < self.MIN_SAMPLES:
            empty["samples_used"] = valid_count
            empty["message"] = (
                f"Only {valid_count} valid samples (need ≥ {self.MIN_SAMPLES}). "
                "Is the robot stuck, colliding, or on uneven ground?"
            )
            return empty

        ax_valid = imu[mask, 1]
        ay_valid = imu[mask, 2]
        a_sign = np.sign(a_interp[mask])

        # Rotate by sign(a_body) so decel samples align with accel samples
        # along a single body +X direction before circular averaging.
        ax_s = ax_valid * a_sign
        ay_s = ay_valid * a_sign

        per_sample = np.arctan2(-ay_s, ax_s)
        mean_cos = float(np.mean(np.cos(per_sample)))
        mean_sin = float(np.mean(np.sin(per_sample)))
        imu_yaw_rad = math.atan2(mean_sin, mean_cos)

        # Fisher's circular std.
        r_bar = math.hypot(mean_cos, mean_sin)
        r_bar = min(max(r_bar, 1e-9), 1.0)
        std_rad = math.sqrt(-2.0 * math.log(r_bar))

        return {
            "success": True,
            "message": f"Calibrated from {valid_count} valid samples.",
            "imu_yaw_rad": imu_yaw_rad,
            "imu_yaw_deg": math.degrees(imu_yaw_rad),
            "samples_used": valid_count,
            "std_dev_deg": math.degrees(std_rad),
        }


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuYawCalibrator()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
