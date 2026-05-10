# Copyright 2026 Mowgli Project
#
# Licensed under the GNU GPL v3.

"""
Kinematic drivetrain plugin for the Webots Mowgli sim.

Phase 2.2 of the Webots migration (Gazebo→Webots port).

PROBLEM
-------
ODE wheel-floor friction does not faithfully reproduce a 0.10 m/s
diff-drive command on this PROTO geometry. After exhaustive tuning of
mass, damping, friction, COM, and motor torque (see the file header in
``protos/MowgliMower.proto`` for the full list of failed knobs), the
chassis still pitches ~13° forward at static equilibrium and only ~10 %
of the commanded velocity reaches the body (~90 % wheel-floor slip).

SOLUTION
--------
We do not care about realistic wheel slip in this simulator — we care
about Nav2 + BT + FTC controller logic working end-to-end so we can
iterate on coverage strips, obstacle deviation, and dock approach.

This plugin replaces the wheel-friction-coupled body motion with a
**fully kinematic** one. Each Webots timestep, the plugin:

  1. Reads the latest ``/cmd_vel`` (TwistStamped) from twist_mux (or a
     manual ``ros2 topic pub`` in Phase 1 testing).
  2. Integrates the diff-drive kinematics forward by the timestep:
        new_yaw = yaw + ω·Δt
        new_x   = x + vx·cos(yaw + 0.5·ω·Δt)·Δt
        new_y   = y + vx·sin(yaw + 0.5·ω·Δt)·Δt
  3. Teleports the Robot to ``(new_x, new_y, current_z)`` via the
     ``translation`` field and to ``(0,0,1, new_yaw)`` via the
     ``rotation`` field. Webots honours these field writes
     instantaneously, so the body's pose is exactly what we compute.
  4. The diff_drive_controller (a ros2_control plugin instantiated by
     webots_ros2_control alongside this plugin) continues to drive the
     wheel motors with velocity setpoints derived from the same
     ``/cmd_vel``. This is what keeps the wheel ``PositionSensor``
     outputs realistic and therefore /joint_states → /wheel_odom_raw →
     ekf_odom_node → odom→base_footprint TF all advancing — which Nav2
     pose-tracking behaviors (BackUp, DriveOnHeading, Spin) read for
     their success/timeout checks. We deliberately do NOT touch the
     wheel motors from this plugin, see the long comment in init() and
     step() for why.

EVOLUTION
---------
Two earlier revisions failed in instructive ways:

  * Revision 1 used ``Supervisor.setVelocity()`` instead of field-set.
    It improved cmd_vel honesty from ~10 % → ~70 % (Phase 1 test:
    ``ds=0.36 m`` over 5 s vs 0.50 m expected) but did NOT fix the
    chassis pitch (still 13°), because ``setVelocity`` only sets the
    velocity *vector* — gravity and wheel-friction normal forces still
    act on the body during ODE integration. The velocity also gets
    degraded between ticks by the next physics step.

  * Revision 2 used field-set teleport + per-step ``resetPhysics()``.
    cmd_vel honesty hit 95 % and pitch dropped to 0°, but
    ``resetPhysics()`` recursively zeroes the children's ODE state,
    including the wheel HingeJoint angular velocities. This silently
    overwrote the diff_drive_controller's motor setpoints back to
    zero every tick — wheels never spun, /joint_states reported 0,
    /wheel_odom_raw integrated to 0, and Nav2's BackUp/DriveOnHeading
    timed out thinking the robot had not moved.

The current revision (3) drops per-step ``resetPhysics()``. The
init-time call clears any settling impulses from the world load; from
then on the field-set teleport is sufficient. ODE may briefly try to
infer "infinite velocity" between teleports but the wheel constraints
+ chassis static friction dampen those impulses to a level that does
not visibly affect the body pose, while the wheels are now free to
spin under the diff_drive_controller's commands.

TRADEOFFS
---------
+ cmd_vel response is exact (acceptance tests #1–#3 pass at ≥95 %).
+ Chassis pitch stays effectively at 0° throughout (typically <0.1°).
+ Nav2's costmap, FTC controller, BT undock/coverage logic all see a
  robot that goes where they tell it (Phase 2: BackUp 1 m completes,
  GetNextUnmowedArea + FollowStrip dispatch reached).
+ /wheel_odom_raw is honest, so ekf_odom_node + the rest of the
  localisation chain receive realistic dead-reckoning.
- The simulator no longer reproduces wheel-floor slip events.
  Acceptable: the production wheel-slip detector (sim_wheel_slip.py)
  injects synthetic slip into the wheel-odom stream before EKF fusion,
  so slip behaviour is testable through that channel.
- Collisions with static obstacles are detected by Webots' collision
  detection on the chassis bounding box, but because we teleport every
  step the body will pass straight through. This is OK in the current
  sim — Nav2's collision_monitor + costmap are responsible for
  stopping before collisions, so collision *response* itself is not
  under test. (Future: gate the teleport on contact-points read.)

PLUGIN PROTOCOL
---------------
The class implements the contract documented in
``webots_ros2_driver``'s ``PluginInterface``:
  * ``init(webots_node, properties)`` — called once when the driver
    boots. ``webots_node.robot`` is the Webots ``Robot``/``Supervisor``
    instance.
  * ``step()`` — called on every Webots timestep (via the driver's main
    loop, after ``robot.step()``). Use this to spin rclpy + apply the
    per-step kinematic update. The driver itself calls ``robot.step()``
    so we must NOT call it here.
"""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import TwistStamped


class KinematicDrive:
    """Drives the Mowgli body kinematically from /cmd_vel."""

    # ── Robot geometry (must match config_webots/ros2_control.yaml + PROTO) ──
    # WHEEL_SEPARATION / WHEEL_RADIUS are kept as constants for
    # documentation only — they are NOT used to drive the wheel motors
    # any more (see the long comment in init() about why we leave the
    # motors to the diff_drive_controller). They appear in the boot log
    # so the operator can sanity-check the geometry against the YAML.
    WHEEL_SEPARATION = 0.325  # m, track between left and right wheels
    WHEEL_RADIUS = 0.093      # m

    # Topic the rest of the stack uses for the post-mux velocity command.
    DEFAULT_CMD_VEL_TOPIC = "/cmd_vel"

    # If we receive no fresh cmd_vel within this **wall-clock** window,
    # command zero velocity. Wall-clock (not sim-time) is critical
    # because in ``mode:=fast`` Webots advances sim time at e.g. 50–100x
    # realtime, but most cmd_vel publishers (Nav2 controller_server,
    # behavior_server, collision_monitor, twist_mux) cycle at a
    # *wall-rate* (typically 10–20 Hz wall). A sim-time timeout would
    # therefore trip every cycle, causing the robot to repeatedly stop
    # and stutter through coverage paths. 0.5 s wall is generous enough
    # for any sane publisher cadence and tight enough that a true
    # publisher hang is caught quickly.
    CMD_VEL_TIMEOUT_S = 0.5

    # ──────────────────────────────────────────────────────────────────────
    # Plugin lifecycle
    # ──────────────────────────────────────────────────────────────────────
    def init(self, webots_node, properties):
        self.__supervisor = webots_node.robot
        self.__timestep_ms = int(self.__supervisor.getBasicTimeStep())
        self.__dt_s = self.__timestep_ms / 1000.0

        # Resolve the Robot node so we can manipulate its pose. getSelf()
        # works because the Robot is supervisor=TRUE in the PROTO/world.
        self.__robot_node = self.__supervisor.getSelf()
        if self.__robot_node is None or self.__robot_node._ref is None:
            raise RuntimeError(
                "KinematicDrive: Supervisor.getSelf() returned a NULL node. "
                "The Robot must be supervisor=TRUE in both the PROTO and the "
                "world instantiation."
            )

        # Pose fields: we set translation + rotation directly each step.
        self.__translation_field = self.__robot_node.getField("translation")
        self.__rotation_field = self.__robot_node.getField("rotation")
        if self.__translation_field is None or self.__rotation_field is None:
            raise RuntimeError(
                "KinematicDrive: failed to resolve translation/rotation "
                "fields on the Robot node."
            )

        # Seed the integrated pose from whatever the world placed the
        # Robot at — so a non-zero initial pose in the .wbt is honoured.
        init_xyz = self.__translation_field.getSFVec3f()
        init_axis_angle = self.__rotation_field.getSFRotation()
        # Webots SFRotation is (axis_x, axis_y, axis_z, angle). For a
        # flat-ground robot we expect axis ≈ (0, 0, ±1) and treat the
        # angle as yaw. If the axis is flipped we negate the angle so
        # downstream math stays consistent.
        ax_z = init_axis_angle[2]
        sign = 1.0 if ax_z >= 0 else -1.0
        self.__x = init_xyz[0]
        self.__y = init_xyz[1]
        self.__z = init_xyz[2]
        self.__yaw = sign * init_axis_angle[3]

        # We intentionally do NOT touch the wheel motors here. The
        # diff_drive_controller (instantiated by ros2_control via
        # webots_ros2_control) initialises the motors into velocity-
        # control mode (position=+inf) and writes setVelocity setpoints
        # every controller cycle. Earlier revisions of this plugin
        # called motor.setPosition(inf) + motor.setVelocity(0) here as
        # an "idempotent re-init", but in practice that race-condition'd
        # with the controller's own init and left the motors stuck at
        # zero velocity — wheels never spun, /joint_states reported 0,
        # /wheel_odom_raw integrated to 0, ekf_odom_node could not
        # advance odom→base_footprint, and Nav2 BackUp/DriveOnHeading
        # timed out thinking the robot had not moved.

        # ROS interface. webots_ros2_driver loads each plugin in the
        # SAME process as the driver, so rclpy may already be initialised
        # by the driver itself — guard with rclpy.ok().
        if not rclpy.ok():
            rclpy.init(args=None)

        # Use a unique node name per Robot — supports multi-robot worlds.
        node_name = f"{self.__supervisor.getName()}_kinematic_drive"
        node_name = node_name.replace(" ", "_").replace("-", "_")
        self.__node = rclpy.create_node(node_name)

        topic = properties.get("cmdVelTopic", self.DEFAULT_CMD_VEL_TOPIC)
        # TwistStamped to match twist_mux output and the diff_drive
        # controller's use_stamped_vel: true.
        self.__node.create_subscription(
            TwistStamped, topic, self.__cmd_vel_callback, 1
        )

        # Latest commanded velocity. Default to zero so the robot is at
        # rest until the first cmd_vel arrives.
        self.__target_vx = 0.0
        self.__target_wz = 0.0
        self.__last_cmd_time_s = -1.0

        # Reset physics ONCE at boot so any settling-impulse buildup
        # from the world load is cleared before we start teleporting.
        self.__robot_node.resetPhysics()

        self.__node.get_logger().info(
            f"KinematicDrive initialised. Robot='{self.__supervisor.getName()}', "
            f"cmd_vel topic='{topic}', timestep={self.__timestep_ms} ms, "
            f"wheel_separation={self.WHEEL_SEPARATION} m, "
            f"wheel_radius={self.WHEEL_RADIUS} m, "
            f"initial_pose=(x={self.__x:.3f}, y={self.__y:.3f}, "
            f"z={self.__z:.3f}, yaw={self.__yaw:.3f})."
        )

    def __cmd_vel_callback(self, msg: TwistStamped) -> None:
        self.__target_vx = float(msg.twist.linear.x)
        self.__target_wz = float(msg.twist.angular.z)
        # Wall-clock timestamp — see CMD_VEL_TIMEOUT_S docstring for why.
        self.__last_cmd_time_s = time.monotonic()

    def step(self):
        # The driver also calls robot.step() — we must NOT call it here
        # or the simulator will double-step.
        rclpy.spin_once(self.__node, timeout_sec=0)

        # Apply cmd_vel timeout — wall-clock so fast-mode does not
        # spuriously stop us between widely-spaced cmd_vel cycles. See
        # CMD_VEL_TIMEOUT_S docstring.
        wall_now = time.monotonic()
        if (
            self.__last_cmd_time_s < 0
            or (wall_now - self.__last_cmd_time_s) > self.CMD_VEL_TIMEOUT_S
        ):
            vx = 0.0
            wz = 0.0
        else:
            vx = self.__target_vx
            wz = self.__target_wz

        # ── Diff-drive integration (body frame → world frame) ─────────
        # Use mid-point yaw for slightly better arc accuracy at high ω.
        dt = self.__dt_s
        new_yaw = self.__yaw + wz * dt
        mid_yaw = self.__yaw + 0.5 * wz * dt
        self.__x += vx * math.cos(mid_yaw) * dt
        self.__y += vx * math.sin(mid_yaw) * dt
        self.__yaw = new_yaw
        # Wrap yaw to (-π, π] so the SFRotation angle stays bounded.
        if self.__yaw > math.pi or self.__yaw < -math.pi:
            self.__yaw = math.atan2(math.sin(self.__yaw), math.cos(self.__yaw))

        # ── Teleport via translation + rotation fields ─────────────────
        # Keep z fixed at whatever the world placed us at — flat-ground
        # motion. (Future: ride a heightmap by reading z from a lookup.)
        self.__translation_field.setSFVec3f([self.__x, self.__y, self.__z])
        self.__rotation_field.setSFRotation([0.0, 0.0, 1.0, self.__yaw])
        # We deliberately do NOT call resetPhysics() here every step.
        #
        # resetPhysics() recursively zeroes the internal ODE solver
        # state for this node AND ALL ITS CHILDREN, which means the
        # wheel HingeJoint angular velocities also get zeroed every
        # tick. Combined with the field-set teleport (which forces
        # the parent's pose), the wheels physically cannot rotate:
        # the diff_drive_controller's motor.setVelocity() command is
        # silently overwritten back to zero by our resetPhysics. The
        # downstream effect is /joint_states reporting position=0
        # forever, /wheel_odom_raw integrating to zero, the EKF
        # dead-reckoning chain (odom→base_footprint via ekf_odom_node)
        # frozen, and Nav2's pose-tracking behaviors (BackUp,
        # DriveOnHeading) thinking the robot never moves.
        #
        # Without resetPhysics() the field-set teleport is still
        # honoured by Webots; ODE may briefly try to compute residual
        # impulses from the implied "infinite velocity" between
        # teleports, but those are dampened by the wheel HingeJoint
        # constraints and the chassis static friction enough that the
        # body settles to the kinematic pose we set, while the wheels
        # are free to spin under the diff_drive_controller's commands.
        # We rely on the init-time resetPhysics() to clear any startup
        # impulses; from then on the field-set is sufficient.

        # ── Wheel cosmetic rotation ────────────────────────────────────
        # We deliberately do NOT call motor.setVelocity() here — the
        # diff_drive_controller (instantiated by ros2_control via
        # webots_ros2_control) is already writing wheel velocity
        # setpoints derived from the same /cmd_vel we just consumed,
        # using the same wheel_separation / wheel_radius values from
        # config_webots/ros2_control.yaml. Letting the controller own
        # the motor commands has two important consequences:
        #
        #   1. The Webots PositionSensor on each HingeJoint accumulates
        #      a realistic wheel position, which the joint_state_-
        #      broadcaster publishes on /joint_states. RSP then animates
        #      the wheel TFs; diff_drive_controller integrates them
        #      into /wheel_odom_raw; and ekf_odom_node fuses that into
        #      odom→base_footprint. *This is what Nav2's behaviors
        #      (BackUp, DriveOnHeading, Spin) read for their pose-
        #      tracking success/timeout checks.* Skipping this would
        #      freeze the odom frame and break every Nav2 motion goal.
        #
        #   2. We avoid a writer-vs-writer race on the motor velocity
        #      setpoint. Even though both writes would compute the same
        #      omega from the same cmd_vel + same diff-drive math,
        #      having a single owner per Webots device is the cleaner
        #      contract.
