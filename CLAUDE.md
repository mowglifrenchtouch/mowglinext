# MowgliNext

Open-source autonomous robot mower monorepo. ROS2 Kilted, Nav2, Cartographer (drift-overlay on GPS-anchored FusionCore), BehaviorTree.CPP v4, cell-based strip coverage.

**Website:** https://mowgli.garden | **Wiki:** https://github.com/cedbossneo/mowglinext/wiki

## Safety — READ FIRST

This robot has spinning blades. The STM32 firmware is the sole blade safety authority.

- NEVER bypass firmware blade safety checks from ROS2
- Blade commands from ROS2 are fire-and-forget — firmware decides whether to execute
- Emergency stop is handled by firmware, not software
- Flag ANY change that could affect physical behavior as safety-critical in PR reviews

## Monorepo Layout

| Directory | Language | Build | Description |
|-----------|----------|-------|-------------|
| `ros2/` | C++17, Python | `colcon build` | ROS2 stack: 12 packages (Nav2, SLAM, BT, coverage, hardware bridge) |
| `install/` | Shell | `./mowglinext.sh` | Interactive installer, hardware presets, modular Docker Compose configs |
| `gui/` | Go, TypeScript/React | `go build`, `yarn build` | Web interface for config, map editing, monitoring |
| `docker/` | YAML, Shell | `docker compose` | Manual deployment configs, DDS, service orchestration |
| `sensors/` | Dockerfile | `docker build` | Dockerized sensor drivers (GPS, LiDAR) |
| `firmware/` | C | `pio run` | STM32F103 firmware (motor, IMU, blade, battery) |
| `docs/` | HTML, CSS, JS | GitHub Pages | Landing page + install composer at mowgli.garden |

## Architecture Invariants (DO NOT VIOLATE)

1. **Cartographer is TF authority for map→odom**, in drift-overlay mode (`POSE_GRAPH.optimize_every_n_nodes = 0`, `use_imu_data = false`). FusionCore is the primary localizer (GPS-RTK anchor, ~5 mm σ); Cartographer's map→odom stays near identity. FusionCore owns `odom→base_footprint` (GPS+IMU+wheels fused in single UKF, 100 Hz). No feedback loop: Cartographer does not feed into FusionCore.
2. **TF chain follows REP-105** — `map→odom→base_footprint→base_link→sensors`. All Nav2 nodes, FusionCore, and SLAM use `base_footprint` as the robot frame. `base_link` is at the rear wheel axis (OpenMower convention, do not move).
3. **Cyclone DDS** — not FastRTPS (stale shm issues on ARM)
4. **Map frame = GPS frame** — X=east, Y=north, no rotation transform
5. **Costmap obstacles disabled in coverage mode** — collision_monitor handles real-time avoidance
6. **dock_pose_yaw from phone compass** — measured once at installation, not computed
7. **Cell-based multi-area strip coverage** — `map_server_node` plans strips on demand via `~/get_next_strip` service; no pre-planned full path. BT nodes `GetNextUnmowedArea` (outer loop, iterates through all mowing areas), `GetNextStrip` (inner loop, fetches strips for current area), `TransitToStrip`, `FollowStrip` execute sequentially. Progress tracked in `mow_progress` grid layer (survives restarts). Coverage status via `~/get_coverage_status` service and `/map_server_node/coverage_cells` OccupancyGrid topic.
8. **FTCController for coverage paths** — RPP for transit only, FTCController (PID on 3 axes) for coverage path following
9. **Emergency auto-reset on dock** — When emergency is active and robot is on dock (charging detected), BT auto-sends `ResetEmergency` to firmware. Firmware is sole safety authority and only clears latch if physical trigger is no longer asserted.
10. **Undock via Nav2 BackUp behavior** — BackUp (1.5m, 0.15 m/s) via `behavior_server`, not `opennav_docking` UndockRobot (isDocked() unreliable with GPS/SLAM drift). Costmaps are cleared after undock.
11. **Zero-odom only when charging AND idle** — `hardware_bridge_node` does not reset odometry during undock sequence.
12. **Battery current for dock detection** — Hardware bridge publishes `abs(charging_current)` when charging, `0.0` when not, for `SimpleChargingDock` compatibility.
13. **Docking server cmd_vel** — Remapped to `/cmd_vel_docking` through twist_mux (priority 15).
14. **Coverage grid_map convention** — `getSize()(0)` = X cells, `getSize()(1)` = Y cells. Manual OccupancyGrid conversion: `width=size(0)`, `height=size(1)`, with Y-axis flip.

## High-Level Commands and States

### HighLevelControl.srv Commands
| Value | Constant | Description |
|-------|----------|-------------|
| 1 | `COMMAND_START` | Begin autonomous mowing |
| 2 | `COMMAND_HOME` | Return to dock |
| 3 | `COMMAND_RECORD_AREA` | Start area boundary recording |
| 4 | `COMMAND_S2` | Mow next area |
| 5 | `COMMAND_RECORD_FINISH` | Finish recording, save polygon |
| 6 | `COMMAND_RECORD_CANCEL` | Cancel recording, discard trajectory |
| 7 | `COMMAND_MANUAL_MOW` | Enter manual mowing mode (teleop + blade) |
| 254 | `COMMAND_RESET_EMERGENCY` | Reset latched emergency |
| 255 | `COMMAND_DELETE_MAPS` | Delete all maps |

### HighLevelStatus.msg States
| Value | Constant | Description |
|-------|----------|-------------|
| 0 | `HIGH_LEVEL_STATE_NULL` | Emergency or transitional |
| 1 | `HIGH_LEVEL_STATE_IDLE` | Idle, docked, charging, returning home |
| 2 | `HIGH_LEVEL_STATE_AUTONOMOUS` | Autonomous mowing (undocking, transit, mowing, recovering) |
| 3 | `HIGH_LEVEL_STATE_RECORDING` | Area recording in progress |
| 4 | `HIGH_LEVEL_STATE_MANUAL_MOWING` | Manual mowing via teleop |

### Area Recording Flow
1. GUI sends `COMMAND_RECORD_AREA` (3) to start recording
2. BT enters `RecordArea` node — records position at 2 Hz, publishes live preview on `~/recording_trajectory`
3. User drives robot along boundary
4. GUI sends `COMMAND_RECORD_FINISH` (5) — trajectory is simplified (Douglas-Peucker) and saved via `/map_server_node/add_area`
5. Or GUI sends `COMMAND_RECORD_CANCEL` (6) — trajectory discarded

### Manual Mowing
- Dedicated BT state with `COMMAND_MANUAL_MOW` (7) — does not hijack recording mode
- Teleop via `/cmd_vel_teleop` (twist_mux priority)
- Blade managed by GUI (fire-and-forget to firmware)
- Collision_monitor, GPS, Cartographer all remain active

## Code Style

| Component | Style | Tool |
|-----------|-------|------|
| C++ (ros2/) | 2-space indent, `snake_case` files/params, `CamelCase` classes | `clang-format` (config in `ros2/.clang-format`) |
| Go (gui/) | Standard Go | `gofmt` |
| TypeScript (gui/web/) | Prettier + ESLint | `yarn lint` |
| Python (launch files) | PEP 8 | — |
| YAML (config) | 2-space indent, `snake_case` keys | — |

## Commit Conventions

```
<type>: <description>

Types: feat, fix, refactor, docs, test, chore, perf, ci
```

No Co-Authored-By lines. Keep messages concise and focused on "why".

## ROS2 Specifics

- **Distro:** Kilted
- **DDS:** Cyclone DDS (all containers share `docker/config/cyclonedds.xml`)
- **Topics:** Mowgli-specific topics under `/mowgli/` namespace
- **Frames:** `map` (global), `odom` (local), `base_footprint` (robot frame for Nav2/FusionCore/SLAM), `base_link` (rear axle), `lidar_link`, `imu_link`
- **TF chain:** `map→odom` (Cartographer, drift-overlay), `odom→base_footprint` (FusionCore), `base_footprint→base_link` (static), `base_link→sensors` (static — `base_link→imu_link` rotation = `imu_yaw/pitch/roll` from `mowgli_robot.yaml`, auto-calibratable via GUI button)
- **Units:** SI throughout (metres, radians, seconds)
- **Sensor fusion:** FusionCore (single UKF, 50Hz, GPS+IMU+wheels → odom→base_footprint TF + `/fusion/odom`). Source-built from `ros2/src/fusioncore/`. Lifecycle node auto-configured at launch.
- **Navigation:** RPP for transit, FTCController (Follow-the-Carrot with 3-axis PID) for coverage paths (NOT MPPI — it jumps between adjacent swaths)
- **Coverage:** Cell-based strip planner in `map_server_node`. Multi-area outer loop (`GetNextUnmowedArea`) iterates through all mowing areas. Inner strip loop fetches strips one at a time (`GetNextStrip` -> `TransitToStrip` -> `FollowStrip`). No full-path pre-planning. Progress persisted in `mow_progress` grid layer. All areas mowed sequentially, then robot docks.
- **Area Recording:** `RecordArea` BT node records trajectory at 2 Hz, Douglas-Peucker simplification, saves polygon via `/map_server_node/add_area`. Live preview on `~/recording_trajectory`.
- **Manual Mowing:** Dedicated BT state (COMMAND_MANUAL_MOW=7). Teleop via `/cmd_vel_teleop`, blade managed by GUI. Collision_monitor/GPS/Cartographer remain active.
- **Emergency Auto-Reset:** BT auto-resets emergency when robot placed on dock (charging detected). Firmware is safety authority.
- **GPS fusion:** FusionCore takes `/gps/fix` (NavSatFix) directly — no intermediate converter. `navsat_to_absolute_pose_node` still provides `/gps/absolute_pose` for GUI and BT.
- **SLAM:** Cartographer publishes `map→odom` TF (20 Hz) and occupancy grid. Drift-overlay stance: `optimize_every_n_nodes=0` (global pose-graph optimization off), `use_imu_data=false` (imu_link not colocated with tracking_frame), trusts FusionCore's odom. No feedback into FusionCore. State saved via `/cartographer_node/write_state` → `.pbstream` (GUI button, no autosave).
- **IMU mounting calibration:** `base_link→imu_link` rotation (imu_roll, imu_pitch, imu_yaw in mowgli_robot.yaml) is critical — if wrong, FusionCore's gravity-removal leaks into pitch, pitch drifts past ±π/2 during rotation, and yaw integration flips sign (process model at `ros2/src/fusioncore/fusioncore_core/src/ukf.cpp:101`). Use the GUI's "Auto-calibrate" button next to IMU Yaw — the robot drives itself ~0.6 m forward then back and solves `imu_yaw = atan2(-ay_chip, ax_chip)` from accel direction vs wheel-derived `a_body`.
- **Nav2 tuning:** Global costmap 30m x 30m rolling window; keepout_filter disabled in global costmap (blocks transit/docking); collision_monitor PolygonStop min_points=8, PolygonSlow min_points=6; source_timeout 5.0s (ARM TF jitter); progress checker 0.15m required movement, 30s timeout; failure_tolerance 1.0; speeds: mowing 0.3/0.15 m/s, transit 0.2 m/s, max 0.3 m/s.
- **Joystick:** Foxglove client passes `schemaName` in `clientAdvertise` for JSON-to-CDR conversion. GUI shows joystick during "RECORDING" state (not just "AREA_RECORDING").

See sections below for detailed package descriptions, topics, and architecture.

## Git Workflow

**NEVER commit directly to main.** Always use feature branches and PRs:
```bash
git checkout main && git pull
git checkout -b feat/my-feature    # or fix/, refactor/, test/, chore/, docs/
# ... make changes ...
git add <files> && git commit -m "feat: description"
gh pr create --title "feat: my feature" --body "..."
```

### Dev Branch Workflow

Docker builds trigger on both `main` and `dev` branches. Images are tagged `:main` and `:dev` respectively. Use `mowgli-dev` / `mowgli-main` commands to switch between environments. Iterate on `dev`, merge to `main` when stable.

## Quick Commands

All ROS2 commands assume you are inside the devcontainer.

```bash
# Build ROS2 workspace
cd ros2 && make build

# Build a single package
cd ros2 && make build-pkg PKG=mowgli_behavior

# Run headless simulation
cd ros2 && make sim

# Run E2E tests (simulation must be running in another terminal)
cd ros2 && make e2e-test

# Format C++ code
cd ros2 && make format

# Run unit tests
cd ros2 && make test

# Build firmware
cd firmware/stm32/ros_usbnode && pio run

# GUI development
cd gui && go build -o openmower-gui && cd web && yarn dev

# --- Code generation (run after changing .msg/.srv files) ---

# Regenerate firmware rosserial C++ headers from ROS2 .msg files
python3 firmware/scripts/sync_ros_lib.py          # writes to firmware/stm32/ros_usbnode/src/ros/ros_lib/mower_msgs/
python3 firmware/scripts/sync_ros_lib.py --check   # diff-only, no writes (CI)

# Regenerate Go message/service structs from ROS2 .msg/.srv files
cd gui && ./generate_go_msgs.sh                    # writes to gui/pkg/msgs/*/types_generated.go

# Regenerate TypeScript ROS types (snake_case fields matching rosbridge JSON)
cd gui && ./generate_ts_types.sh                   # writes to gui/web/src/types/ros.ts
```

### Code Generation Workflow

When you modify `ros2/src/mowgli_interfaces/msg/*.msg` or `srv/*.srv`:
1. **Firmware headers:** `python3 firmware/scripts/sync_ros_lib.py` — regenerates rosserial C++ headers
2. **Go types:** `cd gui && ./generate_go_msgs.sh` — regenerates Go structs with JSON tags for rosbridge
3. **TypeScript types:** `cd gui && ./generate_ts_types.sh` — regenerates `gui/web/src/types/ros.ts` with snake_case fields matching rosbridge JSON
4. **Protocol constants:** Update `HL_MODE_*` defines in `firmware/stm32/ros_usbnode/include/mowgli_protocol.h` AND `ros2/src/mowgli_hardware/firmware/mowgli_protocol.h` (these are manually maintained — keep in sync with `HighLevelStatus.msg`)

Do NOT hand-edit `*_generated.go`, `ros_lib/mower_msgs/*.h`, or `gui/web/src/types/ros.ts` — re-run the scripts instead.

## Mowing Session Monitoring

**Whenever a mowing test is run (COMMAND_START, undock, autonomous motion, or any tuning session that involves the robot moving), also run the session monitor in parallel.** Output is a JSONL timeline that can be diffed/plotted across sessions to see how tuning changes affect behavior.

```bash
# Detached background from the host (writes to /ros2_ws/logs/mow_sessions/
# inside the container, which is not mounted — better to bind-mount docker/logs/
# or redirect via --output-dir):
docker exec -d mowgli-ros2 bash -c '
  source /opt/ros/kilted/setup.bash && source /ros2_ws/install/setup.bash && \
  python3 /ros2_ws/scripts/mow_session_monitor.py \
    --session 2026-04-19-cartographer-tuning-v3 \
    --output-dir /ros2_ws/maps'

# Interactively from inside the container (Ctrl-C to stop + write summary):
docker exec -it mowgli-ros2 bash -c '
  source /opt/ros/kilted/setup.bash && source /ros2_ws/install/setup.bash && \
  python3 /ros2_ws/scripts/mow_session_monitor.py --session <name> \
    --output-dir /ros2_ws/maps'
```

The `--output-dir /ros2_ws/maps` redirects to the bind-mounted `install_mowgli_maps` Docker volume so logs persist outside the container. Or bind-mount `docker/logs/mow_sessions/` explicitly in compose for a host-visible path.

**What it records** (per-sample, 10 Hz default):
- FusionCore pose + twist (x/y/z, yaw, vx/vy/wz)
- TF snapshots: `map→base_footprint` (Cartographer composed), `map→odom` (Cartographer correction alone), `odom→base_footprint` (FusionCore alone)
- Wheel twist + covariance + integrated distance and yaw
- IMU gyro + accel + integrated gyro yaw
- GPS NavSatFix (lat/lon/alt/status/covariance) + `/gps/absolute_pose` ENU
- Dock heading (`/gnss/heading` while charging)
- BT state (state_name, current_area, current_strip), hardware mode, emergency flags, battery
- `cmd_vel_nav` (Nav2 output) + `cmd_vel` (post-safety, what reaches motors)
- Nav2 `/plan` length, next pose, goal pose, distance-to-goal
- LiDAR scan health (valid point count, min range)
- **Cross-source consistency**: `fusion ↔ gps` distance, `fusion ↔ cartographer` distance + yaw diff, `wheel ↔ gyro` yaw drift

**Metadata header** (first line of the JSONL): session name, UTC timestamp, git branch + commit + dirty flag, docker image tags from `.env`, SHA-256 truncated hashes of `mowgli_robot.yaml`, `localization.yaml`, `nav2_params.yaml`, `cartographer.lua` — so sessions from different tunings are grouped/comparable.

**Summary record** (last line, written on Ctrl-C or clean shutdown): total duration, samples written, wheel-integrated distance, straight-line displacement, peak `fusion↔gps` error, peak `wheel↔gyro` yaw drift, final BT state.

**Log directory:** `docker/logs/mow_sessions/<session_name>.jsonl`. Commit notable sessions (golden runs, failure cases) so they survive in git history.

## Recommended Skills and Agents

When using Claude Code on this project:

### Skills to Use
- `/ros2-engineering` — ROS2 node patterns, QoS, launch files, Nav2, SLAM (use for any ros2/ work)
- `/cpp-coding-standards` — C++ Core Guidelines (use for C++ reviews)
- `/docker-patterns` — Dockerfile and compose patterns (use for docker/ and sensors/ work)
- `/tdd` — Test-driven development (use when adding new features)

### Agents to Invoke
- **code-reviewer** — after any code changes
- **cpp-reviewer** — after C++ changes in ros2/
- **security-reviewer** — before commits touching auth, configs, or firmware commands
- **build-error-resolver** — when colcon or Docker builds fail
- **tdd-guide** — when implementing new features
- **architect** — for design decisions spanning multiple packages

## What NOT to Do

- Do NOT add ROS1 patterns (rosserial, roscore, catkin) — this is ROS2 only
- Do NOT use FastRTPS — Cyclone DDS is required
- Do NOT mock the database/firmware in integration tests — use real interfaces
- Do NOT add a second TF publisher for `map→odom` — Cartographer is the sole authority (falls back to static identity when `slam=false`)
- Do NOT re-enable Cartographer global optimization (`optimize_every_n_nodes > 0`) or `use_imu_data` without explicit revisit — both introduced instability on this sparse-feature outdoor setup and were deliberately disabled
- Do NOT feed SLAM output into FusionCore — causes feedback loops (SLAM reads FusionCore's TF)
- Do NOT use robot_localization — replaced by FusionCore (single UKF)
- Do NOT send blade commands without firmware safety checks
- Do NOT hardcode GPS coordinates, dock poses, or NTRIP credentials
- Do NOT use MPPI controller for coverage paths — it jumps between swaths
- Do NOT use RPP for coverage paths — use FTCController for <10mm lateral accuracy on swaths
- Do NOT use `base_link` as robot_base_frame in Nav2/FusionCore — use `base_footprint` (REP-105)
- Do NOT use opennav_docking UndockRobot — use Nav2 BackUp behavior (isDocked() unreliable with GPS/SLAM drift)
