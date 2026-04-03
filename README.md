# MowgliNext

Autonomous robot mower built on ROS2 Jazzy — a complete rewrite of OpenMower for YardForce Classic 500 hardware with LiDAR, behavior trees, and modern navigation.

## Monorepo Structure

| Directory | Description |
|-----------|-------------|
| [`ros2/`](ros2/) | ROS2 stack: Nav2, SLAM Toolbox, behavior trees, coverage planner, hardware bridge |
| [`docker/`](docker/) | Docker Compose deployment with Cyclone DDS, GPS/LiDAR containers |
| [`gui/`](gui/) | React + Go web interface for configuration, map editing, and monitoring |
| [`firmware/`](firmware/) | STM32 firmware for motor control, IMU, blade safety |

## Architecture

```
┌─────────────────────────────────────────────────┐
│  GUI (React + Go)          :4006                │
├─────────────────────────────────────────────────┤
│  ROS2 Stack (Jazzy)                             │
│  ┌──────────┐ ┌──────────┐ ┌──────────────────┐│
│  │ Nav2     │ │ SLAM     │ │ Behavior Tree    ││
│  │ (navigate│ │ Toolbox  │ │ (main_tree.xml)  ││
│  │  dock    │ │          │ │                  ││
│  │  cover)  │ │          │ │                  ││
│  └──────────┘ └──────────┘ └──────────────────┘│
│  ┌──────────┐ ┌──────────┐ ┌──────────────────┐│
│  │ Coverage │ │ Localiz. │ │ Hardware Bridge  ││
│  │ Planner  │ │ (GPS+EKF)│ │ (serial ↔ ROS2) ││
│  │ (F2C v2) │ │          │ │                  ││
│  └──────────┘ └──────────┘ └──────────────────┘│
├─────────────────────────────────────────────────┤
│  STM32 Firmware (blade safety, motor control)   │
└─────────────────────────────────────────────────┘
```

## Key Design Decisions

- **base_link at rear wheel axis** — OpenMower convention
- **SLAM is sole TF authority** — EKF publishes odometry only, not TF
- **Cyclone DDS** — replaces FastRTPS (stale shm issues on ARM)
- **Map frame = GPS frame** — X=east, Y=north, no rotation
- **Firmware is blade safety authority** — ROS2 blade control is fire-and-forget
- **Collision monitor for real-time avoidance** — costmap obstacles disabled in coverage planner

## Quick Start

See [`docker/README.md`](docker/README.md) for deployment instructions.

## Hardware

- YardForce Classic 500 chassis
- Rockchip ARM board (Docker host)
- RPLiDAR for obstacle detection
- RTK GPS (u-blox F9P)
- Custom STM32 board for motor/blade/IMU

## License

See individual component directories for license information.
