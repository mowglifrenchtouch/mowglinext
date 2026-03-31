# Mowgli Docker — v3 (ROS2 Jazzy)

v3 is a complete rewrite of the container deployment.  The ROS1 stack
(roscore, rosserial, openmower) has been replaced by a single
`mowgli_ros2` container running ROS2 Jazzy.

## What changed from v2

| v2 (ROS1 Noetic) | v3 (ROS2 Jazzy) |
|------------------|-----------------|
| roscore | removed — DDS has no master |
| rosserial | removed — hardware bridge is inside mowgli_ros2 |
| openmower | removed — replaced by mowgli_ros2 (Nav2 + BT + coverage) |
| foxglove-bridge (separate image) | built into mowgli_ros2, launched optionally |
| rosbridge (separate container) | built into mowgli_ros2, enabled via launch arg |

## Services

| Service | Description | Port(s) |
|---------|-------------|---------|
| `mowgli` | ROS2 hardware bridge, Nav2, behavior trees, SLAM, rosbridge, foxglove | 9090 (rosbridge), 8765 (foxglove) |
| `gui` | OpenMower GUI — Go backend + React frontend | host networking |
| `mosquitto` | MQTT broker for Home Assistant and telemetry | 1883, 9001 |
| `watchtower` | Automatic image update polling (gui only) | — |

## Hardware

| Device | Symlink | Description |
|--------|---------|-------------|
| Mowgli STM32 board | `/dev/mowgli` | Main controller (serial 115200) |
| u-blox F9P / RTK1010 | `/dev/gps` | GNSS receiver (serial 921600) |
| youyeetoo FHL-LD19 | `/dev/lidar` | LiDAR scanner (serial 230400) |

## Quick start

### One-line install

```bash
curl -sSL https://raw.githubusercontent.com/cedbossneo/mowgli-ros2/main/mowgli-docker/install.sh | bash
```

The install script handles everything: Docker, udev rules, configuration,
image pull, and startup.  Run it again to upgrade.

### Manual install

#### 1. Install Docker

```bash
curl https://get.docker.com | sh
sudo usermod -aG docker $USER
# Log out and back in
```

#### 2. Clone this repository

```bash
git clone https://github.com/cedbossneo/mowgli-ros2.git
cd mowgli-ros2/mowgli-docker
```

#### 3. udev rules (stable device symlinks)

Create `/etc/udev/rules.d/50-mowgli.rules`:

```
# Mowgli STM32 board
SUBSYSTEM=="tty", ATTRS{product}=="Mowgli", SYMLINK+="mowgli", MODE="0666"

# GPS: simpleRTK2B (u-blox F9P)
SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", SYMLINK+="gps", MODE="0666"
# GPS: RTK1010Board (ESP USB CDC)
SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="4001", SYMLINK+="gps", MODE="0666"

# LiDAR: youyeetoo FHL-LD19 (CP2102 USB-to-UART)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="lidar", MODE="0666"
```

Reload:

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Verify:

```bash
ls -l /dev/mowgli /dev/gps /dev/lidar
```

> **Note:** If your LD19 uses a CH340 adapter instead of CP2102, replace
> the LiDAR rule with:
> `SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="lidar", MODE="0666"`
>
> If you have multiple CP2102 devices, disambiguate with:
> `udevadm info -a /dev/ttyUSBx | grep product`

#### 4. Configure

```bash
cp .env.example .env
# Edit .env — set MOWER_IP for ser2net mode
# Edit config/mowgli/mowgli_robot.yaml — set GPS datum, NTRIP, dock pose
# Edit config/om/mower_config.sh — set GPS datum, NTRIP for the GUI
```

#### 5. Launch

```bash
docker compose up -d
```

## Configuration

### `.env` — image tags and network

| Variable | Default | Description |
|----------|---------|-------------|
| `ROS_DOMAIN_ID` | `0` | DDS domain — must match across all machines |
| `MOWER_IP` | `10.0.0.161` | Mower Pi IP (ser2net mode only) |
| `MOWGLI_ROS2_IMAGE` | `ghcr.io/cedbossneo/mowgli-ros2:v3` | ROS2 stack image |
| `GUI_IMAGE` | `ghcr.io/cedbossneo/openmower-gui:v3` | GUI image |

### `config/mowgli/` — ROS2 parameters

YAML parameter override files, bind-mounted at `/ros2_ws/config/` inside the
container.  The main file to edit is `mowgli_robot.yaml`.

See [`config/mowgli/README.md`](config/mowgli/README.md) for the full list of
overrideable files.

### `config/om/` — GUI settings

`mower_config.sh` and GUI-specific config files, bind-mounted at `/config`
inside the `gui` container.

### `config/mqtt/` — MQTT broker

Standard `mosquitto.conf`.

## Foxglove Studio

Foxglove Bridge runs inside the main `mowgli` container on port **8765**.
Connect Foxglove Studio to `ws://<pi-ip>:8765`.

To run Foxglove Bridge as a separate container:

```bash
docker compose -f docker-compose.yaml -f docker-compose.foxglove.yaml up -d
```

When using this override, set `enable_foxglove:=false` in a
`docker-compose.override.yaml` to avoid the port conflict.

## Deployment modes

### Ser2net — remote brain, serial over TCP

Use when the compute board (Pi running Docker) is separate from the mower
board, connected via Ethernet.

On the mower Pi, install `ser2net` to expose:
- `/dev/mowgli` on TCP port 4001
- `/dev/gps` on TCP port 4002
- `/dev/lidar` on TCP port 4003

Then on the brain machine:

```bash
# Edit .env: set MOWER_IP to the mower Pi's IP
docker compose -f docker-compose.ser2net.yaml up -d
```

### Remote split — nav on a powerful host, hardware on the Pi

Run Nav2, behavior trees, the GUI, and the map server on a desktop/server.
Run only the hardware bridge on the mower Pi.

On the mower Pi (serial access required):

```bash
docker compose -f docker-compose.remote.pi.yaml up -d
```

On the remote host:

```bash
docker compose -f docker-compose.remote.host.yaml up -d
```

Both machines must share the same `ROS_DOMAIN_ID` in `.env`.  DDS multicast
must be routable between them (same L2 segment), or configure FastDDS unicast
peer discovery.

## Logs

```bash
docker compose logs -f mowgli
docker compose logs -f gui
```

## Updating

Run the install script again:

```bash
./install.sh
```

Or manually:

```bash
docker compose pull
docker compose up -d
```

Watchtower auto-updates the `gui` image every 4 hours.

## Shutdown

```bash
docker compose down
```

SLAM maps are preserved in the `mowgli_maps` Docker volume across restarts.
