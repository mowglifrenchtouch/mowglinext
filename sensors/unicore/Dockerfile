# syntax=docker/dockerfile:1.7
# =============================================================================
# Unicore UM982 GNSS driver (mowgli_unicore_gnss) — ROS 2 C++
#
# Publishes:
#   /gps/fix            sensor_msgs/NavSatFix (NMEA GGA or Unicore PVTSLNA)
#   /gps/azimuth        compass_msgs/Azimuth (heading from HDT or HPR)
#   /gps/diagnostics    diagnostic_msgs/DiagnosticArray (status + counters)
#
# Serial device mounted at runtime: /dev/gps (configurable via um982.yaml)
# =============================================================================

# ─── Builder ────────────────────────────────────────────────────────────────
FROM ros:kilted-ros-base AS builder

ARG DEBIAN_FRONTEND=noninteractive

RUN sed -i 's|http://archive.ubuntu.com/ubuntu|http://azure.archive.ubuntu.com/ubuntu|g' /etc/apt/sources.list.d/*.sources 2>/dev/null || true

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      libcurl4-openssl-dev \
      python3-colcon-common-extensions \
      ros-kilted-rclcpp \
      ros-kilted-rclcpp-components \
      ros-kilted-diagnostic-msgs \
      ros-kilted-rosidl-default-generators \
      ros-kilted-rosidl-default-runtime \
      ros-kilted-rtcm-msgs \
      ros-kilted-sensor-msgs \
      ros-kilted-std-msgs \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /ws/src
COPY compass_msgs/package.xml compass_msgs/package.xml
COPY compass_msgs/CMakeLists.txt compass_msgs/CMakeLists.txt
COPY compass_msgs/msg/Azimuth.msg compass_msgs/msg/Azimuth.msg

COPY ntrip_client_node/package.xml ntrip_client_node/package.xml
COPY ntrip_client_node/CMakeLists.txt ntrip_client_node/CMakeLists.txt
COPY ntrip_client_node/include/ ntrip_client_node/include/
COPY ntrip_client_node/src/ ntrip_client_node/src/

COPY CMakeLists.txt mowgli_unicore_gnss/CMakeLists.txt
COPY package.xml mowgli_unicore_gnss/package.xml
COPY include/ mowgli_unicore_gnss/include/
COPY src/ mowgli_unicore_gnss/src/
COPY launch/ mowgli_unicore_gnss/launch/
COPY config/ mowgli_unicore_gnss/config/
COPY test/ mowgli_unicore_gnss/test/

WORKDIR /ws
RUN . /opt/ros/kilted/setup.sh \
 && colcon build --merge-install \
      --base-paths src/compass_msgs src/ntrip_client_node src/mowgli_unicore_gnss \
      --packages-up-to ntrip_client_node mowgli_unicore_gnss \
      --cmake-args -DCMAKE_BUILD_TYPE=Release \
                   -Wno-dev \
 && rm -rf /ws/build /ws/log /ws/src

# ─── Runtime ────────────────────────────────────────────────────────────────
FROM ros:kilted-ros-base

ARG DEBIAN_FRONTEND=noninteractive

RUN sed -i 's|http://archive.ubuntu.com/ubuntu|http://azure.archive.ubuntu.com/ubuntu|g' /etc/apt/sources.list.d/*.sources 2>/dev/null || true

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
      libcurl4 \
      ros-kilted-diagnostic-msgs \
      ros-kilted-rclcpp-components \
      ros-kilted-rmw-cyclonedds-cpp \
      ros-kilted-rosidl-default-runtime \
      ros-kilted-rtcm-msgs \
      ros-kilted-sensor-msgs \
      ros-kilted-std-msgs \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /ws

COPY --from=builder /ws/install /ws/install

COPY ros2_entrypoint.sh /ros2_entrypoint.sh
COPY start_gps.sh /start_gps.sh
RUN chmod +x /ros2_entrypoint.sh /start_gps.sh

ENTRYPOINT ["/ros2_entrypoint.sh"]
CMD ["/start_gps.sh"]
