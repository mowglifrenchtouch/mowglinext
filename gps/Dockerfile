# =============================================================================
# u-blox GPS driver + NTRIP client for ROS2 Jazzy
#
# Publishes sensor_msgs/NavSatFix on /ublox_gps_node/fix.
# Receives RTK corrections via ntrip_client and forwards to u-blox receiver.
#
# The mowgli_ros2 container's navsat_to_absolute_pose node converts NavSatFix
# into the internal AbsolutePose format for the localization pipeline.
# =============================================================================
FROM ros:jazzy-ros-base

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ublox \
    ros-jazzy-ntrip-client \
 && rm -rf /var/lib/apt/lists/*

COPY ros2_entrypoint.sh /ros2_entrypoint.sh
RUN chmod +x /ros2_entrypoint.sh

ENTRYPOINT ["/ros2_entrypoint.sh"]
CMD ["bash"]
