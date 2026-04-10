from launch import LaunchDescription
from launch_ros.actions import Node
import os


def env_bool(name: str, default: str = "false") -> bool:
    return os.getenv(name, default).strip().lower() in ("1", "true", "yes", "on")


def generate_launch_description():
    actions = []

    front_enabled = env_bool("TFLUNA_FRONT_ENABLED", "false")
    edge_enabled = env_bool("TFLUNA_EDGE_ENABLED", "false")

    if front_enabled:
        front_port = os.getenv("TFLUNA_FRONT_PORT", "/dev/tfluna_front")
        front_baud = int(os.getenv("TFLUNA_FRONT_BAUD", "115200"))

        actions.append(
            Node(
                package="tfluna",
                executable="tfluna_node",
                name="tfluna_front",
                output="screen",
                parameters=[{
                    "port": front_port,
                    "baudrate": front_baud,
                    "frame_id": "tfluna_front_link",
                    "topic_name": "/sensors/tfluna/front/range",
                    "field_of_view": 0.035,
                    "min_range": 0.2,
                    "max_range": 8.0,
                    "clamp_to_limits": False,
                    "debug_log": False,
                    "reconnect_period_ms": 2000,
                }],
            )
        )

    if edge_enabled:
        edge_port = os.getenv("TFLUNA_EDGE_PORT", "/dev/tfluna_edge")
        edge_baud = int(os.getenv("TFLUNA_EDGE_BAUD", "115200"))

        actions.append(
            Node(
                package="tfluna",
                executable="tfluna_node",
                name="tfluna_edge",
                output="screen",
                parameters=[{
                    "port": edge_port,
                    "baudrate": edge_baud,
                    "frame_id": "tfluna_edge_link",
                    "topic_name": "/sensors/tfluna/edge/range",
                    "field_of_view": 0.035,
                    "min_range": 0.2,
                    "max_range": 8.0,
                    "clamp_to_limits": False,
                    "debug_log": False,
                    "reconnect_period_ms": 2000,
                }],
            )
        )

    return LaunchDescription(actions)