"""SDK-owned, strictly read-only Wuji Hand + paired tactile visualization.

The public C++ ROS driver and Python SDK both claim the physical USB hand
exclusively.  This tactile launch therefore leaves the original driver
unmodified and uses exactly one official Python SDK Hand handle to publish both
read-only joint states and paired-glove pressure.  No actuator interface is
created by this launch.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, logging
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from wuji_tactile_bridge.sdk_loader import DEFAULT_SDK_SITE_PACKAGES


_logger = logging.get_logger(__name__)


def _as_bool(value: str) -> bool:
    return value.strip().lower() in ("1", "true", "yes", "on")


def _spawn_model_and_rviz(context):
    """Start the official description after the SDK bridge has connected."""
    hand_name = LaunchConfiguration("hand_name").perform(context).strip("/")
    hand_side = LaunchConfiguration("hand_side").perform(context).strip().lower()
    if hand_side not in ("left", "right"):
        _logger.error("hand_side must be 'left' or 'right'; no URDF/RViz will be started")
        return []

    try:
        publish_frequency = float(
            LaunchConfiguration("robot_state_publish_frequency").perform(context)
        )
    except ValueError:
        publish_frequency = 100.0
    if publish_frequency <= 0.0:
        publish_frequency = 100.0

    description_dir = get_package_share_directory("wuji_description")
    urdf_path = os.path.join(description_dir, "urdf", f"{hand_side}-ros.urdf")
    rviz_path = os.path.join(description_dir, "rviz", f"{hand_side}.rviz")
    try:
        with open(urdf_path, "r", encoding="utf-8") as urdf_file:
            robot_description = urdf_file.read()
    except OSError as exc:
        _logger.error(f"Failed to read official Wuji URDF {urdf_path}: {exc}")
        return []

    actions = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            namespace=hand_name,
            parameters=[
                {
                    "robot_description": robot_description,
                    "publish_frequency": publish_frequency,
                }
            ],
            remappings=[("joint_states", "joint_states")],
            output="screen",
        )
    ]
    if _as_bool(LaunchConfiguration("rviz").perform(context)):
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                namespace=hand_name,
                arguments=["-d", rviz_path],
                output="screen",
            )
        )
    return actions


def generate_launch_description() -> LaunchDescription:
    hand_name = LaunchConfiguration("hand_name")
    hand_serial = LaunchConfiguration("hand_serial_number")
    tactile_serial = LaunchConfiguration("tactile_serial_number")
    hand_side = LaunchConfiguration("hand_side")
    sdk_site_packages = LaunchConfiguration("sdk_site_packages")

    sdk_read_only_bridge = Node(
        package="wuji_tactile_bridge",
        executable="wuji_tactile_bridge",
        name="tactile_bridge",
        output="screen",
        parameters=[
            {
                "hand_name": hand_name,
                "hand_serial_number": hand_serial,
                "tactile_serial_number": tactile_serial,
                "sdk_site_packages": sdk_site_packages,
                "publish_joint_states": True,
                "expected_hand_side": hand_side,
            }
        ],
    )

    heatmap = Node(
        package="wuji_tactile_bridge",
        executable="wuji_tactile_heatmap",
        name="tactile_heatmap",
        output="screen",
        arguments=["--hand-name", hand_name],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("hand_name", default_value="hand_0"),
            DeclareLaunchArgument("hand_serial_number", default_value="336636733434"),
            DeclareLaunchArgument("tactile_serial_number", default_value="WT1JC01260720019"),
            DeclareLaunchArgument(
                "hand_side",
                default_value="left",
                description="Expected physical side and matching official URDF; the SDK bridge validates it.",
            ),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "robot_state_publish_frequency",
                default_value="100.0",
                description="TF publish frequency for robot_state_publisher only.",
            ),
            DeclareLaunchArgument(
                "sdk_site_packages",
                default_value=DEFAULT_SDK_SITE_PACKAGES,
                description="Site-packages containing the verified official wuji-sdk",
            ),
            sdk_read_only_bridge,
            TimerAction(period=2.0, actions=[OpaqueFunction(function=_spawn_model_and_rviz)]),
            TimerAction(period=2.5, actions=[heatmap]),
        ]
    )
