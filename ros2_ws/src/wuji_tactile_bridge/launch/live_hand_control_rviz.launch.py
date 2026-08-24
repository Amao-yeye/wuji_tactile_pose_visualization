"""RViz-hosted tactile visualization plus constrained hand control.

Exactly one Python SDK owner publishes joint/tactile state and accepts only
named, safety-gated actions. The RViz panel is a ROS client; it never owns the
SDK or publishes arbitrary joint targets. A panel heartbeat is required for
remote motion, and loss of that heartbeat auto-disables an enabled hand.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, logging
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from wuji_tactile_bridge.sdk_loader import DEFAULT_SDK_SITE_PACKAGES


_logger = logging.get_logger(__name__)


def _spawn_rviz_host(context):
    hand_name = LaunchConfiguration("hand_name").perform(context).strip("/")
    hand_side = LaunchConfiguration("hand_side").perform(context).strip().lower()
    if hand_side not in ("left", "right"):
        _logger.error("hand_side must be 'left' or 'right'; RViz host will not start")
        return []

    try:
        publish_frequency = float(
            LaunchConfiguration("robot_state_publish_frequency").perform(context)
        )
    except ValueError:
        publish_frequency = 50.0
    if publish_frequency <= 0.0:
        publish_frequency = 50.0

    description_dir = get_package_share_directory("wuji_description")
    urdf_path = os.path.join(description_dir, "urdf", f"{hand_side}-ros.urdf")
    try:
        with open(urdf_path, "r", encoding="utf-8") as urdf_file:
            robot_description = urdf_file.read()
    except OSError as exc:
        _logger.error(f"Failed to read official Wuji URDF {urdf_path}: {exc}")
        return []

    panel_dir = get_package_share_directory("wuji_rviz_panel")
    rviz_config = os.path.join(panel_dir, "config", f"live_hand_control_{hand_side}.rviz")
    robot_state_publisher = Node(
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
        remappings=[("joint_states", "joint_states_visual")],
        output="screen",
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        namespace=hand_name,
        arguments=["-d", rviz_config],
        parameters=[{"wuji_hand_name": hand_name}],
        output="screen",
    )
    close_with_rviz = RegisterEventHandler(
        OnProcessExit(
            target_action=rviz,
            on_exit=[EmitEvent(event=Shutdown(reason="Integrated RViz window exited"))],
        )
    )
    return [robot_state_publisher, close_with_rviz, rviz]


def generate_launch_description() -> LaunchDescription:
    hand_name = LaunchConfiguration("hand_name")
    hand_serial = LaunchConfiguration("hand_serial_number")
    tactile_serial = LaunchConfiguration("tactile_serial_number")
    hand_side = LaunchConfiguration("hand_side")
    sdk_site_packages = LaunchConfiguration("sdk_site_packages")
    visual_joint_publish_hz = LaunchConfiguration("visual_joint_publish_hz")
    visual_joint_low_pass_hz = LaunchConfiguration("visual_joint_low_pass_hz")

    backend = Node(
        package="wuji_tactile_bridge",
        executable="wuji_live_hand_control",
        name="live_hand_control",
        output="screen",
        parameters=[
            {
                "hand_name": hand_name,
                "hand_serial_number": hand_serial,
                "tactile_serial_number": tactile_serial,
                "sdk_site_packages": sdk_site_packages,
                "expected_hand_side": hand_side,
                "visual_joint_publish_hz": visual_joint_publish_hz,
                "visual_joint_low_pass_hz": visual_joint_low_pass_hz,
            }
        ],
    )
    close_if_backend_exits = RegisterEventHandler(
        OnProcessExit(
            target_action=backend,
            on_exit=[EmitEvent(event=Shutdown(reason="Single-SDK backend exited"))],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("hand_name", default_value="hand_0"),
            DeclareLaunchArgument("hand_serial_number", default_value="336636733434"),
            DeclareLaunchArgument("tactile_serial_number", default_value="WT1JC01260720019"),
            DeclareLaunchArgument(
                "hand_side",
                default_value="left",
                description="Expected physical side and matching RViz model; validated by the SDK owner.",
            ),
            DeclareLaunchArgument(
                "visual_joint_publish_hz",
                default_value="50.0",
                description="Rate of the visualization-only filtered joint-state topic.",
            ),
            DeclareLaunchArgument(
                "visual_joint_low_pass_hz",
                default_value="5.0",
                description="Low-pass cutoff for RViz posture only; control keeps raw SDK values.",
            ),
            DeclareLaunchArgument(
                "robot_state_publish_frequency",
                default_value="50.0",
                description="Maximum TF publish frequency for robot_state_publisher.",
            ),
            DeclareLaunchArgument(
                "sdk_site_packages",
                default_value=DEFAULT_SDK_SITE_PACKAGES,
                description="Verified official wuji-sdk site-packages location.",
            ),
            backend,
            close_if_backend_exits,
            TimerAction(period=2.0, actions=[OpaqueFunction(function=_spawn_rviz_host)]),
        ]
    )
