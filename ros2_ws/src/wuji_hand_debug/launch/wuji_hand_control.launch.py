"""One-command command-capable Wuji Hand debug session.

The included official driver is intentionally command-capable but never enables
motors or publishes a target on startup. The GUI remains locked until it has
received all 20 live joint states and an operator explicitly confirms Enable.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    hand_name_arg = DeclareLaunchArgument(
        "hand_name", default_value="hand_0", description="Namespace for the connected hand"
    )
    serial_number_arg = DeclareLaunchArgument(
        "serial_number",
        default_value="336636733434",
        description="Expected Wuji Hand serial number",
    )
    publish_rate_arg = DeclareLaunchArgument(
        "publish_rate",
        default_value="1000.0",
        description="Driver state publication rate in Hz",
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="true", description="Start the official RViz configuration"
    )

    official_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("wujihand_bringup"),
                "launch",
                "wujihand.launch.py",
            )
        ),
        launch_arguments={
            "hand_name": LaunchConfiguration("hand_name"),
            "serial_number": LaunchConfiguration("serial_number"),
            "publish_rate": LaunchConfiguration("publish_rate"),
            # The patched driver never auto-enables or sends an initial target.
            # This only exposes the official command topic and SetEnabled service
            # for the GUI after an explicit user action.
            "read_only": "false",
            "rviz": LaunchConfiguration("rviz"),
            "foxglove": "false",
        }.items(),
    )

    gui = Node(
        package="wuji_hand_debug",
        executable="wuji_hand_control_gui",
        name="wuji_hand_control_gui",
        arguments=["--hand-name", LaunchConfiguration("hand_name")],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription(
        [hand_name_arg, serial_number_arg, publish_rate_arg, rviz_arg, official_bringup, gui]
    )
