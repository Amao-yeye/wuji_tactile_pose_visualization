# Wuji Hand ROS 2 control and tactile workspace

This repository is the local ROS 2 Kilted workspace for Wuji Hand 1 control,
joint-state visualization, and paired tactile visualization. The production
UI is one RViz window backed by one Python SDK owner.

## Clone and restore dependencies

Clone the project together with its pinned official dependencies:

```bash
git clone --recurse-submodules \
  git@github.com:Amao-yeye/wuji_tactile_pose_visualization.git
cd wuji_tactile_pose_visualization
```

The repository preserves the local read-only safety changes for the official
ROS driver as a reviewed patch. Apply it once after a fresh clone:

```bash
git -C ros2_ws/src/wujihandros2 apply \
  ../../../patches/wujihandros2-read-only.patch
```

This keeps the official dependency history separate while reproducing the
eight-package workspace used by this project.

## Environment

Use ROS 2 **Kilted** only:

```bash
source /opt/ros/kilted/setup.bash
source /home/oem/Workspaces/wuji/ros2_ws/install/setup.bash
```

Do not source ROS 2 Jazzy in the same shell. Do not activate the SDK `.venv`
in the ROS shell; the bridge imports the verified SDK from its configured
site-packages path after Kilted's Python modules are available.

## Start the integrated UI

Power the Wuji Hand, connect its USB cable, and make sure no other Wuji
controller is running. Then open a fresh terminal and run:

```bash
cd /home/oem/Workspaces/wuji/ros2_ws
source /opt/ros/kilted/setup.bash
source install/setup.bash

ros2 launch wuji_tactile_bridge \
  live_hand_control_rviz.launch.py
```

The integrated RViz UI starts with the joints `DISABLED`; motion requires an
explicit click on `Enable`. Before exiting, click `Disable`, then close RViz or
press `Ctrl+C` in the launch terminal. Run only one controller instance for the
physical hand.

## Runtime modes

The physical USB hand is exclusive: run exactly one mode at a time.

| Role | Command | Owner | Purpose |
| --- | --- | --- | --- |
| Production | `ros2 launch wuji_tactile_bridge live_hand_control_rviz.launch.py` | one Python SDK handle | RViz RobotModel, tactile maps, pose library, and safety-gated motion |
| Read-only tactile | `ros2 launch wuji_tactile_bridge wuji_hand_tactile.launch.py` | one Python SDK handle | Joint/tactile acquisition with a separate read-only heatmap |
| Read-only driver diagnostic | `ros2 launch wujihand_bringup wujihand_read_only.launch.py` | official C++ driver | Joint feedback and URDF/RViz validation |
| Command diagnostic | `ros2 launch wuji_hand_debug wuji_hand_control.launch.py` | official C++ driver | Deliberately gated single-joint development controls |

The production, read-only, and diagnostic modes must never be launched
together for the same physical hand.

The production launch is hardware-capable. It starts in `DISABLED` and sends
no position target on startup, but it must only be launched by an operator who
has read [hardware_safety.md](docs/hardware_safety.md).

## Build

```bash
cd /home/oem/Workspaces/wuji/ros2_ws
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

The workspace contains eight packages:

- local: `wuji_hand_debug`, `wuji_rviz_panel`,
  `wuji_tactile_bridge`, `wuji_tactile_msgs`;
- official: `wujihand_bringup`, `wujihand_driver`,
  `wujihand_msgs`, `wuji_description`.

## Offline checks

These commands do not start a hardware node:

```bash
ros2 launch wuji_tactile_bridge live_hand_control_rviz.launch.py --show-args
ros2 run wuji_tactile_bridge wuji_live_hand_control --self-test
ros2 interface show wuji_tactile_msgs/msg/HandControlStatus
ros2 interface show wuji_tactile_msgs/srv/HandControlCommand
```

The self-test must finish with:

```text
live_hand_control self-test passed (no SDK connection, no motor calls)
```

## Documentation

- [Architecture](docs/architecture.md): runtime graph, ROS interfaces,
  tactile threading, and pose-library contract.
- [Hardware safety](docs/hardware_safety.md): motion gates, limits,
  shutdown, and hardware-validation status.
- [Diagnostics](docs/diagnostics.md): Kilted checks, offline verification,
  read-only tools, and troubleshooting.

Official dependencies are pinned as Git submodules:

- `third_party/wujihandpy`: official Python API/examples, pinned to v1.8.0;
- `ros2_ws/src/wujihandros2`: official ROS 2 stack; the project-specific
  read-only safety changes are preserved in
  `patches/wujihandros2-read-only.patch`.
