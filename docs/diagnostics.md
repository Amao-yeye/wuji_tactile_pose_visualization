# Diagnostics and offline validation

## Pure Kilted gate

Every build and runtime shell must satisfy:

```bash
echo "$ROS_DISTRO"
which ros2
printenv | grep -i jazzy
```

Expected:

```text
kilted
/opt/ros/kilted/bin/ros2
# no Jazzy output
```

Use:

```bash
source /opt/ros/kilted/setup.bash
source /home/oem/Workspaces/wuji/ros2_ws/install/setup.bash
```

## Clean build

```bash
cd /home/oem/Workspaces/wuji/ros2_ws
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

A complete build must report eight finished packages.

## Offline verification

These checks parse files or run the backend's early no-SDK self-test path:

```bash
ros2 launch wuji_tactile_bridge live_hand_control_rviz.launch.py --show-args
ros2 run wuji_tactile_bridge wuji_live_hand_control --self-test
ros2 interface show wuji_tactile_msgs/msg/HandControlStatus
ros2 interface show wuji_tactile_msgs/srv/HandControlCommand
```

Do not run the launch without `--show-args` during offline validation.

## Read-only and development entrypoints

These entries are retained intentionally and are not legacy code:

| Entry | Capability |
| --- | --- |
| `wuji_hand_tactile.launch.py` | Read-only SDK joint/tactile acquisition, RViz, standalone heatmap |
| `wujihand_read_only.launch.py` | Official C++ driver in locally added read-only mode |
| `wuji_hand_control.launch.py` | Gated single-joint development GUI |

Although their behavior may be read-only, the first two still open the
physical USB device. They are not offline tests.

## Runtime observations

Useful read-only checks after an operator intentionally starts one hardware
mode:

```bash
ros2 topic info -v /hand_0/tactile/pressure
ros2 topic info -v /hand_0/joint_states_visual
ros2 topic echo --once /hand_0/tactile/pressure --field rows
ros2 topic echo --once /hand_0/tactile/pressure --field cols
```

For the production panel, diagnostics are emitted every five seconds rather
than every frame. They report backend, callback, Qt timer, GUI frame, actual
render and processed rates, buffer drops, batch size, and thread IDs.

The intended display architecture is:

- high-rate backend and tactile ROS topic unchanged;
- tactile callback on its dedicated executor, no Qt access;
- Qt timer near 50 Hz;
- existing heatmap objects updated in place;
- no duplicate JointState publisher;
- no sustained RViz RobotModel flicker.

## Repository boundaries

`.venv/`, `ros2_ws/build/`, `ros2_ws/install/`, `ros2_ws/log/`, and
Python caches are generated/local state and are ignored at the repository
root.

`third_party/wujihandpy` and `ros2_ws/src/wujihandros2` are pinned Git
submodules. The local read-only driver changes are preserved separately in
`patches/wujihandros2-read-only.patch`; do not reset or clean them from an
active hardware workspace.
