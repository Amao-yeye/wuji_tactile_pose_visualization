# Architecture

## Production data and control path

```text
live_hand_control_rviz.launch.py
        |
        +-- wuji_live_hand_control
        |      |
        |      +-- one Wuji SDK handle and one serialized SDK worker
        |      +-- raw joint/tactile subscriptions
        |      +-- constrained motion and pose-library validation
        |      +-- HandControlCommand service
        |      +-- HandControlStatus publisher
        |      +-- joint_states and joint_states_visual publishers
        |
        +-- robot_state_publisher
        |      +-- joint_states_visual -> TF -> RobotModel
        |
        +-- RViz + wuji_rviz_panel/WujiHandControlPanel
               +-- tactile subscriber and fixed short buffer
               +-- Qt 20 ms timer -> existing heatmap widgets
               +-- pose selector and safety-gated command client
```

The RViz panel is a ROS client. It does not own an SDK object and does not
publish arbitrary joint targets. The Python backend is the only process that
can write to the hardware in production mode.

## ROS interfaces

For the default `hand_0` namespace:

| Name | Type | Purpose |
| --- | --- | --- |
| `/hand_0/tactile/pressure` | `wuji_tactile_msgs/msg/TactilePressureFrame` | Original-rate paired tactile frames |
| `/hand_0/joint_states` | `sensor_msgs/msg/JointState` | Newest unfiltered SDK feedback |
| `/hand_0/joint_states_visual` | `sensor_msgs/msg/JointState` | 50 Hz, 5 Hz low-pass visualization stream |
| `/hand_0/hand_control/status` | `wuji_tactile_msgs/msg/HandControlStatus` | Read-only backend state |
| `/hand_0/hand_control/command` | `wuji_tactile_msgs/srv/HandControlCommand` | Named safety-gated actions |
| `/hand_0/hand_control/ui_heartbeat` | `std_msgs/msg/Empty` | Production-panel watchdog heartbeat |

`HandControlCommand` accepts only Enable, Disable, and Move to Pose. Named
pose IDs cover the read-only JSON library plus the existing final `cup_grasp`
target. The interface contains no arbitrary position, effort, current, or
limit vector.

## Tactile threading and rendering

The SDK/ROS tactile frequency is not reduced. The RViz panel creates a tactile
callback group with automatic executor association disabled and schedules that
group on its own `SingleThreadedExecutor`. The callback only buffers frames;
it never touches a Qt object.

The Qt GUI thread runs a precise 20 ms timer (nominally 50 Hz), drains the
short buffer, aggregates the strongest absolute adjacent-frame transient, and
updates existing `QImage`/widget state. It does not recreate widgets or run
`QApplication.processEvents()`.

The three views are:

- **Raw Pressure**: current finite SDK values;
- **Baseline Residual**: positive residual from an independent five-second,
  per-taxel median baseline; a following independent five-second no-contact
  phase sets the display threshold to the finite residual P99.9;
- **Temporal Peak |Delta|**: maximum absolute adjacent-frame change observed
  during one GUI refresh interval.

Known layouts are 20 x 31 and the observed paired 24 x 32 layout. SDK NaNs
remain invalid. There is no inferred taxel-to-URDF geometry, PointCloud2,
MarkerArray, or tactile TF.

At startup the panel performs one GUI-thread layout stabilization: it activates
the relevant layouts, briefly nudges the top-level and RViz render-panel width,
restores both sizes after 16 ms, and queues rendering. This is production logic
that prevents the verified initial RViz flicker; it does not change steady-state
render frequency.

## Pose library

The canonical file is installed with `wuji_tactile_bridge`:

```text
share/wuji_tactile_bridge/config/poses/wuji_hand1_left_pose_library.json
```

The backend resolves it through `ament_index_python` package-share lookup.
It does not depend on the current working directory, a fixed workspace path,
or upward directory searches.

The JSON contract is:

- `hand.model == "Wuji Hand 1"`;
- `hand.handedness == "left"`;
- `hand.dof == 20`;
- `hand.unit == "rad"`;
- qpos uses `ordering.urdf_joint_order` exactly;
- all 11 required poses contain 20 finite values within JSON model limits.

Execution performs a second validation against runtime firmware limits with a
0.08 rad margin. A failing pose is rejected; values are never reordered,
clipped, normalized, or modified.

`Cup Grasp` is not a new JSON qpos. It resolves to the exact final target
previously used by the completed 3/3 constrained-grasp workflow, based on the
Enable-time measured reference. It now executes in one user request through
the same named-pose interpolation, step limit, and runtime safety checks.

## Module responsibilities

| Module | Responsibility |
| --- | --- |
| `live_hand_control.py` | Production SDK owner, motion state, pose loader, tactile/joint publishing |
| `tactile_bridge.py` | Strictly read-only SDK bridge and shared raw frame/layout types |
| `tactile_heatmap.py` | Standalone read-only GUI plus shared `TactileSnapshot` |
| `sdk_loader.py` | Loads the verified SDK path without replacing ROS Python modules |
| `wuji_rviz_panel` | Integrated Qt/RViz rendering and command client |
| `wuji_tactile_msgs` | Tactile and generic hand-control ROS interfaces |

`tactile_heatmap.py` intentionally remains because the read-only diagnostic
mode uses its standalone GUI and the production backend imports its immutable
snapshot type.
