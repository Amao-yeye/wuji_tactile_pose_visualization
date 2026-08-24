# Wuji Hand safe ROS 2 debug environment

This local package is built around the official `wujihandros2` driver and
official `wuji_description` URDF. It targets one Wuji Hand at serial
`336636733434` under the `/hand_0` namespace.

## Always use Kilted

```bash
source /opt/ros/kilted/setup.bash
source /home/oem/Workspaces/wuji/ros2_ws/install/setup.bash
```

Do not source Jazzy in the same shell.

## Strict read-only visualization

```bash
ros2 launch wujihand_bringup wujihand_read_only.launch.py
```

This launch hard-codes `read_only:=true`. The patched driver then:

- publishes `/hand_0/joint_states` from direct hardware reads;
- starts no realtime controller, command subscription, SetEnabled service, or
  ResetError service;
- makes no device write on connection or shutdown;
- starts `robot_state_publisher` and the official left/right RViz config.

## Command-capable debugging session

```bash
ros2 launch wuji_hand_debug wuji_hand_control.launch.py
```

This starts the patched driver in command-capable mode, official
`robot_state_publisher`, official RViz, and `wuji_hand_control_gui`. It still
does **not** enable motors or publish a target at startup.

The GUI safety gate is:

1. It waits for exactly 20 finite live joint positions.
2. It initializes every command target from those measured values.
3. It keeps sliders, single-joint tests, and gesture buttons disabled until an operator confirms
   **Enable**.
4. On Enable, it sends eight measured-pose hold messages before calling the
   official `/{hand_name}/set_enabled` service with `(255, 255, true)`.
5. Slider events are capped to 0.03 rad, with a 0.20 rad window from the last
   `Read Current Position` sync and the official URDF joint limits.
6. Single-joint `±0.02 rad` tests require a second confirmation and hold the
   other 19 target positions.
7. **Fist +0.050 rad** is a deliberately limited incremental curl, not the
   upstream zero-based wave demo: it acts on thumb J3/J4 and F2--F5
   MCP/PIP/DIP joints. Thumb J1/J2 remain available through explicit
   single-joint controls because their functions/directions differ. Every
   gesture ramps by at most 0.020 rad every 60 ms and cannot exceed 0.20 rad
   from the last measured-pose capture. **Release captured pose** ramps those
   same joints back to the pose captured by the last `Read Current Position`
   or `Enable` action. Each gesture needs a confirmation, and **Disable**
   cancels any remaining ramp before removing motor power.

Use **Disable** and wait for the service confirmation before closing the GUI.
The GUI refuses to close while its control state is active.
