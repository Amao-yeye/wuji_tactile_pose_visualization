# Hardware safety

## Scope

The production backend is command-capable. This document describes safety
constraints; it is not authorization to run hardware tests automatically.
Agents and offline validation must use only `--show-args`, interface
inspection, and `--self-test`.

## Exclusive USB ownership

The official C++ driver and a Python `WujiHand` connection both claim the
physical USB device exclusively. A previous simultaneous test returned
`USB device busy`. Stop one mode completely before starting another.

Never run these together for the same hand:

- production `live_hand_control_rviz.launch.py`;
- read-only `wuji_hand_tactile.launch.py`;
- `wujihand_read_only.launch.py`;
- `wuji_hand_control.launch.py`;
- any official Python example that opens the hand.

## Startup and Enable

The production process starts in `DISABLED`. Startup creates no automatic
motion. `Enable` requires:

1. one connected hand with the expected handedness;
2. a fresh, finite, 20-element actual joint state;
3. the proven SDK-index to URDF joint order;
4. valid firmware soft limits.

JSON **Move to Pose** additionally requires a valid pose library and selected
pose. A pose-library error blocks JSON pose execution without weakening the
existing final Cup Grasp target or read-only visualization paths.

`Enable` captures the current actual position as the Cup Grasp reference
and enables motors, but does not send a target. Selecting a pose in
the combo box also sends no command; only an explicit **Move to Pose** request
can start that motion.

## Motion gates

- Every trajectory starts from the latest actual 20D joint feedback, never the
  previous target.
- Only one trajectory may run.
- Moving disables pose selection, Move to Pose, and the Open shortcut.
- Stale feedback or backend status blocks new motion.
- Disable has priority, clears queued/active motion, and calls
  `hand.disable()`.
- Loss of the RViz heartbeat for more than one second after remote motion has
  been enabled stops motion and disables the hand.
- Persistent joint-state loss auto-disables.

Current named-motion parameters:

| Parameter | Value |
| --- | ---: |
| Control update | 100 Hz |
| SDK low-pass cutoff | 5 Hz |
| Maximum command step | 0.003 rad/cycle |
| Minimum trajectory duration | 5.0 s |
| Maximum planned smoothstep peak speed | 0.144 rad/s |
| Target settle | 0.5 s |
| Firmware-limit margin | 0.08 rad |
| Required feedback freshness | 0.25 s |
| Auto-disable after persistent feedback loss | 0.50 s |

Interpolation is cubic smoothstep followed by per-cycle step limiting and
firmware-safe clamping. Tactile data is visualization only; it never
automatically closes the hand.

## Pose safety

The library contains:

```text
open
relaxed
four_finger_90
fist
thumb_index_touch
thumb_middle_touch
thumb_ring_touch
thumb_pinky_touch
tripod
index_point
book_flick_ready
```

Static JSON/model validation does not constitute hardware validation. Move one
pose at a time at the existing conservative speed and return to Open when an
operator judges that necessary; the controller does not require an automatic
return-to-zero between valid pose moves.

A red flashing Pre-fault was previously reported while testing
`thumb_pinky_touch`. Treat that pose as **not cleared for further hardware
testing** until temperature/fault logs and the affected joints are reviewed.
Do not auto-clear faults or automatically modify its qpos.

The Panel dropdown contains `Cup Grasp` and the JSON poses other than the
duplicate `open` entry. The independent **Open** shortcut sends the same
`MOVE_POSE + open` request used by a named pose. **Cup Grasp** sends one
`MOVE_POSE + cup_grasp` request to the existing verified final target; it
does not expose intermediate close levels or bypass safe interpolation.

## Shutdown and power-off

1. Stop any trajectory and make the UI report `DISABLED`.
2. Close RViz or press Ctrl-C and wait for the backend process to exit.
3. Confirm the hand is stationary and no SDK/driver process owns the USB
   device.
4. Only then disconnect USB or power.

If there is an abnormal sound, obstruction, communication failure, Pre-fault,
or worsening red indication, stop, request Disable if communication is still
healthy, and do not retry automatically.

The next hardware stage is one manually observed smoke test. This repository
cleanup does not perform it.
