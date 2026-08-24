#!/usr/bin/env python3
"""Read-only ROS 2 bridge for a Wuji Hand paired tactile glove.

This process uses only official ``wuji_sdk`` discovery, connection, hand-side,
joint-state, tactile attachment, device-info, and pressure-subscription APIs.
It never imports or calls any joint command, enable, disable, homing,
calibration, or firmware API.
"""

from __future__ import annotations

import math
import queue
import threading
import time
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from wuji_tactile_msgs.msg import TactilePressureFrame

from .sdk_loader import DEFAULT_SDK_SITE_PACKAGES, import_wuji_sdk


# Only layouts that are explicitly documented or observed from this paired
# tactile API are accepted.  Any other length is rejected rather than guessed.
KNOWN_LAYOUTS: Dict[int, Tuple[int, int]] = {
    620: (20, 31),  # Current Wuji Hand SDK documentation.
    768: (24, 32),  # Layout returned by this machine's paired glove/firmware.
}


# The order matches the official Wuji SDK's finger-major HandJointStates
# stream and the official ROS driver / description packages.
JOINT_SUFFIXES: Tuple[str, ...] = (
    "finger1_joint1", "finger1_joint2", "finger1_joint3", "finger1_joint4",
    "finger2_joint1", "finger2_joint2", "finger2_joint3", "finger2_joint4",
    "finger3_joint1", "finger3_joint2", "finger3_joint3", "finger3_joint4",
    "finger4_joint1", "finger4_joint2", "finger4_joint3", "finger4_joint4",
    "finger5_joint1", "finger5_joint2", "finger5_joint3", "finger5_joint4",
)


@dataclass(frozen=True)
class RawFrame:
    handedness: int
    sequence: int
    timestamp_ms: int
    pressure: Tuple[float, ...]
    rows: int
    cols: int


@dataclass(frozen=True)
class RawJointState:
    """One official SDK HandJointStates frame, kept separate from ROS time."""

    sequence: int
    timestamp_us: int
    position: Tuple[float, ...]
    velocity: Tuple[float, ...]
    effort: Tuple[float, ...]


class TactileBridge(Node):
    """Owns one official SDK Hand connection and republishes read-only data."""

    def __init__(self) -> None:
        super().__init__("tactile_bridge")

        self.declare_parameter("hand_name", "hand_0")
        # ros2 run parses an unquoted all-digit serial as an integer.  Accept
        # that launch/CLI representation, then normalize it below without
        # changing the actual SDK serial value used for discovery.
        self.declare_parameter(
            "hand_serial_number",
            "336636733434",
            descriptor=ParameterDescriptor(dynamic_typing=True),
        )
        self.declare_parameter("tactile_serial_number", "WT1JC01260720019")
        self.declare_parameter("sdk_site_packages", DEFAULT_SDK_SITE_PACKAGES)
        self.declare_parameter("publish_joint_states", False)
        self.declare_parameter("expected_hand_side", "")
        self.declare_parameter("handedness", "")
        self.declare_parameter("reconnect_interval_s", 2.0)
        self.declare_parameter("poll_sleep_ms", 2.0)

        self.hand_name = str(self.get_parameter("hand_name").value).strip("/")
        self.hand_serial = str(self.get_parameter("hand_serial_number").value)
        self.tactile_serial = str(self.get_parameter("tactile_serial_number").value)
        self._sdk_site_packages = str(self.get_parameter("sdk_site_packages").value)
        self._publish_joint_states = bool(self.get_parameter("publish_joint_states").value)
        self._expected_hand_side = str(
            self.get_parameter("expected_hand_side").value
        ).strip().lower()
        if self._expected_hand_side not in ("", "left", "right"):
            raise ValueError("expected_hand_side must be '', 'left', or 'right'")
        self._reconnect_interval_s = max(
            0.5, float(self.get_parameter("reconnect_interval_s").value)
        )
        self._poll_sleep_s = max(0.001, float(self.get_parameter("poll_sleep_ms").value) / 1000.0)

        self.publisher = self.create_publisher(
            TactilePressureFrame,
            f"/{self.hand_name}/tactile/pressure",
            qos_profile_sensor_data,
        )
        self._joint_state_publisher = (
            self.create_publisher(
                JointState,
                f"/{self.hand_name}/joint_states",
                qos_profile_sensor_data,
            )
            if self._publish_joint_states
            else None
        )

        self._sdk = None
        self._manager = None
        self._hand = None
        self._subscription = None
        self._joint_subscription = None
        self._reader_thread: Optional[threading.Thread] = None
        self._stop_reader = threading.Event()
        self._frames: queue.Queue[RawFrame] = queue.Queue(maxsize=8)
        self._joint_frames: queue.Queue[RawJointState] = queue.Queue(maxsize=8)
        self._events: queue.Queue[Tuple[str, str]] = queue.Queue()
        self._last_connect_attempt = 0.0
        self._last_publish_log = 0.0
        self._published_frames = 0
        self._published_joint_frames = 0
        self._unknown_layout_counts: Dict[int, int] = {}
        self._invalid_joint_counts: Dict[int, int] = {}
        self._joint_names: Tuple[str, ...] = ()

        self._publish_timer = self.create_timer(0.005, self._publish_queued_frames)
        self._connection_timer = self.create_timer(0.25, self._connection_tick)
        data_paths = (
            "joint_states + tactile pressure"
            if self._publish_joint_states
            else "tactile pressure"
        )
        self.get_logger().info(
            f"Read-only SDK data bridge created ({data_paths}): it has no joint-command "
            "publisher and no motor-control API."
        )
        self._connection_tick()

    @staticmethod
    def _layout_for_count(count: int) -> Optional[Tuple[int, int]]:
        return KNOWN_LAYOUTS.get(count)

    def _connection_tick(self) -> None:
        while True:
            try:
                level, detail = self._events.get_nowait()
            except queue.Empty:
                break
            if level == "error":
                self.get_logger().error(detail)
            elif level == "warn":
                self.get_logger().warn(detail)
            else:
                self.get_logger().info(detail)

        if self._reader_thread is not None and not self._reader_thread.is_alive():
            self._disconnect_tactile()

        if self._hand is not None:
            return
        now = time.monotonic()
        if now - self._last_connect_attempt < self._reconnect_interval_s:
            return
        self._last_connect_attempt = now
        self._connect_tactile()

    def _connect_tactile(self) -> None:
        hand = None
        tactile_subscription = None
        joint_subscription = None
        try:
            if self._sdk is None:
                self._sdk = import_wuji_sdk(self._sdk_site_packages)
                self._manager = self._sdk.SdkManager.instance()

            devices = self._manager.scan()
            hands = [
                device
                for device in devices
                if device.device_type == self._sdk.DeviceType.WujiHand and device.sn == self.hand_serial
            ]
            if len(hands) != 1:
                self.get_logger().warn(
                    f"Waiting for Wuji Hand serial {self.hand_serial}; scan found "
                    f"{[(device.sn, str(device.device_type)) for device in devices]}"
                )
                return

            hand = self._manager.connect(
                sn=self.hand_serial,
                device_name=f"{self.hand_name}_tactile_bridge",
            )
            if not hand.is_tactile_attached():
                hand.disconnect()
                self.get_logger().warn(
                    "Wuji Hand connected but no paired tactile glove is attached. "
                    "The bridge will retry; re-plugging the glove requires a hand reconnect."
                )
                return

            actual_hand_side = str(hand.handedness_name()).strip().lower()
            if actual_hand_side not in ("left", "right"):
                hand.disconnect()
                self.get_logger().error(
                    f"Hand reported unsupported handedness {actual_hand_side!r}; refusing to publish."
                )
                return
            if (
                self._expected_hand_side
                and self._expected_hand_side != actual_hand_side
            ):
                hand.disconnect()
                self.get_logger().error(
                    f"Handedness mismatch: launch expects {self._expected_hand_side}, "
                    f"SDK reports {actual_hand_side}. Refusing to publish a mismatched URDF."
                )
                return

            info = hand.tactile.device_info()
            actual_tactile_serial = str(info.serial)
            if self.tactile_serial and actual_tactile_serial != self.tactile_serial:
                hand.disconnect()
                self.get_logger().error(
                    f"Paired tactile serial mismatch: expected {self.tactile_serial}, "
                    f"got {actual_tactile_serial}. Refusing to publish."
                )
                return

            # Both subscriptions use this exact official SDK Hand handle.  A
            # second process cannot claim the same USB device while the
            # official C++ driver owns it, so this optional data-only mode is
            # the safe one-claim route for joint feedback plus paired tactile.
            if self._publish_joint_states:
                joint_subscription = hand.joint_states().subscribe()
            tactile_subscription = hand.tactile.subscribe_pressure_frame()
            self._stop_reader.clear()
            self._hand = hand
            self._subscription = tactile_subscription
            self._joint_subscription = joint_subscription
            self._joint_names = tuple(
                f"{actual_hand_side}_{suffix}" for suffix in JOINT_SUFFIXES
            )
            self.set_parameters([Parameter("handedness", value=actual_hand_side)])
            self._reader_thread = threading.Thread(
                target=self._reader_loop,
                args=(tactile_subscription, joint_subscription),
                name="wuji-tactile-reader",
                daemon=True,
            )
            self._reader_thread.start()
            self.get_logger().info(
                f"Connected read-only SDK stream: hand={hand.serial_number} ({actual_hand_side}), "
                f"tactile={actual_tactile_serial}, topic=/{self.hand_name}/tactile/pressure"
            )
        except Exception as exc:  # Hardware / USB state is external to ROS.
            # A failure between SDK connect() and assigning self._hand must not
            # leave an otherwise invisible read-only SDK connection open.
            if hand is not None and hand is not self._hand:
                try:
                    hand.disconnect()
                except Exception:
                    pass
            for active_subscription in (joint_subscription, tactile_subscription):
                if (
                    active_subscription is not None
                    and active_subscription is not self._subscription
                    and active_subscription is not self._joint_subscription
                ):
                    try:
                        active_subscription.close()
                    except Exception:
                        pass
            self._disconnect_tactile()
            self.get_logger().warn(f"Read-only tactile connection attempt failed: {exc}")

    @staticmethod
    def _enqueue_latest(target: queue.Queue, item) -> None:
        """Keep low-latency data when ROS publishing falls behind the SDK."""
        while True:
            try:
                target.put_nowait(item)
                return
            except queue.Full:
                try:
                    target.get_nowait()
                except queue.Empty:
                    return

    def _queue_joint_state(self, state) -> None:
        position = tuple(float(value) for value in state.position)
        positions_are_finite = all(math.isfinite(value) for value in position)
        if len(position) != len(JOINT_SUFFIXES) or not positions_are_finite:
            count = len(position)
            occurrences = self._invalid_joint_counts.get(count, 0) + 1
            self._invalid_joint_counts[count] = occurrences
            if occurrences == 1 or occurrences % 500 == 0:
                self._events.put(
                    (
                        "error",
                        f"Refusing invalid SDK joint state: values={count}, finite="
                        f"{positions_are_finite} ({occurrences} rejected frames).",
                    )
                )
            return

        velocity = tuple(float(value) for value in state.velocity)
        effort = tuple(float(value) for value in state.effort)
        # The SDK represents unavailable optional channels as length 0.  Do
        # not manufacture values for an unknown or partial channel.
        if len(velocity) != len(JOINT_SUFFIXES) or not all(math.isfinite(value) for value in velocity):
            velocity = ()
        if len(effort) != len(JOINT_SUFFIXES) or not all(math.isfinite(value) for value in effort):
            effort = ()
        header = getattr(state, "header", None)
        self._enqueue_latest(
            self._joint_frames,
            RawJointState(
                sequence=int(getattr(header, "seq", 0)),
                timestamp_us=int(getattr(header, "timestamp_us", 0)),
                position=position,
                velocity=velocity,
                effort=effort,
            ),
        )

    def _reader_loop(self, subscription, joint_subscription) -> None:
        """Receive SDK data without blocking the ROS executor or Tk UI."""
        try:
            while not self._stop_reader.is_set():
                received = False
                if joint_subscription is not None:
                    joint_state = joint_subscription.recv()
                    if joint_state is not None:
                        self._queue_joint_state(joint_state)
                        received = True

                frame = subscription.recv()
                if frame is None:
                    if not received:
                        self._stop_reader.wait(self._poll_sleep_s)
                    continue

                pressure = tuple(float(value) for value in frame.pressure)
                layout = self._layout_for_count(len(pressure))
                if layout is None:
                    count = len(pressure)
                    occurrences = self._unknown_layout_counts.get(count, 0) + 1
                    self._unknown_layout_counts[count] = occurrences
                    if occurrences == 1 or occurrences % 500 == 0:
                        self._events.put(
                            (
                                "error",
                                f"Refusing unknown tactile layout: received {count} values "
                                f"({occurrences} rejected frames). No rows/cols or 3D coordinates "
                                "will be guessed.",
                            )
                        )
                    self._stop_reader.wait(self._poll_sleep_s)
                    continue
                rows, cols = layout
                self._enqueue_latest(
                    self._frames,
                    RawFrame(
                        handedness=int(frame.handedness),
                        sequence=int(frame.sequence),
                        timestamp_ms=int(frame.timestamp_ms),
                        pressure=pressure,
                        rows=rows,
                        cols=cols,
                    ),
                )
        except Exception as exc:  # USB disconnect or SDK stream failure.
            if not self._stop_reader.is_set():
                self._events.put(("warn", f"SDK data subscription ended: {exc}"))

    def _publish_queued_frames(self) -> None:
        while True:
            try:
                joint_frame = self._joint_frames.get_nowait()
            except queue.Empty:
                break
            if self._joint_state_publisher is None or len(self._joint_names) != len(JOINT_SUFFIXES):
                continue
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = list(self._joint_names)
            message.position = list(joint_frame.position)
            message.velocity = list(joint_frame.velocity)
            message.effort = list(joint_frame.effort)
            self._joint_state_publisher.publish(message)
            self._published_joint_frames += 1

        while True:
            try:
                frame = self._frames.get_nowait()
            except queue.Empty:
                break
            message = TactilePressureFrame()
            message.header.stamp = self.get_clock().now().to_msg()
            # This frame ID identifies only a 2D data grid; it is deliberately
            # not an RViz/TF spatial frame because no official taxel geometry
            # was found for the paired 20x31/24x32 pressure stream.
            message.header.frame_id = f"{self.hand_name}_tactile_grid_{frame.rows}x{frame.cols}"
            message.handedness = frame.handedness
            message.sequence = frame.sequence
            message.device_timestamp_ms = frame.timestamp_ms
            message.rows = frame.rows
            message.cols = frame.cols
            message.pressure = list(frame.pressure)
            self.publisher.publish(message)
            self._published_frames += 1

            now = time.monotonic()
            if now - self._last_publish_log >= 5.0:
                finite = [value for value in frame.pressure if math.isfinite(value)]
                if finite:
                    self.get_logger().info(
                        f"Publishing {frame.rows}x{frame.cols} tactile frames; "
                        f"seq={frame.sequence}, max={max(finite):.4f}, "
                        f"mean={sum(finite) / len(finite):.4f}, frames={self._published_frames}"
                    )
                self._last_publish_log = now

    def _disconnect_tactile(self) -> None:
        self._stop_reader.set()
        subscription = self._subscription
        self._subscription = None
        joint_subscription = self._joint_subscription
        self._joint_subscription = None
        for active_subscription in (joint_subscription, subscription):
            if active_subscription is not None:
                try:
                    active_subscription.close()
                except Exception:
                    pass

        reader = self._reader_thread
        self._reader_thread = None
        if reader is not None and reader.is_alive():
            reader.join(timeout=2.0)

        hand = self._hand
        self._hand = None
        if hand is not None:
            try:
                hand.disconnect()
            except Exception:
                pass

        while True:
            try:
                self._frames.get_nowait()
            except queue.Empty:
                break
        while True:
            try:
                self._joint_frames.get_nowait()
            except queue.Empty:
                break

    def destroy_node(self) -> bool:
        self._disconnect_tactile()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TactileBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
