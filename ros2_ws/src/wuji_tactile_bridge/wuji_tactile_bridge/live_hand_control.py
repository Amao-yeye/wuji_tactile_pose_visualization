#!/usr/bin/env python3
"""Single-SDK live tactile backend with deliberately limited cup motions.

This module is intentionally separate from :mod:`tactile_bridge`, which stays
strictly read-only.  The executable created from this module owns one and only
one official ``wuji_sdk.WujiHand`` connection.  That one handle provides both
the paired-tactile stream and joint-state stream, and all motor writes are
serialized in its SDK worker thread.

The RViz panel consumes ROS topics and services from this backend. It
has no arbitrary joint sliders, no tactile feedback control, no current/effort
writes, and no automatic movement after Enable.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import queue
import signal
import threading
import time
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from types import MappingProxyType
from typing import Dict, Mapping, Optional, Sequence, Tuple

import rclpy
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty

from wuji_tactile_msgs.msg import HandControlStatus, TactilePressureFrame
from wuji_tactile_msgs.srv import HandControlCommand

from .sdk_loader import DEFAULT_SDK_SITE_PACKAGES, import_wuji_sdk
from .tactile_bridge import JOINT_SUFFIXES, KNOWN_LAYOUTS, RawFrame, RawJointState
from .tactile_heatmap import TactileSnapshot


POSE_LIBRARY_FILENAME = "wuji_hand1_left_pose_library.json"
REQUIRED_POSE_IDS: Tuple[str, ...] = (
    "open",
    "relaxed",
    "four_finger_90",
    "fist",
    "thumb_index_touch",
    "thumb_middle_touch",
    "thumb_ring_touch",
    "thumb_pinky_touch",
    "tripod",
    "index_point",
    "book_flick_ready",
)


@dataclass(frozen=True)
class PoseLibrary:
    """Validated, read-only pose data; qpos uses canonical URDF ordering."""

    path: Path
    model: str
    handedness: str
    dof: int
    unit: str
    canonical_joint_order: Tuple[str, ...]
    poses: Mapping[str, Tuple[float, ...]]


def _ros_joint_order_for_side(hand_side: str) -> Tuple[str, ...]:
    """Reuse the backend's existing SDK-index-to-ROS-name order."""

    return tuple(f"{hand_side}_{suffix}" for suffix in JOINT_SUFFIXES)


def _default_pose_library_path() -> Path:
    """Resolve the installed pose library from this ROS package's share path."""

    package_share = Path(get_package_share_directory("wuji_tactile_bridge"))
    return package_share / "config" / "poses" / POSE_LIBRARY_FILENAME


def load_pose_library(
    path: Path, expected_urdf_joint_order: Sequence[str]
) -> PoseLibrary:
    """Load and statically validate JSON without reordering or clipping qpos."""

    source = Path(path).expanduser().resolve()
    try:
        document = json.loads(source.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"could not read pose library {source}: {exc}") from exc
    if not isinstance(document, dict):
        raise ValueError("pose library root must be a JSON object")

    hand = document.get("hand")
    if not isinstance(hand, dict):
        raise ValueError("hand must be a JSON object")
    expected_hand = {
        "model": "Wuji Hand 1",
        "handedness": "left",
        "dof": 20,
        "unit": "rad",
    }
    for field, expected in expected_hand.items():
        if hand.get(field) != expected:
            raise ValueError(
                f"hand.{field} must be {expected!r}, got {hand.get(field)!r}"
            )

    ordering = document.get("ordering")
    if not isinstance(ordering, dict):
        raise ValueError("ordering must be a JSON object")
    raw_order = ordering.get("urdf_joint_order")
    if not isinstance(raw_order, list) or len(raw_order) != 20:
        raise ValueError("ordering.urdf_joint_order must contain exactly 20 entries")
    if not all(isinstance(name, str) for name in raw_order):
        raise ValueError("ordering.urdf_joint_order entries must all be strings")
    canonical_order = tuple(raw_order)
    expected_order = tuple(expected_urdf_joint_order)
    if len(expected_order) != 20:
        raise ValueError("backend ROS joint order must contain exactly 20 entries")
    if canonical_order != expected_order:
        mismatches = [
            f"[{index}] JSON={actual!r}, backend={expected!r}"
            for index, (actual, expected) in enumerate(zip(canonical_order, expected_order))
            if actual != expected
        ]
        raise ValueError(
            "canonical URDF joint order does not exactly match the existing backend order: "
            + "; ".join(mismatches)
        )

    raw_limits = document.get("left_model_joint_limits_rad")
    if not isinstance(raw_limits, list) or len(raw_limits) != 20:
        raise ValueError("left_model_joint_limits_rad must contain exactly 20 entries")
    model_limits: Dict[int, Tuple[float, float]] = {}
    for entry in raw_limits:
        if not isinstance(entry, dict) or type(entry.get("index")) is not int:
            raise ValueError("each model-limit entry must have an integer index")
        index = entry["index"]
        if index not in range(20) or index in model_limits:
            raise ValueError(f"invalid or duplicate model-limit index {index!r}")
        if entry.get("urdf_name") != canonical_order[index]:
            raise ValueError(
                f"model-limit index {index} names {entry.get('urdf_name')!r}, "
                f"expected canonical joint {canonical_order[index]!r}"
            )
        lower_value = entry.get("lower")
        upper_value = entry.get("upper")
        if (
            isinstance(lower_value, bool)
            or isinstance(upper_value, bool)
            or not isinstance(lower_value, (int, float))
            or not isinstance(upper_value, (int, float))
        ):
            raise ValueError(f"model-limit index {index} must contain numeric bounds")
        lower = float(lower_value)
        upper = float(upper_value)
        if not math.isfinite(lower) or not math.isfinite(upper) or upper <= lower:
            raise ValueError(f"model-limit index {index} has invalid bounds {lower}..{upper}")
        model_limits[index] = (lower, upper)
    if set(model_limits) != set(range(20)):
        raise ValueError("model-limit indices must be exactly 0..19")

    raw_poses = document.get("poses")
    if not isinstance(raw_poses, dict):
        raise ValueError("poses must be a JSON object")
    missing = [pose_id for pose_id in REQUIRED_POSE_IDS if pose_id not in raw_poses]
    if missing:
        raise ValueError(f"required poses are missing: {', '.join(missing)}")

    validated_poses: Dict[str, Tuple[float, ...]] = {}
    for pose_id, pose in raw_poses.items():
        if not isinstance(pose_id, str) or not isinstance(pose, dict):
            raise ValueError("each pose must be a named JSON object")
        raw_qpos = pose.get("qpos")
        if not isinstance(raw_qpos, list) or len(raw_qpos) != 20:
            raise ValueError(f"pose {pose_id!r} qpos must contain exactly 20 entries")
        qpos = []
        for index, value in enumerate(raw_qpos):
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"pose {pose_id!r} qpos[{index}] must be numeric")
            position = float(value)
            if not math.isfinite(position):
                raise ValueError(f"pose {pose_id!r} qpos[{index}] must be finite")
            lower, upper = model_limits[index]
            if position < lower or position > upper:
                raise ValueError(
                    f"pose {pose_id!r} qpos[{index}] ({canonical_order[index]})={position} "
                    f"is outside JSON model limits [{lower}, {upper}]"
                )
            qpos.append(position)
        validated_poses[pose_id] = tuple(qpos)

    return PoseLibrary(
        path=source,
        model=hand["model"],
        handedness=hand["handedness"],
        dof=hand["dof"],
        unit=hand["unit"],
        canonical_joint_order=canonical_order,
        poses=MappingProxyType(validated_poses),
    )


def _pose_library_summary(library: PoseLibrary) -> str:
    separator = chr(10)
    return separator.join(
        (
            "Pose library: VALID",
            f"Hand: {library.model} {library.handedness}",
            f"DOF: {library.dof}",
            "Canonical order: URDF",
            f"Loaded poses: {len(library.poses)}",
            "",
            *library.poses,
        )
    )


class MotionState(str, Enum):
    """The complete, intentionally small live-control state machine."""

    DISABLED = "DISABLED"
    IDLE = "IDLE"
    MOVING_TO_POSE = "MOVING_TO_POSE"


@dataclass(frozen=True)
class ControlStatus:
    """Thread-safe view for control clients; it never contains an SDK handle."""

    state: MotionState
    connected: bool
    detail: str
    updated_at: float


@dataclass(frozen=True)
class JointAudit:
    """Runtime-confirmed SDK index/label and firmware soft limit for one joint."""

    sdk_index: int
    sdk_label: str
    ros_joint_name: str
    lower: float
    upper: float


@dataclass(frozen=True)
class CupTargets:
    """All targets are 20-element, finger-major radians vectors."""

    # Enable-time measured reference used by the already-validated cup recipe.
    open_position: Tuple[float, ...]
    cup_grasp: Tuple[float, ...]


@dataclass(frozen=True)
class QueuedAction:
    """A validated GUI command queued for the one SDK-owning worker."""

    action: str
    pose_id: str = ""


@dataclass
class ActiveTrajectory:
    """A worker-owned, non-blocking smooth trajectory."""

    name: str
    motion_state: MotionState
    target: Tuple[float, ...]
    final_state: MotionState
    final_detail: str
    start: Tuple[float, ...]
    command: list[float]
    started_at: float
    duration_s: float
    reached_target_at: Optional[float] = None


SDK_FINGER_NAMES: Tuple[str, ...] = ("thumb", "index", "middle", "ring", "pinky")

# J1/J3 make the main cylindrical envelope.  J4 is intentionally separate so
# the fingertips can curl farther around a cup without inventing a joint axis.
FINGER_BASE_CURL_INDICES: Tuple[int, ...] = tuple(
    finger * 4 + joint
    for finger in range(1, 5)
    for joint in (0, 2)
)
FINGERTIP_CURL_INDICES: Tuple[int, ...] = tuple(
    finger * 4 + 3 for finger in range(1, 5)
)
THUMB_OPPOSITION_INDICES: Tuple[int, int] = (0, 1)

# Source: Wuji's official wujihandpy example/joint/7.glove_donning.py.  It
# identifies these *measured* per-side positions as "thumb adducted across the
# palm".  We move only a small interpolation fraction toward that documented
# pose; no thumb direction is inferred from an unlabeled URDF axis.
OFFICIAL_THUMB_ADDUCTION: Dict[str, Tuple[float, float]] = {
    "left": (1.1355, 0.7829),
    "right": (1.3029, 0.7528),
}
CUP_GRASP_POSE_ID = "cup_grasp"


class CupMotionProfile:
    """Pure, testable construction and clamping of constrained cup targets.

    ``open_position`` is captured from the real, fresh joint-state stream when
    the user presses Enable.  ``cup_grasp`` is exactly the former final
    Light Close 3/3 target, constructed once from that reference and the same
    runtime firmware-limit safety margin.  Intermediate click targets are no
    longer exposed or constructed.
    """

    JOINT_COUNT = 20
    LIMIT_MARGIN_RAD = 0.08
    REFERENCE_CAPTURE_TOLERANCE_RAD = 0.005
    # Preserve the exact arithmetic of the previously hardware-validated final
    # Light Close 3/3 target. These are not newly designed joint values.
    CUP_GRASP_BASE_CURL_RAD = 0.35
    CUP_GRASP_FINAL_CURL_INCREMENT_RAD = 3 * 0.06
    CUP_GRASP_BASE_FINGERTIP_CURL_RAD = 0.45
    CUP_GRASP_FINAL_FINGERTIP_INCREMENT_RAD = 3 * 0.08
    CUP_GRASP_THUMB_BLEND = 0.40 + (3 * 0.06)
    MAX_JOINT_STEP_RAD = 0.003
    UPDATE_RATE_HZ = 100.0
    LOW_PASS_CUTOFF_HZ = 5.0
    JSON_POSE_MIN_DURATION_S = 5.0
    # Named-pose planned peak speed, increased by another 20% from the
    # operator-tested 0.12 rad/s setting. Independent step, firmware-limit,
    # and feedback safety bounds remain unchanged.
    JSON_POSE_PEAK_SPEED_RAD_S = 0.144
    TARGET_SETTLE_S = 0.50
    JOINT_STATE_TIMEOUT_S = 0.50
    FRESH_STATE_MAX_AGE_S = 0.25

    @classmethod
    def _require_vector(cls, values: Sequence[float], label: str) -> Tuple[float, ...]:
        vector = tuple(float(value) for value in values)
        if len(vector) != cls.JOINT_COUNT or not all(math.isfinite(value) for value in vector):
            raise ValueError(f"{label} must contain {cls.JOINT_COUNT} finite values")
        return vector

    @classmethod
    def safe_bounds(
        cls, lower: Sequence[float], upper: Sequence[float]
    ) -> Tuple[Tuple[float, ...], Tuple[float, ...]]:
        lower_values = cls._require_vector(lower, "firmware lower limits")
        upper_values = cls._require_vector(upper, "firmware upper limits")
        safe_lower: list[float] = []
        safe_upper: list[float] = []
        for index, (lo, hi) in enumerate(zip(lower_values, upper_values)):
            if hi <= lo:
                raise ValueError(f"firmware limit order is invalid at SDK index {index}: {lo}..{hi}")
            margin_lo = lo + cls.LIMIT_MARGIN_RAD
            margin_hi = hi - cls.LIMIT_MARGIN_RAD
            if margin_hi <= margin_lo:
                raise ValueError(
                    f"firmware limit range at SDK index {index} is too small for the "
                    f"{cls.LIMIT_MARGIN_RAD:.3f} rad safety margin"
                )
            safe_lower.append(margin_lo)
            safe_upper.append(margin_hi)
        return tuple(safe_lower), tuple(safe_upper)

    @classmethod
    def safe_reference_position(
        cls, open_position: Sequence[float], safe_lower: Sequence[float], safe_upper: Sequence[float]
    ) -> Tuple[float, ...]:
        position = cls._require_vector(open_position, "measured reference position")
        lower = cls._require_vector(safe_lower, "safe lower limits")
        upper = cls._require_vector(safe_upper, "safe upper limits")
        tolerance = cls.REFERENCE_CAPTURE_TOLERANCE_RAD
        outside = [
            index
            for index, value in enumerate(position)
            if value < lower[index] - tolerance or value > upper[index] + tolerance
        ]
        if outside:
            raise ValueError(
                "measured reference position is outside the firmware-limit safety margin plus "
                f"the {tolerance:.3f} rad measured-reference tolerance at SDK indices "
                f"{outside}; move is refused"
            )
        # Sensor/tracking noise may put the measured state a few milliradians
        # beyond a command boundary.  References within that narrow tolerance
        # are accepted, but every generated command remains clamped to the
        # original 0.08 rad firmware-limit safety margin.
        return tuple(
            min(max(value, lower[index]), upper[index])
            for index, value in enumerate(position)
        )

    @classmethod
    def clamp(
        cls, values: Sequence[float], safe_lower: Sequence[float], safe_upper: Sequence[float]
    ) -> Tuple[float, ...]:
        vector = cls._require_vector(values, "command")
        lower = cls._require_vector(safe_lower, "safe lower limits")
        upper = cls._require_vector(safe_upper, "safe upper limits")
        return tuple(
            min(max(value, lower[index]), upper[index])
            for index, value in enumerate(vector)
        )

    @classmethod
    def require_within_safe_bounds(
        cls,
        values: Sequence[float],
        safe_lower: Sequence[float],
        safe_upper: Sequence[float],
        *,
        label: str,
        joint_names: Sequence[str],
    ) -> Tuple[float, ...]:
        """Validate a target against firmware-safe bounds without clipping it."""

        vector = cls._require_vector(values, label)
        lower = cls._require_vector(safe_lower, "safe lower limits")
        upper = cls._require_vector(safe_upper, "safe upper limits")
        names = tuple(str(name) for name in joint_names)
        if len(names) != cls.JOINT_COUNT:
            raise ValueError(
                f"runtime joint order must contain {cls.JOINT_COUNT} names, got {len(names)}"
            )
        violations = [
            (index, names[index], value, lower[index], upper[index])
            for index, value in enumerate(vector)
            if value < lower[index] or value > upper[index]
        ]
        if violations:
            details = "; ".join(
                f"qpos[{index}] {name}={value:.6f} outside [{lo:.6f}, {hi:.6f}]"
                for index, name, value, lo, hi in violations
            )
            raise ValueError(
                f"{label} rejected by runtime firmware limits with "
                f"{cls.LIMIT_MARGIN_RAD:.2f} rad margin: {details}"
            )
        return vector

    @classmethod
    def library_pose_duration(
        cls, start: Sequence[float], target: Sequence[float]
    ) -> float:
        """Choose a conservative smoothstep duration from the largest move."""

        start_values = cls._require_vector(start, "pose start")
        target_values = cls._require_vector(target, "pose target")
        max_delta = max(
            abs(target_value - start_value)
            for start_value, target_value in zip(start_values, target_values)
        )
        # Cubic smoothstep has a peak derivative of 1.5. This keeps the
        # planned peak at or below 0.144 rad/s, while the existing independent
        # 0.003 rad/control-cycle bound remains active as a second limit.
        return max(
            cls.JSON_POSE_MIN_DURATION_S,
            1.5 * max_delta / cls.JSON_POSE_PEAK_SPEED_RAD_S,
        )

    @classmethod
    def targets(
        cls,
        open_position: Sequence[float],
        side: str,
        safe_lower: Sequence[float],
        safe_upper: Sequence[float],
    ) -> CupTargets:
        open_vector = cls.safe_reference_position(open_position, safe_lower, safe_upper)
        side_key = str(side).strip().lower()
        if side_key not in OFFICIAL_THUMB_ADDUCTION:
            raise ValueError(f"unsupported hand side {side!r}; thumb target is intentionally not guessed")

        cup_grasp = list(open_vector)
        for index in FINGER_BASE_CURL_INDICES:
            # Positive curl direction is explicitly used in the official F2-F5
            # realtime example.  F2-F5 J2 remains unchanged.
            cup_grasp[index] = (
                open_vector[index]
                + cls.CUP_GRASP_BASE_CURL_RAD
                + cls.CUP_GRASP_FINAL_CURL_INCREMENT_RAD
            )
        for index in FINGERTIP_CURL_INDICES:
            cup_grasp[index] = (
                open_vector[index]
                + cls.CUP_GRASP_BASE_FINGERTIP_CURL_RAD
                + cls.CUP_GRASP_FINAL_FINGERTIP_INCREMENT_RAD
            )

        for index, adducted_reference in zip(
            THUMB_OPPOSITION_INDICES, OFFICIAL_THUMB_ADDUCTION[side_key]
        ):
            displacement = adducted_reference - open_vector[index]
            cup_grasp[index] = (
                open_vector[index] + cls.CUP_GRASP_THUMB_BLEND * displacement
            )

        return CupTargets(
            open_position=open_vector,
            cup_grasp=cls.clamp(cup_grasp, safe_lower, safe_upper),
        )

class LiveHandControlBridge(Node):
    """One worker serializes the one SDK handle, readers, and any motor write."""

    REMOTE_UI_HEARTBEAT_TIMEOUT_S = 1.0
    _ACTION_BY_REQUEST = {
        HandControlCommand.Request.ENABLE: "enable",
        HandControlCommand.Request.DISABLE: "disable",
        HandControlCommand.Request.MOVE_POSE: "move_pose",
    }
    _STATUS_BY_STATE = {
        MotionState.DISABLED: HandControlStatus.DISABLED,
        MotionState.IDLE: HandControlStatus.IDLE,
        MotionState.MOVING_TO_POSE: HandControlStatus.MOVING_TO_POSE,
    }

    def __init__(self) -> None:
        super().__init__("live_hand_control")
        self.declare_parameter("hand_name", "hand_0")
        self.declare_parameter(
            "hand_serial_number",
            "336636733434",
            descriptor=ParameterDescriptor(dynamic_typing=True),
        )
        self.declare_parameter("tactile_serial_number", "WT1JC01260720019")
        self.declare_parameter("sdk_site_packages", DEFAULT_SDK_SITE_PACKAGES)
        self.declare_parameter("expected_hand_side", "")
        self.declare_parameter("reconnect_interval_s", 2.0)
        self.declare_parameter("poll_sleep_ms", 2.0)
        self.declare_parameter("visual_joint_publish_hz", 50.0)
        self.declare_parameter("visual_joint_low_pass_hz", 5.0)
        self.declare_parameter(
            "pose_library_path", str(_default_pose_library_path())
        )

        self.hand_name = str(self.get_parameter("hand_name").value).strip("/")
        self.hand_serial = str(self.get_parameter("hand_serial_number").value)
        self.tactile_serial = str(self.get_parameter("tactile_serial_number").value)
        self._sdk_site_packages = str(self.get_parameter("sdk_site_packages").value)
        self._expected_hand_side = str(self.get_parameter("expected_hand_side").value).strip().lower()
        if self._expected_hand_side not in ("", "left", "right"):
            raise ValueError("expected_hand_side must be '', 'left', or 'right'")
        self._pose_library_path = Path(
            str(self.get_parameter("pose_library_path").value)
        ).expanduser()
        self._pose_library: Optional[PoseLibrary] = None
        self._pose_library_error = ""
        pose_hand_side = self._expected_hand_side or "left"
        try:
            self._pose_library = load_pose_library(
                self._pose_library_path, _ros_joint_order_for_side(pose_hand_side)
            )
        except ValueError as exc:
            self._pose_library_error = str(exc)
            self.get_logger().error(
                "Pose library: INVALID; "
                f"reason: {self._pose_library_error}. "
                "The existing Cup Grasp target, tactile, and JointState paths remain "
                "available; JSON pose execution is unavailable."
            )
        else:
            self.get_logger().info(_pose_library_summary(self._pose_library))
        self._reconnect_interval_s = max(
            0.5, float(self.get_parameter("reconnect_interval_s").value)
        )
        self._poll_sleep_s = max(
            0.001, float(self.get_parameter("poll_sleep_ms").value) / 1000.0
        )
        self._visual_joint_publish_hz = min(
            120.0,
            max(10.0, float(self.get_parameter("visual_joint_publish_hz").value)),
        )
        self._visual_joint_low_pass_hz = min(
            0.45 * self._visual_joint_publish_hz,
            max(0.5, float(self.get_parameter("visual_joint_low_pass_hz").value)),
        )
        self._remote_ui_lock = threading.Lock()
        self._last_remote_ui_heartbeat = 0.0
        self._remote_ui_watchdog_enabled = False

        self.publisher = self.create_publisher(
            TactilePressureFrame, f"/{self.hand_name}/tactile/pressure", qos_profile_sensor_data
        )
        self._joint_state_publisher = self.create_publisher(
            JointState, f"/{self.hand_name}/joint_states", qos_profile_sensor_data
        )
        self._visual_joint_state_publisher = self.create_publisher(
            JointState, f"/{self.hand_name}/joint_states_visual", qos_profile_sensor_data
        )
        self._control_status_publisher = self.create_publisher(
            HandControlStatus, f"/{self.hand_name}/hand_control/status", 10
        )
        self._control_command_service = self.create_service(
            HandControlCommand,
            f"/{self.hand_name}/hand_control/command",
            self._handle_control_command,
        )
        self._remote_ui_heartbeat_subscription = self.create_subscription(
            Empty,
            f"/{self.hand_name}/hand_control/ui_heartbeat",
            self._on_remote_ui_heartbeat,
            10,
        )

        self._profile = CupMotionProfile()
        self._latest_lock = threading.Lock()
        self._latest: Optional[TactileSnapshot] = None
        self._status_lock = threading.Lock()
        self._status = ControlStatus(
            state=MotionState.DISABLED,
            connected=False,
            detail="Waiting for the single SDK connection; motors are disabled by default.",
            updated_at=time.monotonic(),
        )

        self._frames: queue.Queue[RawFrame] = queue.Queue(maxsize=8)
        self._joint_frames: queue.Queue[RawJointState] = queue.Queue(maxsize=8)
        self._latest_visual_joint_frame: Optional[RawJointState] = None
        self._visual_joint_position: Optional[Tuple[float, ...]] = None
        self._last_visual_joint_publish_at = 0.0
        self._events: queue.Queue[Tuple[str, str]] = queue.Queue()
        self._actions: queue.Queue[QueuedAction] = queue.Queue(maxsize=16)
        self._disable_requested = threading.Event()
        self._stop_worker = threading.Event()
        self._worker: Optional[threading.Thread] = None
        self._shutdown_lock = threading.Lock()
        self._shutdown_started = False

        # These fields are owned exclusively by _sdk_worker. ROS executor
        # callbacks never receive the SDK hand, subscription, or controller.
        self._sdk = None
        self._manager = None
        self._hand = None
        self._tactile_subscription = None
        self._joint_subscription = None
        self._controller_context = None
        self._controller = None
        self._hand_side = ""
        self._joint_names: Tuple[str, ...] = ()
        self._joint_audit: Tuple[JointAudit, ...] = ()
        self._safe_lower: Tuple[float, ...] = ()
        self._safe_upper: Tuple[float, ...] = ()
        self._targets: Optional[CupTargets] = None
        self._last_joint_position: Optional[Tuple[float, ...]] = None
        self._last_joint_received_at = 0.0
        self._active_trajectory: Optional[ActiveTrajectory] = None
        self._holding_target: Optional[Tuple[float, ...]] = None
        self._unknown_layout_counts: Dict[int, int] = {}
        self._invalid_joint_counts: Dict[int, int] = {}
        self._published_frames = 0
        self._published_joint_frames = 0
        self._published_visual_joint_frames = 0
        self._last_publish_log = 0.0

        self._publish_timer = self.create_timer(0.005, self._publish_queued_frames)
        self._control_status_timer = self.create_timer(0.1, self._publish_control_status)
        self._worker = threading.Thread(
            target=self._sdk_worker,
            name="wuji-live-hand-control-sdk",
            daemon=True,
        )
        self._worker.start()
        self.get_logger().info(
            "Live hand-control backend created. It will use one SDK connection and remains DISABLED "
            "until the user presses Enable in an active safety UI."
        )

    @staticmethod
    def _enqueue_latest(target: queue.Queue, item) -> None:
        """Prefer the freshest data if ROS publishing temporarily falls behind."""
        while True:
            try:
                target.put_nowait(item)
                return
            except queue.Full:
                try:
                    target.get_nowait()
                except queue.Empty:
                    return

    @staticmethod
    def _layout_for_count(count: int) -> Optional[Tuple[int, int]]:
        # Exact same accepted layouts and row-major mapping as tactile_bridge.
        return KNOWN_LAYOUTS.get(count)

    def latest(self) -> Optional[TactileSnapshot]:
        """Return only immutable tactile data to the inherited heatmap GUI."""
        with self._latest_lock:
            return self._latest

    def control_status(self) -> ControlStatus:
        with self._status_lock:
            return self._status

    def joint_audit(self) -> Tuple[JointAudit, ...]:
        """Read-only access for diagnostics/reporting, never used for commands."""
        return self._joint_audit

    def _runtime_validated_pose_target(self, pose_id: str) -> Tuple[float, ...]:
        """Resolve one named target and validate it against runtime firmware bounds."""

        if len(self._safe_lower) != 20 or len(self._safe_upper) != 20:
            raise ValueError("runtime firmware safe limits are unavailable")
        if len(self._joint_names) != 20:
            raise ValueError("runtime SDK-to-URDF joint order is unavailable")

        if pose_id == CUP_GRASP_POSE_ID:
            if self._targets is None:
                raise ValueError(
                    "Cup Grasp target is unavailable until Enable captures the reference pose"
                )
            return self._profile.require_within_safe_bounds(
                self._targets.cup_grasp,
                self._safe_lower,
                self._safe_upper,
                label="existing final Cup Grasp target",
                joint_names=self._joint_names,
            )

        library = self._pose_library
        if library is None:
            reason = self._pose_library_error or "pose library was not loaded"
            raise ValueError(f"Pose Library INVALID: {reason}")
        if pose_id not in library.poses:
            raise ValueError(f"selected pose {pose_id!r} does not exist in the loaded library")
        if library.canonical_joint_order != self._joint_names:
            raise ValueError(
                "runtime SDK-to-URDF order no longer matches the pose library canonical order"
            )
        return self._profile.require_within_safe_bounds(
            library.poses[pose_id],
            self._safe_lower,
            self._safe_upper,
            label=f"pose {pose_id!r}",
            joint_names=library.canonical_joint_order,
        )

    def _event(self, level: str, detail: str) -> None:
        self._events.put((level, detail))

    def _set_status(
        self,
        state: MotionState,
        detail: str,
        *,
        connected: Optional[bool] = None,
    ) -> None:
        with self._status_lock:
            previous = self._status
            self._status = ControlStatus(
                state=state,
                connected=previous.connected if connected is None else connected,
                detail=detail,
                updated_at=time.monotonic(),
            )

    def _set_connected_status(self, connected: bool, detail: str) -> None:
        status = self.control_status()
        self._set_status(
            MotionState.DISABLED if not connected else status.state,
            detail,
            connected=connected,
        )

    def _on_remote_ui_heartbeat(self, _message: Empty) -> None:
        with self._remote_ui_lock:
            self._last_remote_ui_heartbeat = time.monotonic()

    def _remote_ui_is_fresh(self) -> bool:
        with self._remote_ui_lock:
            age = time.monotonic() - self._last_remote_ui_heartbeat
        return age <= self.REMOTE_UI_HEARTBEAT_TIMEOUT_S

    def _set_remote_ui_watchdog(self, enabled: bool) -> None:
        with self._remote_ui_lock:
            self._remote_ui_watchdog_enabled = enabled

    def _remote_ui_watchdog_is_stale(self, now: float) -> bool:
        with self._remote_ui_lock:
            return self._remote_ui_watchdog_enabled and (
                now - self._last_remote_ui_heartbeat > self.REMOTE_UI_HEARTBEAT_TIMEOUT_S
            )

    def _handle_control_command(self, request, response):
        """Accept only named actions from a live, heartbeating remote panel."""
        action = self._ACTION_BY_REQUEST.get(int(request.action))
        pose_id = str(request.pose_id)
        if action is None:
            response.accepted = False
            response.detail = f"Unknown constrained hand-control action {int(request.action)}."
            return response

        if action != "disable":
            if not self._remote_ui_is_fresh():
                response.accepted = False
                response.detail = (
                    "Remote UI heartbeat is absent or stale; motion request was not queued."
                )
                return response
            self._set_remote_ui_watchdog(True)

        response.accepted, response.detail = self.request_action(action, pose_id=pose_id)
        if action == "disable":
            self._set_remote_ui_watchdog(False)
        return response

    def _publish_control_status(self) -> None:
        status = self.control_status()
        message = HandControlStatus()
        message.header.stamp = self.get_clock().now().to_msg()
        message.state = int(self._STATUS_BY_STATE[status.state])
        message.connected = status.connected
        message.detail = status.detail
        self._control_status_publisher.publish(message)

    def request_action(self, action: str, *, pose_id: str = "") -> Tuple[bool, str]:
        """Queue a named GUI action; this function never calls the SDK."""
        action_key = str(action).strip().lower()
        pose_key = str(pose_id)
        if action_key not in {"enable", "disable", "move_pose"}:
            return False, f"Unsupported action {action!r}."
        if action_key != "move_pose" and pose_key:
            return False, f"Action {action_key!r} must not contain a pose_id."

        status = self.control_status()
        if action_key == "disable":
            # This event makes Disable take priority over any queued motion.
            self._disable_requested.set()
            try:
                self._actions.put_nowait(QueuedAction(action_key))
            except queue.Full:
                pass
            return True, "Disable requested; the SDK worker will stop the trajectory and disable the hand."

        if not status.connected:
            return False, "SDK connection is not ready; no action was sent."
        if action_key == "move_pose":
            if status.state != MotionState.IDLE:
                return False, (
                    "Move to Pose is only valid from Enabled + IDLE "
                    f"(currently {status.state.value})."
                )
            if not pose_key:
                return False, "Move to Pose requires an explicit pose_id."
            feedback_age = time.monotonic() - self._last_joint_received_at
            if (
                self._last_joint_position is None
                or feedback_age > self._profile.FRESH_STATE_MAX_AGE_S
            ):
                return False, (
                    "Move to Pose refused: actual joint feedback is absent or stale; "
                    "no command was queued."
                )
            try:
                self._runtime_validated_pose_target(pose_key)
            except ValueError as exc:
                return False, f"Move to Pose refused: {exc}"
        if action_key == "enable" and status.state != MotionState.DISABLED:
            return False, f"Enable is only valid from DISABLED (currently {status.state.value})."
        try:
            self._actions.put_nowait(
                QueuedAction(action_key, pose_key if action_key == "move_pose" else "")
            )
        except queue.Full:
            return False, "Action queue is full; wait for the current state to update."
        if action_key == "move_pose":
            return True, (
                f"Pose {pose_key!r} passed runtime checks and was queued for the single SDK worker."
            )
        return True, f"{action_key.replace('_', ' ').title()} queued."

    def _sdk_worker(self) -> None:
        """The only thread allowed to touch the WujiHand SDK handle."""
        next_connect_attempt = 0.0
        next_control_tick = time.monotonic()
        try:
            while not self._stop_worker.is_set():
                if not rclpy.ok():
                    self._event("warn", "[SAFETY] ROS shutdown detected; disabling the hand.")
                    break
                now = time.monotonic()
                if self._hand is None:
                    if now >= next_connect_attempt:
                        self._connect_worker()
                        next_connect_attempt = now + self._reconnect_interval_s
                    self._stop_worker.wait(0.05)
                    continue

                try:
                    if self._disable_requested.is_set():
                        self._disable_requested.clear()
                        self._discard_actions_worker()
                        self._safe_disable_worker("Disable requested by user")
                        continue

                    self._drain_actions_worker()
                    self._poll_sdk_streams_worker()
                    now = time.monotonic()
                    status = self.control_status()
                    if (
                        status.state != MotionState.DISABLED
                        and now - self._last_joint_received_at > self._profile.JOINT_STATE_TIMEOUT_S
                    ):
                        self._safe_disable_worker(
                            "joint state stream stale for more than "
                            f"{self._profile.JOINT_STATE_TIMEOUT_S:.2f} s"
                        )
                        continue

                    if (
                        status.state != MotionState.DISABLED
                        and self._remote_ui_watchdog_is_stale(now)
                    ):
                        self._safe_disable_worker(
                            "RViz control-panel heartbeat stale for more than "
                            f"{self.REMOTE_UI_HEARTBEAT_TIMEOUT_S:.2f} s"
                        )
                        continue

                    if now >= next_control_tick:
                        self._advance_motion_worker(now)
                        tick = 1.0 / self._profile.UPDATE_RATE_HZ
                        while next_control_tick <= now:
                            next_control_tick += tick
                    self._stop_worker.wait(self._poll_sleep_s)
                except Exception as exc:  # SDK exceptions, disconnects, or worker faults.
                    self._event("error", f"[SAFETY] SDK/control worker exception: {exc}")
                    self._safe_disable_worker("SDK exception")
                    self._disconnect_worker()
        finally:
            # Every exit path through the only SDK-owning worker attempts disable.
            self._safe_disable_worker("SDK worker exiting")
            self._disconnect_worker()

    def _connect_worker(self) -> None:
        """Open exactly one hand connection; never enable or command it here."""
        hand = None
        tactile_subscription = None
        joint_subscription = None
        controller_context = None
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
                self._event(
                    "warn",
                    f"Waiting for Wuji Hand serial {self.hand_serial}; scan found "
                    f"{[(device.sn, str(device.device_type)) for device in devices]}",
                )
                return

            # This is the sole connect() call in the live executable.  The GUI
            # shares this node in-process and never creates another SDK handle.
            hand = self._manager.connect(
                sn=self.hand_serial,
                device_name=f"{self.hand_name}_live_hand_control",
            )
            if not hand.is_tactile_attached():
                raise RuntimeError("Wuji Hand connected but no paired tactile glove is attached")

            hand_side = str(hand.handedness_name()).strip().lower()
            if hand_side not in ("left", "right"):
                raise RuntimeError(f"unsupported SDK handedness {hand_side!r}")
            if self._expected_hand_side and hand_side != self._expected_hand_side:
                raise RuntimeError(
                    f"handedness mismatch: launch expects {self._expected_hand_side}, SDK reports {hand_side}"
                )

            info = hand.tactile.device_info()
            actual_tactile_serial = str(info.serial)
            if self.tactile_serial and actual_tactile_serial != self.tactile_serial:
                raise RuntimeError(
                    f"paired tactile serial mismatch: expected {self.tactile_serial}, got {actual_tactile_serial}"
                )

            upper, lower = hand.get_soft_limits()
            safe_lower, safe_upper = self._profile.safe_bounds(lower, upper)
            audit, joint_names = self._audit_joint_mapping_worker(hand, hand_side, lower, upper)

            # The SDK reports that joint_states().subscribe() implicitly opens
            # a 1000 Hz realtime controller.  Create this session first so the
            # one shared controller has our official 5 Hz LowPass configuration.
            # Entering the context only prepares the transport/readback cache:
            # it does not enable motors and this code never sets a target here.
            controller_context = hand.realtime_controller(
                self._sdk.LowPass(cutoff_hz=self._profile.LOW_PASS_CUTOFF_HZ)
            )
            controller = controller_context.__enter__()
            joint_subscription = hand.joint_states().subscribe()
            tactile_subscription = hand.tactile.subscribe_pressure_frame()

            self._hand = hand
            self._tactile_subscription = tactile_subscription
            self._joint_subscription = joint_subscription
            self._controller_context = controller_context
            self._controller = controller
            self._hand_side = hand_side
            self._joint_audit = audit
            self._joint_names = joint_names
            self._safe_lower = safe_lower
            self._safe_upper = safe_upper
            self._targets = None
            self._last_joint_position = None
            self._last_joint_received_at = 0.0
            self._latest_visual_joint_frame = None
            self._visual_joint_position = None
            self._last_visual_joint_publish_at = 0.0
            self._set_status(
                MotionState.DISABLED,
                "SDK connected; GUI remains DISABLED and has sent no target or motor-enable command. "
                "Verify a safe open pose, then press Enable.",
                connected=True,
            )
            self._event(
                "info",
                f"Connected one SDK WujiHand handle: hand={hand.serial_number} ({hand_side}), "
                f"tactile={actual_tactile_serial}; no motor command has been sent.",
            )
            audit_text = "; ".join(
                f"{entry.ros_joint_name}: sdk[{entry.sdk_index}]={entry.sdk_label}, "
                f"firmware [{entry.lower:.4f}, {entry.upper:.4f}] rad"
                for entry in audit
            )
            self._event("info", f"[SAFETY] Runtime joint/limit audit: {audit_text}")
        except Exception as exc:
            # The worker is the only owner, so even failed connection cleanup
            # does not race a second SDK client.
            for subscription in (joint_subscription, tactile_subscription):
                if subscription is not None:
                    try:
                        subscription.close()
                    except Exception:
                        pass
            if controller_context is not None:
                try:
                    controller_context.__exit__(None, None, None)
                except Exception:
                    pass
            if hand is not None:
                try:
                    hand.disable()
                except Exception:
                    pass
                try:
                    hand.disconnect()
                except Exception:
                    pass
            self._set_status(
                MotionState.DISABLED,
                f"Waiting for a valid SDK connection: {exc}",
                connected=False,
            )
            self._event("warn", f"Live hand-control connection attempt failed safely: {exc}")

    def _audit_joint_mapping_worker(
        self, hand, hand_side: str, lower: Sequence[float], upper: Sequence[float]
    ) -> Tuple[Tuple[JointAudit, ...], Tuple[str, ...]]:
        """Reject any unverified index/label layout before it can be commanded."""
        expected_labels = tuple(
            f"{finger}_joint{joint}"
            for finger in SDK_FINGER_NAMES
            for joint in range(1, 5)
        )
        handles = sorted(tuple(hand.joints()), key=lambda handle: int(handle.index))
        actual_pairs = tuple((int(handle.index), str(handle.label)) for handle in handles)
        expected_pairs = tuple(enumerate(expected_labels))
        if actual_pairs != expected_pairs:
            raise RuntimeError(
                "SDK joint index/label audit failed; refusing motion instead of guessing: "
                f"got {actual_pairs}, expected {expected_pairs}"
            )

        fingers = tuple(hand.fingers())
        if len(fingers) != len(SDK_FINGER_NAMES):
            raise RuntimeError(f"SDK finger audit expected 5 entries, got {len(fingers)}")
        for finger_index, (finger, expected_name) in enumerate(zip(fingers, SDK_FINGER_NAMES)):
            labels = tuple(str(joint.label) for joint in finger.joints())
            expected = expected_labels[finger_index * 4 : finger_index * 4 + 4]
            if str(finger.name) != expected_name or labels != expected:
                raise RuntimeError(
                    "SDK finger/joint audit failed; refusing motion instead of guessing: "
                    f"finger={finger.name!r}, labels={labels}, expected={expected_name!r}/{expected}"
                )

        lower_values = tuple(float(value) for value in lower)
        upper_values = tuple(float(value) for value in upper)
        if len(lower_values) != 20 or len(upper_values) != 20:
            raise RuntimeError("firmware soft limit audit did not return 20 lower and 20 upper values")
        names = _ros_joint_order_for_side(hand_side)
        return (
            tuple(
                JointAudit(
                    sdk_index=index,
                    sdk_label=expected_labels[index],
                    ros_joint_name=names[index],
                    lower=lower_values[index],
                    upper=upper_values[index],
                )
                for index in range(20)
            ),
            names,
        )

    def _poll_sdk_streams_worker(self) -> None:
        """Drain bounded batches so high-rate joint state cannot backlog the SDK."""
        if self._joint_subscription is not None:
            # The SDK joint-state producer can be faster than the 2 ms worker
            # poll period.  Draining a bounded batch keeps only the latest
            # state without starving the control-state and tactile paths.
            for _ in range(8):
                joint_state = self._joint_subscription.recv()
                if joint_state is None:
                    break
                self._queue_joint_state_worker(joint_state)

        if self._tactile_subscription is None:
            return
        for _ in range(4):
            frame = self._tactile_subscription.recv()
            if frame is None:
                break
            pressure = tuple(float(value) for value in frame.pressure)
            layout = self._layout_for_count(len(pressure))
            if layout is None:
                count = len(pressure)
                occurrences = self._unknown_layout_counts.get(count, 0) + 1
                self._unknown_layout_counts[count] = occurrences
                if occurrences == 1 or occurrences % 500 == 0:
                    self._event(
                        "error",
                        f"Refusing unknown tactile layout with {count} values; no shape/mapping is guessed.",
                    )
                continue
            rows, cols = layout
            raw_frame = RawFrame(
                handedness=int(frame.handedness),
                sequence=int(frame.sequence),
                timestamp_ms=int(frame.timestamp_ms),
                pressure=pressure,
                rows=rows,
                cols=cols,
            )
            self._enqueue_latest(self._frames, raw_frame)
            with self._latest_lock:
                self._latest = TactileSnapshot(
                    sequence=raw_frame.sequence,
                    timestamp_ms=raw_frame.timestamp_ms,
                    handedness=raw_frame.handedness,
                    rows=raw_frame.rows,
                    cols=raw_frame.cols,
                    pressure=raw_frame.pressure,
                )

    def _queue_joint_state_worker(self, state) -> None:
        position = tuple(float(value) for value in state.position)
        finite_position = len(position) == 20 and all(math.isfinite(value) for value in position)
        if not finite_position:
            count = len(position)
            occurrences = self._invalid_joint_counts.get(count, 0) + 1
            self._invalid_joint_counts[count] = occurrences
            if occurrences == 1 or occurrences % 500 == 0:
                self._event(
                    "error",
                    f"Ignoring invalid SDK joint state: values={count}, finite={finite_position}.",
                )
            return
        velocity = tuple(float(value) for value in getattr(state, "velocity", ()))
        effort = tuple(float(value) for value in getattr(state, "effort", ()))
        if len(velocity) != 20 or not all(math.isfinite(value) for value in velocity):
            velocity = ()
        if len(effort) != 20 or not all(math.isfinite(value) for value in effort):
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
        self._last_joint_position = position
        self._last_joint_received_at = time.monotonic()

    def _drain_actions_worker(self) -> None:
        while True:
            try:
                queued = self._actions.get_nowait()
            except queue.Empty:
                return
            action = queued.action
            if action == "disable":
                self._safe_disable_worker("Disable requested by user")
                self._discard_actions_worker()
                return
            if action == "enable":
                self._enable_worker()
            elif action == "move_pose":
                self._start_named_pose_worker(queued.pose_id)

    def _discard_actions_worker(self) -> None:
        while True:
            try:
                self._actions.get_nowait()
            except queue.Empty:
                return

    def _fresh_position_worker(self) -> Optional[Tuple[float, ...]]:
        position = self._last_joint_position
        age = time.monotonic() - self._last_joint_received_at
        if position is None or age > self._profile.FRESH_STATE_MAX_AGE_S:
            self._event(
                "warn",
                "[SAFETY] Motion request refused: no fresh 20-joint SDK state is available.",
            )
            return None
        return position

    def _enable_worker(self) -> None:
        status = self.control_status()
        if status.state != MotionState.DISABLED or self._hand is None:
            return
        current = self._fresh_position_worker()
        if current is None or not self._safe_lower or not self._safe_upper:
            return
        try:
            targets = self._profile.targets(
                current, self._hand_side, self._safe_lower, self._safe_upper
            )
        except Exception as exc:
            self._event("error", f"[SAFETY] Enable refused during state initialization: {exc}")
            return

        try:
            # This action intentionally does nothing except enable and capture
            # a safe reference position. The controller session was prepared
            # earlier only for SDK readback compatibility; Enable itself does
            # not call set_target_position().
            self._hand.enable()
            self._targets = targets
            self._active_trajectory = None
            self._holding_target = None
            self._set_status(
                MotionState.IDLE,
                "Enabled with the current real position captured as the Cup Grasp "
                "reference. No trajectory has started.",
            )
            self._event("info", "[STATE] Enabled")
        except Exception as exc:
            self._event("error", f"[SAFETY] Enable failed: {exc}")
            self._safe_disable_worker("enable exception")

    def _start_named_pose_worker(self, pose_id: str) -> None:
        """Execute one validated named target through the existing trajectory path."""

        status = self.control_status()
        if status.state != MotionState.IDLE:
            self._event(
                "warn",
                f"[SAFETY] Pose {pose_id!r} refused in worker: state is {status.state.value}.",
            )
            return
        try:
            target = self._runtime_validated_pose_target(pose_id)
        except ValueError as exc:
            self._event("error", f"[SAFETY] Move to Pose refused: {exc}")
            return
        current = self._fresh_position_worker()
        if current is None:
            return
        duration_s = self._profile.library_pose_duration(current, target)
        max_delta = max(
            abs(target_value - start_value)
            for start_value, target_value in zip(current, target)
        )
        is_cup_grasp = pose_id == CUP_GRASP_POSE_ID
        motion_name = "Cup Grasp" if is_cup_grasp else f"JSON pose {pose_id!r}"
        started = self._begin_trajectory_worker(
            name=motion_name,
            motion_state=MotionState.MOVING_TO_POSE,
            target=target,
            final_state=MotionState.IDLE,
            final_detail=(
                "Cup Grasp completed at the existing verified final target through "
                "the safe motion pipeline."
                if is_cup_grasp else
                f"Pose {pose_id!r} completed through the existing safe motion pipeline."
            ),
            duration_s=duration_s,
            start=current,
        )
        if started:
            source = (
                "existing verified final Cup Grasp target"
                if is_cup_grasp else "exact read-only JSON qpos"
            )
            self._event(
                "info",
                f"[POSE] {pose_id!r}: fresh measured start, max_delta={max_delta:.3f} rad, "
                f"planned_duration={duration_s:.2f} s, source={source}, no qpos reorder/clip.",
            )

    def _begin_trajectory_worker(
        self,
        *,
        name: str,
        motion_state: MotionState,
        target: Tuple[float, ...],
        final_state: MotionState,
        final_detail: str,
        duration_s: float,
        start: Tuple[float, ...],
    ) -> bool:
        if self._active_trajectory is not None:
            self._event("error", f"[SAFETY] {name} refused: another trajectory is active.")
            return False
        try:
            controller = self._ensure_controller_worker(start)
            safe_start = self._profile.safe_reference_position(
                start, self._safe_lower, self._safe_upper
            )
            safe_target = self._profile.require_within_safe_bounds(
                target,
                self._safe_lower,
                self._safe_upper,
                label=name,
                joint_names=self._joint_names,
            )
            # The first write is the measured pose clamped to the unchanged
            # command safety margin.  For an accepted reference-boundary error,
            # this correction is at most 0.005 rad and subsequent updates retain
            # the existing 0.003 rad/cycle limit.
            controller.set_target_position(list(safe_start))
            self._active_trajectory = ActiveTrajectory(
                name=name,
                motion_state=motion_state,
                target=safe_target,
                final_state=final_state,
                final_detail=final_detail,
                start=safe_start,
                command=list(safe_start),
                started_at=time.monotonic(),
                duration_s=duration_s,
            )
            self._holding_target = None
            self._set_status(motion_state, f"{name} started.")
            self._event("info", f"[MOTION] {name} started")
            return True
        except Exception as exc:
            self._event("error", f"[SAFETY] Could not start {name}: {exc}")
            self._safe_disable_worker(f"{name} start exception")
            return False

    def _ensure_controller_worker(self, current: Tuple[float, ...]):
        if self._controller is not None:
            return self._controller
        raise RuntimeError(
            "realtime controller is unavailable; reconnect the live session rather than "
            "opening a second controller after joint-state subscription"
        )

    def _advance_motion_worker(self, now: float) -> None:
        controller = self._controller
        if controller is None:
            return
        active = self._active_trajectory
        if active is None:
            if self._holding_target is not None:
                # Completed named poses keep their fixed target at 100 Hz,
                # without any tactile-based adjustment.
                controller.set_target_position(list(self._holding_target))
            return

        fraction = min(max((now - active.started_at) / active.duration_s, 0.0), 1.0)
        smooth_fraction = fraction * fraction * (3.0 - 2.0 * fraction)  # cubic smoothstep
        desired = tuple(
            start + smooth_fraction * (target - start)
            for start, target in zip(active.start, active.target)
        )
        bounded_command = []
        for current, desired_value in zip(active.command, desired):
            step = max(
                -self._profile.MAX_JOINT_STEP_RAD,
                min(self._profile.MAX_JOINT_STEP_RAD, desired_value - current),
            )
            bounded_command.append(current + step)
        active.command = list(self._profile.clamp(bounded_command, self._safe_lower, self._safe_upper))
        controller.set_target_position(active.command)

        command_at_target = all(
            abs(value - target) <= 1e-6 for value, target in zip(active.command, active.target)
        )
        if fraction < 1.0 or not command_at_target:
            return
        if active.reached_target_at is None:
            active.reached_target_at = now
            return
        if now - active.reached_target_at < self._profile.TARGET_SETTLE_S:
            return

        completed = active
        self._active_trajectory = None
        self._holding_target = completed.target
        self._set_status(completed.final_state, completed.final_detail)
        self._event(
            "info",
            f"[MOTION] {completed.name} completed; holding the exact validated named target",
        )

    def _close_controller_worker(self) -> None:
        context = self._controller_context
        self._controller_context = None
        self._controller = None
        if context is not None:
            try:
                context.__exit__(None, None, None)
            except Exception as exc:
                self._event("warn", f"Realtime controller cleanup warning: {exc}")

    def _safe_disable_worker(self, reason: str) -> None:
        """Stop targets first, then issue the required all-motor disable once.

        The already-open controller remains attached until disconnect. Closing
        it here would let joint-state subscription recreate an incompatible
        1000 Hz controller before the next manual Enable.
        """
        self._active_trajectory = None
        self._holding_target = None
        self._targets = None
        self._set_remote_ui_watchdog(False)
        hand = self._hand
        if hand is not None:
            try:
                hand.disable()
            except Exception as exc:
                self._event("error", f"[SAFETY] hand.disable() failed while handling {reason}: {exc}")
        self._set_status(MotionState.DISABLED, f"Disabled: {reason}")
        self._event("info", "[SAFETY] Hand disabled")
        self._event("info", "[STATE] Disabled")

    def _disconnect_worker(self) -> None:
        """Release the one SDK connection after control has been disabled."""
        self._close_controller_worker()
        for subscription_name in ("_joint_subscription", "_tactile_subscription"):
            subscription = getattr(self, subscription_name)
            setattr(self, subscription_name, None)
            if subscription is not None:
                try:
                    subscription.close()
                except Exception:
                    pass
        hand = self._hand
        self._hand = None
        self._joint_audit = ()
        self._joint_names = ()
        self._safe_lower = ()
        self._safe_upper = ()
        self._last_joint_position = None
        self._last_joint_received_at = 0.0
        self._latest_visual_joint_frame = None
        self._visual_joint_position = None
        self._last_visual_joint_publish_at = 0.0
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
        self._set_connected_status(False, "SDK connection released; motors are disabled.")

    def _publish_queued_frames(self) -> None:
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

        latest_joint_frame: Optional[RawJointState] = None
        while True:
            try:
                latest_joint_frame = self._joint_frames.get_nowait()
            except queue.Empty:
                break

        joint_names = self._joint_names
        if latest_joint_frame is not None and len(joint_names) == 20:
            raw_message = JointState()
            raw_message.header.stamp = self.get_clock().now().to_msg()
            raw_message.name = list(joint_names)
            raw_message.position = list(latest_joint_frame.position)
            raw_message.velocity = list(latest_joint_frame.velocity)
            raw_message.effort = list(latest_joint_frame.effort)
            self._joint_state_publisher.publish(raw_message)
            self._published_joint_frames += 1
            self._latest_visual_joint_frame = latest_joint_frame

        now = time.monotonic()
        visual_period_s = 1.0 / self._visual_joint_publish_hz
        pending_visual = self._latest_visual_joint_frame
        if (
            pending_visual is not None
            and len(joint_names) == 20
            and (
                self._last_visual_joint_publish_at <= 0.0
                or now - self._last_visual_joint_publish_at >= visual_period_s
            )
        ):
            self._latest_visual_joint_frame = None
            previous = self._visual_joint_position
            if previous is None or len(previous) != len(pending_visual.position):
                filtered_position = tuple(pending_visual.position)
            else:
                elapsed = max(now - self._last_visual_joint_publish_at, 1.0e-6)
                alpha = 1.0 - math.exp(
                    -2.0 * math.pi * self._visual_joint_low_pass_hz * elapsed
                )
                filtered_position = tuple(
                    old + alpha * (new - old)
                    for old, new in zip(previous, pending_visual.position)
                )
            self._visual_joint_position = filtered_position
            self._last_visual_joint_publish_at = now
            visual_message = JointState()
            visual_message.header.stamp = self.get_clock().now().to_msg()
            visual_message.name = list(joint_names)
            visual_message.position = list(filtered_position)
            visual_message.velocity = list(pending_visual.velocity)
            visual_message.effort = list(pending_visual.effort)
            self._visual_joint_state_publisher.publish(visual_message)
            self._published_visual_joint_frames += 1

        while True:
            try:
                frame = self._frames.get_nowait()
            except queue.Empty:
                break
            message = TactilePressureFrame()
            message.header.stamp = self.get_clock().now().to_msg()
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
                        f"Publishing live tactile {frame.rows}x{frame.cols}; seq={frame.sequence}, "
                        f"max={max(finite):.4f}, mean={sum(finite) / len(finite):.4f}, "
                        f"frames={self._published_frames}"
                    )
                self._last_publish_log = now

    def shutdown(self, timeout_s: float = 3.0) -> None:
        """Request Disable and let only the SDK worker perform the hardware call."""
        with self._shutdown_lock:
            if self._shutdown_started:
                return
            self._shutdown_started = True
        self._disable_requested.set()
        self._stop_worker.set()
        worker = self._worker
        if worker is not None and worker.is_alive() and worker is not threading.current_thread():
            deadline = time.monotonic() + max(0.0, timeout_s)
            interrupted = False
            while worker.is_alive():
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                try:
                    worker.join(timeout=min(0.1, remaining))
                except KeyboardInterrupt:
                    # ros2 run can deliver one interrupt to its wrapper and a second
                    # to this child. Finish the bounded Disable/disconnect wait.
                    interrupted = True
            if interrupted:
                self.get_logger().warn(
                    "[SAFETY] Additional interrupt received; completed the bounded shutdown wait."
                )
            if worker.is_alive():
                self.get_logger().error(
                    "[SAFETY] SDK worker did not stop before timeout; it retains exclusive SDK ownership."
                )

    def destroy_node(self) -> bool:
        self.shutdown()
        return super().destroy_node()


def _run_self_test() -> None:
    """No-SDK unit checks for profile math and disabled-by-default UI policy."""
    pose_library = load_pose_library(
        _default_pose_library_path(), _ros_joint_order_for_side("left")
    )
    assert pose_library.model == "Wuji Hand 1"
    assert pose_library.handedness == "left"
    assert pose_library.dof == 20
    assert pose_library.unit == "rad"
    assert pose_library.canonical_joint_order == _ros_joint_order_for_side("left")
    assert tuple(pose_library.poses) == REQUIRED_POSE_IDS
    assert all(
        len(qpos) == 20 and all(math.isfinite(value) for value in qpos)
        for qpos in pose_library.poses.values()
    )
    exact_open = pose_library.poses["open"]
    broad_lower = tuple(-2.0 for _ in range(20))
    broad_upper = tuple(2.0 for _ in range(20))
    exact_validated = CupMotionProfile.require_within_safe_bounds(
        exact_open,
        broad_lower,
        broad_upper,
        label="pose 'open'",
        joint_names=pose_library.canonical_joint_order,
    )
    assert exact_validated == exact_open
    invalid_target = list(exact_open)
    invalid_target[4] = broad_upper[4] + 0.001
    try:
        CupMotionProfile.require_within_safe_bounds(
            invalid_target,
            broad_lower,
            broad_upper,
            label="pose 'invalid'",
            joint_names=pose_library.canonical_joint_order,
        )
    except ValueError as exc:
        assert "qpos[4] left_finger2_joint1" in str(exc)
    else:
        raise AssertionError("firmware-unsafe JSON target was not refused")
    long_target = tuple(1.0 for _ in range(20))
    pose_duration = CupMotionProfile.library_pose_duration(
        tuple(0.0 for _ in range(20)), long_target
    )
    assert math.isclose(CupMotionProfile.JSON_POSE_PEAK_SPEED_RAD_S, 0.144)
    assert math.isclose(pose_duration, 1.5 / 0.144)
    assert pose_duration >= CupMotionProfile.JSON_POSE_MIN_DURATION_S
    assert 1.5 / pose_duration <= CupMotionProfile.JSON_POSE_PEAK_SPEED_RAD_S
    summary_lines = _pose_library_summary(pose_library).splitlines()
    assert summary_lines[:5] == [
        "Pose library: VALID",
        "Hand: Wuji Hand 1 left",
        "DOF: 20",
        "Canonical order: URDF",
        "Loaded poses: 11",
    ]
    assert summary_lines[6:] == list(REQUIRED_POSE_IDS)

    profile = CupMotionProfile()
    lower = tuple(-2.0 for _ in range(20))
    upper = tuple(2.0 for _ in range(20))
    safe_lower, safe_upper = profile.safe_bounds(lower, upper)
    captured_open = tuple(0.1 for _ in range(20))
    targets = profile.targets(captured_open, "left", safe_lower, safe_upper)
    assert targets.open_position == captured_open
    assert exact_open == pose_library.poses["open"]
    assert CUP_GRASP_POSE_ID == "cup_grasp"
    assert CUP_GRASP_POSE_ID not in pose_library.poses
    assert set(LiveHandControlBridge._ACTION_BY_REQUEST.values()) == {
        "enable", "disable", "move_pose"
    }
    legacy_final_payload = json.dumps(targets.cup_grasp, separators=(",", ":"))
    assert hashlib.sha256(legacy_final_payload.encode()).hexdigest() == (
        "6613b86e8d029e23786ace09ddaad4b0d1583480b7f55a9183b2e0d7f4189d5d"
    )
    offline_bridge = type("OfflineBridge", (), {})()
    offline_bridge._profile = profile
    offline_bridge._pose_library = pose_library
    offline_bridge._pose_library_error = ""
    offline_bridge._targets = targets
    offline_bridge._joint_names = pose_library.canonical_joint_order
    offline_bridge._safe_lower = safe_lower
    offline_bridge._safe_upper = safe_upper
    assert LiveHandControlBridge._runtime_validated_pose_target(
        offline_bridge, "open"
    ) == exact_open
    assert LiveHandControlBridge._runtime_validated_pose_target(
        offline_bridge, CUP_GRASP_POSE_ID
    ) == targets.cup_grasp
    narrow_lower = (-0.06,) + tuple(-2.0 for _ in range(19))
    narrow_safe_lower, narrow_safe_upper = profile.safe_bounds(narrow_lower, upper)
    near_boundary = (narrow_safe_lower[0] - 0.001,) + tuple(0.0 for _ in range(19))
    near_boundary_targets = profile.targets(
        near_boundary, "left", narrow_safe_lower, narrow_safe_upper
    )
    assert math.isclose(
        near_boundary_targets.open_position[0], narrow_safe_lower[0], abs_tol=1.0e-12
    )
    outside_tolerance = (
        narrow_safe_lower[0] - profile.REFERENCE_CAPTURE_TOLERANCE_RAD - 1.0e-6,
    ) + tuple(0.0 for _ in range(19))
    try:
        profile.targets(outside_tolerance, "left", narrow_safe_lower, narrow_safe_upper)
    except ValueError:
        pass
    else:
        raise AssertionError("reference outside capture tolerance was not refused")
    assert all(
        safe_lower[index] <= value <= safe_upper[index]
        for index, value in enumerate(targets.cup_grasp)
    )
    print("live_hand_control self-test passed (no SDK connection, no motor calls)")


def main(argv: Optional[Sequence[str]] = None) -> None:
    parser = argparse.ArgumentParser(
        description="Single-SDK tactile and constrained hand-control backend"
    )
    parser.add_argument("--self-test", action="store_true", help="Run profile checks without ROS or SDK hardware")
    parsed, ros_args = parser.parse_known_args(argv)
    if parsed.self_test:
        _run_self_test()
        return

    rclpy.init(args=ros_args)
    node: Optional[LiveHandControlBridge] = None
    executor: Optional[MultiThreadedExecutor] = None
    try:
        node = LiveHandControlBridge()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # ros2 run can forward the same terminal Ctrl-C to both its wrapper and
        # this child. The first interrupt starts cleanup; later SIGINTs must not
        # interrupt the bounded Disable/disconnect path.
        try:
            signal.signal(signal.SIGINT, signal.SIG_IGN)
        except (AttributeError, ValueError):
            pass
        if node is not None:
            node.shutdown()
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
