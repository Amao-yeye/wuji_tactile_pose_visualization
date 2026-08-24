#!/usr/bin/env python3
"""Safety-gated Tk control panel for the official Wuji Hand ROS 2 interface.

The panel intentionally starts locked.  It never publishes a command at
startup, and it will not enable the hand until all 20 live positions have been
received, copied into the command buffer, and the operator confirms Enable.
"""

from __future__ import annotations

import argparse
import math
import queue
import threading
import tkinter as tk
import xml.etree.ElementTree as etree
from pathlib import Path
from tkinter import messagebox, ttk
from typing import Dict, List, Optional, Sequence, Tuple

import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from wujihand_msgs.srv import SetEnabled


JointVector = List[Tuple[str, float]]


class HandControlNode(Node):
    """ROS transport with no command publication during construction."""

    def __init__(self, hand_name: str) -> None:
        super().__init__("wuji_hand_control_gui")
        self.hand_name = hand_name.strip("/")
        prefix = f"/{self.hand_name}"

        self._latest_state: Optional[JointVector] = None
        self._state_lock = threading.Lock()
        self.events: queue.Queue[Tuple[str, bool, str]] = queue.Queue()

        self.create_subscription(
            JointState,
            f"{prefix}/joint_states",
            self._on_joint_state,
            qos_profile_sensor_data,
        )
        # Creating a publisher does not write to the hand. publish_command() is
        # called only after explicit Enable and a user-originated slider/test action.
        self.command_pub = self.create_publisher(
            JointState,
            f"{prefix}/joint_commands",
            qos_profile_sensor_data,
        )
        self.set_enabled_client = self.create_client(SetEnabled, f"{prefix}/set_enabled")

    def _on_joint_state(self, message: JointState) -> None:
        if len(message.name) != 20 or len(message.position) < 20:
            return
        state = [(str(name), float(position)) for name, position in zip(message.name, message.position)]
        if len({name for name, _ in state}) != 20 or not all(
            math.isfinite(position) for _, position in state
        ):
            return
        with self._state_lock:
            self._latest_state = state

    def take_latest_state(self) -> Optional[JointVector]:
        with self._state_lock:
            state = self._latest_state
            self._latest_state = None
        return state

    def command_subscriber_count(self) -> int:
        return self.command_pub.get_subscription_count()

    def publish_command(self, names: Sequence[str], positions: Sequence[float]) -> None:
        if len(names) != 20 or len(positions) != 20:
            raise ValueError("A Wuji Hand command must contain exactly 20 joint values")
        if not all(math.isfinite(position) for position in positions):
            raise ValueError("Refusing to publish a non-finite joint command")
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(names)
        message.position = [float(position) for position in positions]
        self.command_pub.publish(message)

    def request_set_enabled(self, enabled: bool) -> None:
        if not self.set_enabled_client.wait_for_service(timeout_sec=0.0):
            self.events.put(
                (
                    "set_enabled",
                    False,
                    "Official /set_enabled service is unavailable (the driver may still be read-only).",
                )
            )
            return

        request = SetEnabled.Request()
        request.finger_id = 255
        request.joint_id = 255
        request.enabled = enabled
        future = self.set_enabled_client.call_async(request)

        def completed(result_future) -> None:
            try:
                response = result_future.result()
                success = bool(response.success)
                message = response.message
            except Exception as exc:  # pragma: no cover - hardware/ROS failure path
                success = False
                message = f"Service call failed: {exc}"
            self.events.put(("set_enabled", success, message))

        future.add_done_callback(completed)


class HandControlGui:
    """Tk user interface and all local command safety gates."""

    # These limits are deliberately conservative for the first debugging GUI.
    # A slider may change the commanded position by no more than 0.03 rad in one
    # event and may not drift more than 0.20 rad from the last live-state sync.
    MAX_SLIDER_STEP_RAD = 0.03
    MAX_OFFSET_FROM_SYNC_RAD = 0.20
    TEST_STEP_RAD = 0.02
    # A fist is deliberately incremental.  It includes the two distal thumb
    # flex joints plus the same F2-F5 MCP/PIP/DIP joints used by the official
    # wave demo, but never sends that demo's much larger zero-based waveform.
    # Thumb J1/J2 remain individually adjustable because they respectively
    # have different mechanical functions and direction conventions.
    FIST_STEP_RAD = 0.05
    FIST_RAMP_STEP_RAD = 0.02
    FIST_RAMP_INTERVAL_MS = 60
    FIST_JOINT_COUNT = 14
    HOLD_PRE_ENABLE_COUNT = 8
    HOLD_PRE_ENABLE_INTERVAL_MS = 50

    def __init__(self, root: tk.Tk, node: HandControlNode) -> None:
        self.root = root
        self.node = node
        self.root.title("Wuji Hand 安全控制")
        self.root.geometry("1080x880")
        self.root.minsize(900, 620)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self.joint_names: List[str] = []
        self.actual_positions: Dict[str, float] = {}
        self.command_positions: Dict[str, float] = {}
        self.slider_bounds: Dict[str, Tuple[float, float]] = {}
        self.urdf_limits: Dict[str, Tuple[float, float]] = {}
        self.row_widgets: Dict[str, Tuple[tk.DoubleVar, tk.StringVar, tk.StringVar, tk.Scale]] = {}
        self.fist_joint_names: List[str] = []
        self.fist_baseline_positions: Dict[str, float] = {}
        self.has_complete_state = False
        self.control_enabled = False
        self.enable_pending = False
        self.gesture_pending = False
        self._enable_requested_flag = False
        self._gesture_target_positions: Dict[str, float] = {}
        self._gesture_label = ""
        self._updating_widgets = False
        self._hold_messages_remaining = 0

        self.status_var = tk.StringVar(
            value="控制未启用：正在等待来自真机的 20 个关节实时状态。"
        )
        self.test_joint_var = tk.StringVar()

        self._build_layout()
        self._refresh_controls()
        self.root.after(50, self._process_ros_updates)

    def _build_layout(self) -> None:
        top = ttk.Frame(self.root, padding=10)
        top.pack(fill=tk.X)

        ttk.Label(
            top,
            text=f"Wuji Hand 命名空间：/{self.node.hand_name}",
            font=("TkDefaultFont", 11, "bold"),
        ).grid(row=0, column=0, columnspan=5, sticky=tk.W)
        ttk.Label(
            top,
            text="操作顺序：① 读取当前位置  →  ② 启用  →  ③ 单关节微调或握拳。启用本身只保持当前姿态，不会让手跳到零位。",
            wraplength=960,
        ).grid(row=1, column=0, columnspan=5, sticky=tk.W, pady=(5, 8))

        self.read_button = ttk.Button(top, text="① 读取当前位置", command=self._read_current)
        self.read_button.grid(row=2, column=0, padx=(0, 6), sticky=tk.W)
        self.enable_button = ttk.Button(top, text="② 启用", command=self._enable)
        self.enable_button.grid(row=2, column=1, padx=6, sticky=tk.W)
        self.disable_button = ttk.Button(top, text="禁用（随时可按）", command=self._disable)
        self.disable_button.grid(row=2, column=2, padx=6, sticky=tk.W)
        ttk.Label(
            top,
            text=(
                f"滑条单次最多 {self.MAX_SLIDER_STEP_RAD:.3f} rad；微调步长 {self.TEST_STEP_RAD:.3f} rad"
            ),
        ).grid(row=2, column=3, columnspan=2, padx=(18, 0), sticky=tk.W)

        test = ttk.LabelFrame(self.root, text="单关节微调", padding=(10, 4))
        test.pack(fill=tk.X)
        ttk.Label(test, text="选择关节：").pack(side=tk.LEFT)
        self.test_combo = ttk.Combobox(
            test, textvariable=self.test_joint_var, width=31, state="disabled"
        )
        self.test_combo.pack(side=tk.LEFT, padx=6)
        self.test_plus_button = ttk.Button(
            test,
            text=f"正向 +{self.TEST_STEP_RAD:.3f} rad",
            command=lambda: self._run_single_joint_test(+self.TEST_STEP_RAD),
        )
        self.test_plus_button.pack(side=tk.LEFT, padx=3)
        self.test_minus_button = ttk.Button(
            test,
            text=f"负向 -{self.TEST_STEP_RAD:.3f} rad",
            command=lambda: self._run_single_joint_test(-self.TEST_STEP_RAD),
        )
        self.test_minus_button.pack(side=tk.LEFT, padx=3)
        ttk.Label(
            test,
            text="每次只改变所选关节；其他 19 个关节保持当前目标。按钮动作需要再次确认。",
        ).pack(side=tk.LEFT, padx=(14, 0))

        gesture = ttk.LabelFrame(self.root, text="握拳 / 松开（安全步进）", padding=(10, 4))
        gesture.pack(fill=tk.X, padx=10, pady=(0, 8))
        ttk.Label(
            gesture,
            text=(
                "握拳包含拇指 J3/J4，以及其余四指 F2–F5 的 J1/J3/J4；拇指 J1/J2 请用上方微调单独验证。"
            ),
        ).pack(side=tk.LEFT)
        self.fist_button = ttk.Button(
            gesture,
            text=f"握拳一步 +{self.FIST_STEP_RAD:.3f} rad",
            command=self._run_fist_step,
        )
        self.fist_button.pack(side=tk.LEFT, padx=(12, 3))
        self.release_button = ttk.Button(
            gesture,
            text="松开（回到读取时姿态）",
            command=self._release_fist,
        )
        self.release_button.pack(side=tk.LEFT, padx=3)

        table_container = ttk.Frame(self.root, padding=(10, 0, 10, 4))
        table_container.pack(fill=tk.BOTH, expand=True)
        self.canvas = tk.Canvas(table_container, highlightthickness=0)
        scrollbar = ttk.Scrollbar(table_container, orient=tk.VERTICAL, command=self.canvas.yview)
        self.table = ttk.Frame(self.canvas)
        self._table_window = self.canvas.create_window((0, 0), window=self.table, anchor=tk.NW)
        self.canvas.configure(yscrollcommand=scrollbar.set)
        self.table.bind(
            "<Configure>", lambda _event: self.canvas.configure(scrollregion=self.canvas.bbox("all"))
        )
        self.canvas.bind(
            "<Configure>",
            lambda event: self.canvas.itemconfigure(self._table_window, width=event.width),
        )
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        status = ttk.Label(
            self.root,
            textvariable=self.status_var,
            relief=tk.SUNKEN,
            anchor=tk.W,
            padding=8,
            wraplength=1000,
        )
        status.pack(fill=tk.X, padx=10, pady=(0, 10))

    def _make_joint_rows(self) -> None:
        for child in self.table.winfo_children():
            child.destroy()
        self.row_widgets.clear()

        headers = (
            "关节",
            "实际位置 (rad)\n真机实时反馈",
            "目标位置 (rad)\n下一条命令",
            "目标位置调节（不是速度）\n范围：读取姿态 ±0.20 rad",
        )
        for column, header in enumerate(headers):
            ttk.Label(self.table, text=header, font=("TkDefaultFont", 10, "bold")).grid(
                row=0, column=column, padx=5, pady=(0, 3), sticky=tk.W
            )

        for row, name in enumerate(self.joint_names, start=1):
            value_var = tk.DoubleVar(value=0.0)
            actual_var = tk.StringVar(value="—")
            command_var = tk.StringVar(value="—")
            ttk.Label(self.table, text=name).grid(row=row, column=0, padx=5, pady=2, sticky=tk.W)
            ttk.Label(self.table, textvariable=actual_var, width=14).grid(
                row=row, column=1, padx=5, pady=2, sticky=tk.E
            )
            ttk.Label(self.table, textvariable=command_var, width=14).grid(
                row=row, column=2, padx=5, pady=2, sticky=tk.E
            )
            slider = tk.Scale(
                self.table,
                from_=-1.0,
                to=1.0,
                resolution=0.001,
                orient=tk.HORIZONTAL,
                variable=value_var,
                length=520,
                showvalue=False,
                state=tk.DISABLED,
                command=lambda raw_value, joint_name=name: self._slider_changed(joint_name, raw_value),
            )
            slider.grid(row=row, column=3, padx=5, pady=1, sticky=tk.EW)
            self.row_widgets[name] = (value_var, actual_var, command_var, slider)
        self.table.columnconfigure(3, weight=1)
        self.test_combo.configure(values=self.joint_names)
        if self.joint_names:
            self.test_joint_var.set(self.joint_names[0])

    @staticmethod
    def _handedness_from_names(names: Sequence[str]) -> Optional[str]:
        for candidate in ("left", "right"):
            if all(name.startswith(candidate + "_") for name in names):
                return candidate
        return None

    def _discover_fist_joint_names(self) -> List[str]:
        """Return the 14 safe fist joints in the driver's live-state order."""
        selected: List[str] = []
        # Thumb distal flex only.  The remaining two thumb DOFs are deliberately
        # left to the explicit single-joint controls above.
        for joint in (3, 4):
            suffix = f"finger1_joint{joint}"
            matches = [name for name in self.joint_names if name.endswith(suffix)]
            if len(matches) != 1:
                return []
            selected.append(matches[0])
        for finger in range(2, 6):
            for joint in (1, 3, 4):
                suffix = f"finger{finger}_joint{joint}"
                matches = [name for name in self.joint_names if name.endswith(suffix)]
                if len(matches) != 1:
                    return []
                selected.append(matches[0])
        return selected

    def _load_official_urdf_limits(self) -> Dict[str, Tuple[float, float]]:
        handedness = self._handedness_from_names(self.joint_names)
        if handedness is None:
            return {}
        try:
            description_dir = Path(get_package_share_directory("wuji_description"))
            urdf_path = description_dir / "urdf" / f"{handedness}-ros.urdf"
            root = etree.parse(urdf_path).getroot()
        except (PackageNotFoundError, OSError, etree.ParseError) as exc:
            self.status_var.set(f"Could not load official URDF limits; using local safety window only: {exc}")
            return {}

        limits: Dict[str, Tuple[float, float]] = {}
        for joint in root.findall("joint"):
            name = joint.attrib.get("name")
            limit = joint.find("limit")
            if not name or limit is None:
                continue
            try:
                limits[name] = (float(limit.attrib["lower"]), float(limit.attrib["upper"]))
            except (KeyError, ValueError):
                continue
        return limits

    def _process_ros_updates(self) -> None:
        state = self.node.take_latest_state()
        if state is not None:
            self._accept_live_state(state)

        while True:
            try:
                event, success, detail = self.node.events.get_nowait()
            except queue.Empty:
                break
            if event == "set_enabled":
                self.enable_pending = False
                if success and self._enable_requested:
                    self.control_enabled = True
                    self.status_var.set(
                        "已启用（Control enabled）。实际位置来自真机；目标位置是下一条命令。"
                        "拖动滑条或点击按钮后才会发送受限的全手保持命令。"
                    )
                elif success:
                    self.control_enabled = False
                    self.status_var.set("所有关节已通过官方服务禁用。")
                else:
                    self.control_enabled = False
                    self.status_var.set(f"启用/禁用请求未被接受：{detail}")
                self._refresh_controls()

        self.root.after(50, self._process_ros_updates)

    def _accept_live_state(self, state: JointVector) -> None:
        names = [name for name, _ in state]
        if not self.has_complete_state:
            self.joint_names = names
            self.actual_positions = dict(state)
            self.command_positions = dict(state)
            self.urdf_limits = self._load_official_urdf_limits()
            self._make_joint_rows()
            self.fist_joint_names = self._discover_fist_joint_names()
            self.has_complete_state = True
            self._synchronize_targets_from_actual()
            if len(self.fist_joint_names) == self.FIST_JOINT_COUNT:
                self.status_var.set(
                    "已收到真机 20 个实时关节位置。控制仍未启用；目标已从实测姿态初始化，"
                    "尚未发布命令。启用后可使用握拳/松开。"
                )
            else:
                self.status_var.set(
                    "已收到真机 20 个实时关节位置。控制仍未启用；目标已从实测姿态初始化，"
                    "尚未发布命令。未找到预期的握拳关节，因此该手势不可用。"
                )
            self._refresh_controls()
            return

        if names != self.joint_names:
            self.status_var.set("Ignoring a joint-state message with an unexpected joint ordering.")
            return
        self.actual_positions = dict(state)
        for name, position in state:
            self.row_widgets[name][1].set(f"{position:+.4f}")

    def _synchronize_targets_from_actual(self) -> None:
        if not self.has_complete_state:
            return
        self._updating_widgets = True
        try:
            for name in self.joint_names:
                actual = self.actual_positions[name]
                urdf_lower, urdf_upper = self.urdf_limits.get(name, (-math.inf, math.inf))
                lower = max(urdf_lower, actual - self.MAX_OFFSET_FROM_SYNC_RAD)
                upper = min(urdf_upper, actual + self.MAX_OFFSET_FROM_SYNC_RAD)
                if lower > upper:
                    lower = upper = actual
                self.slider_bounds[name] = (lower, upper)
                self.command_positions[name] = actual
                value_var, actual_var, command_var, slider = self.row_widgets[name]
                slider.configure(from_=lower, to=upper)
                value_var.set(actual)
                actual_var.set(f"{actual:+.4f}")
                command_var.set(f"{actual:+.4f}")
            self.fist_baseline_positions = {
                name: self.actual_positions[name] for name in self.fist_joint_names
            }
        finally:
            self._updating_widgets = False

    def _read_current(self) -> None:
        if not self.has_complete_state:
            self.status_var.set("仍在等待真机的 20 个完整关节位置。")
            return
        if self.enable_pending or self.gesture_pending:
            self.status_var.set("启用/禁用或手势动作正在进行，暂时不能重新读取姿态。")
            return
        self._synchronize_targets_from_actual()
        self.status_var.set(
            "已从真机重新读取目标与松开姿态；没有发布任何命令。"
        )

    def _refresh_controls(self) -> None:
        interaction_pending = self.enable_pending or self.gesture_pending
        can_enable = self.has_complete_state and not self.control_enabled and not interaction_pending
        can_disable = self.control_enabled and not self.enable_pending
        can_move = self.has_complete_state and self.control_enabled and not interaction_pending
        can_fist = (
            can_move
            and len(self.fist_joint_names) == self.FIST_JOINT_COUNT
            and all(name in self.fist_baseline_positions for name in self.fist_joint_names)
        )
        self.read_button.configure(
            state=tk.NORMAL if self.has_complete_state and not interaction_pending else tk.DISABLED
        )
        self.enable_button.configure(state=tk.NORMAL if can_enable else tk.DISABLED)
        self.disable_button.configure(state=tk.NORMAL if can_disable else tk.DISABLED)
        for _name, (_value, _actual, _command, slider) in self.row_widgets.items():
            slider.configure(state=tk.NORMAL if can_move else tk.DISABLED)
        self.test_combo.configure(state="readonly" if can_move else "disabled")
        self.test_plus_button.configure(state=tk.NORMAL if can_move else tk.DISABLED)
        self.test_minus_button.configure(state=tk.NORMAL if can_move else tk.DISABLED)
        self.fist_button.configure(state=tk.NORMAL if can_fist else tk.DISABLED)
        self.release_button.configure(state=tk.NORMAL if can_fist else tk.DISABLED)

    @property
    def _enable_requested(self) -> bool:
        return self._enable_requested_flag

    @_enable_requested.setter
    def _enable_requested(self, value: bool) -> None:
        self._enable_requested_flag = value

    def _enable(self) -> None:
        if not self.has_complete_state or self.enable_pending or self.gesture_pending:
            return
        if self.node.command_subscriber_count() == 0:
            self.status_var.set(
                "拒绝启用：没有 /joint_commands 订阅者。请启动控制模式，不是只读模式。"
            )
            return
        if not messagebox.askyesno(
            "启用 Wuji Hand",
            "系统会先发送当前实测姿态作为保持目标，再调用官方服务启用全部 20 个关节。"
            "这一步不应产生明显动作。是否继续？",
            icon=messagebox.WARNING,
        ):
            return

        # Discard any pre-enable UI intent and seed every target from the latest
        # measured positions before even the first hold command is published.
        self._synchronize_targets_from_actual()
        self.enable_pending = True
        self._enable_requested = True
        self._hold_messages_remaining = self.HOLD_PRE_ENABLE_COUNT
        self.status_var.set("正在发送实测姿态保持目标，然后请求启用电机…")
        self._refresh_controls()
        self._publish_hold_before_enable()

    def _publish_hold_before_enable(self) -> None:
        if not self.enable_pending or not self._enable_requested:
            return
        if not self._publish_full_target("pre-enable measured hold"):
            self.enable_pending = False
            self._enable_requested = False
            self.status_var.set("无法发布实测保持目标，已取消启用。")
            self._refresh_controls()
            return
        self._hold_messages_remaining -= 1
        if self._hold_messages_remaining > 0:
            self.root.after(self.HOLD_PRE_ENABLE_INTERVAL_MS, self._publish_hold_before_enable)
            return
        self.status_var.set("实测保持目标已发送；正在请求官方启用服务…")
        self.node.request_set_enabled(True)

    def _disable(self) -> None:
        if not self.control_enabled or self.enable_pending:
            return
        # A queued ramp checks this flag before each subsequent command, so
        # Disable stops it before asking the driver to remove motor power.
        self.gesture_pending = False
        self._gesture_target_positions.clear()
        self._gesture_label = ""
        self.enable_pending = True
        self._enable_requested = False
        self.control_enabled = False
        self.status_var.set("正在请求官方服务禁用全部关节…")
        self._refresh_controls()
        self.node.request_set_enabled(False)

    def _slider_changed(self, name: str, raw_value: str) -> None:
        if self._updating_widgets or not self.control_enabled or self.enable_pending or self.gesture_pending:
            return
        try:
            requested = float(raw_value)
        except ValueError:
            return

        previous = self.command_positions[name]
        lower, upper = self.slider_bounds[name]
        requested = max(lower, min(upper, requested))
        delta = requested - previous
        if abs(delta) > self.MAX_SLIDER_STEP_RAD:
            requested = previous + math.copysign(self.MAX_SLIDER_STEP_RAD, delta)

        self._set_one_target_and_publish(name, requested, "slider")

    def _run_single_joint_test(self, delta: float) -> None:
        if not self.control_enabled or self.enable_pending or self.gesture_pending:
            return
        name = self.test_joint_var.get()
        if name not in self.command_positions:
            self.status_var.set("请先在下拉框中选择一个关节。")
            return
        target = self.command_positions[name] + delta
        if not messagebox.askyesno(
            "单关节微调",
            f"仅改变 {name}：{delta:+.3f} rad；其余 19 个关节保持当前目标。是否继续？",
            icon=messagebox.WARNING,
        ):
            return
        self._set_one_target_and_publish(name, target, "single-joint test")

    def _run_fist_step(self) -> None:
        """Request one limited curl increment for the safe thumb/finger set."""
        if not self.control_enabled or self.enable_pending or self.gesture_pending:
            return
        if len(self.fist_joint_names) != self.FIST_JOINT_COUNT:
            self.status_var.set("握拳不可用：未找到预期的拇指与 F2–F5 弯曲关节。")
            return

        targets: Dict[str, float] = {}
        for name in self.fist_joint_names:
            if name not in self.fist_baseline_positions or name not in self.slider_bounds:
                self.status_var.set("握拳不可用：请先点击“读取当前位置”捕获实测姿态。")
                return
            lower, upper = self.slider_bounds[name]
            # The slider window and this measured-pose cap both apply.  A user
            # must explicitly Read Current Position again before extending the
            # maximum safe offset from a newly observed pose.
            maximum = min(upper, self.fist_baseline_positions[name] + self.MAX_OFFSET_FROM_SYNC_RAD)
            targets[name] = max(lower, min(maximum, self.command_positions[name] + self.FIST_STEP_RAD))

        if all(abs(targets[name] - self.command_positions[name]) < 1e-9 for name in targets):
            self.status_var.set(
                "握拳目标已达到相对读取姿态 +0.200 rad 的安全上限。请松开或禁用。"
            )
            return
        if not messagebox.askyesno(
            "握拳一步",
            "拇指 J3/J4 与 F2–F5 的 J1/J3/J4 将从当前目标最多增加 +0.050 rad。"
            "其他关节保持不动；每条渐进命令最多变化 0.020 rad。是否继续？",
            icon=messagebox.WARNING,
        ):
            return
        self._begin_gesture_ramp("握拳一步", targets)

    def _release_fist(self) -> None:
        """Return the safe thumb/finger set to the last Read/Enable pose."""
        if not self.control_enabled or self.enable_pending or self.gesture_pending:
            return
        if len(self.fist_joint_names) != self.FIST_JOINT_COUNT:
            self.status_var.set("松开不可用：未找到预期的拇指与 F2–F5 弯曲关节。")
            return
        try:
            targets = {
                name: self.fist_baseline_positions[name] for name in self.fist_joint_names
            }
        except KeyError:
            self.status_var.set("松开不可用：请先点击“读取当前位置”捕获实测姿态。")
            return
        if all(abs(targets[name] - self.command_positions[name]) < 1e-9 for name in targets):
            self.status_var.set("握拳所用关节已处于读取时捕获的松开姿态。")
            return
        if not messagebox.askyesno(
            "松开（回到读取时姿态）",
            "让拇指 J3/J4 与 F2–F5 的 J1/J3/J4 回到最近一次读取/启用时的真实姿态。"
            "其他关节保持不动。是否继续？",
            icon=messagebox.WARNING,
        ):
            return
        self._begin_gesture_ramp("回到读取时姿态", targets)

    def _begin_gesture_ramp(self, label: str, targets: Dict[str, float]) -> None:
        self.gesture_pending = True
        self._gesture_label = label
        self._gesture_target_positions = dict(targets)
        self.status_var.set(f"正在渐进执行：{label}；可随时点击“禁用”停止。")
        self._refresh_controls()
        self._advance_gesture_ramp()

    def _advance_gesture_ramp(self) -> None:
        if not self.gesture_pending or not self.control_enabled or self.enable_pending:
            return

        still_moving = False
        self._updating_widgets = True
        try:
            for name, target in self._gesture_target_positions.items():
                current = self.command_positions[name]
                delta = target - current
                if abs(delta) <= self.FIST_RAMP_STEP_RAD:
                    next_target = target
                else:
                    next_target = current + math.copysign(self.FIST_RAMP_STEP_RAD, delta)
                if abs(target - next_target) > 1e-9:
                    still_moving = True
                self.command_positions[name] = next_target
                value_var, _actual_var, command_var, _slider = self.row_widgets[name]
                value_var.set(next_target)
                command_var.set(f"{next_target:+.4f}")
        finally:
            self._updating_widgets = False

        if not self._publish_full_target(self._gesture_label):
            self.gesture_pending = False
            self._gesture_target_positions.clear()
            self.status_var.set("无法发布完整关节命令，手势已取消。")
            self._refresh_controls()
            return
        if still_moving:
            self.root.after(self.FIST_RAMP_INTERVAL_MS, self._advance_gesture_ramp)
            return

        completed_label = self._gesture_label
        self.gesture_pending = False
        self._gesture_target_positions.clear()
        self._gesture_label = ""
        self.status_var.set(f"已完成：{completed_label}。下一次手势前请先点击“读取当前位置”。")
        self._refresh_controls()

    def _set_one_target_and_publish(self, name: str, requested: float, source: str) -> None:
        lower, upper = self.slider_bounds[name]
        previous = self.command_positions[name]
        requested = max(lower, min(upper, requested))
        delta = requested - previous
        if abs(delta) > self.MAX_SLIDER_STEP_RAD:
            requested = previous + math.copysign(self.MAX_SLIDER_STEP_RAD, delta)

        self.command_positions[name] = requested
        self._updating_widgets = True
        try:
            value_var, _actual_var, command_var, _slider = self.row_widgets[name]
            value_var.set(requested)
            command_var.set(f"{requested:+.4f}")
        finally:
            self._updating_widgets = False
        self._publish_full_target(source)

    def _publish_full_target(self, source: str) -> bool:
        try:
            positions = [self.command_positions[name] for name in self.joint_names]
            self.node.publish_command(self.joint_names, positions)
            self.status_var.set(
                f"Published {source}: 20-joint command; only the selected slider target may differ from hold."
            )
            return True
        except (KeyError, ValueError, RuntimeError) as exc:
            self.status_var.set(f"Refusing command: {exc}")
            return False

    def _on_close(self) -> None:
        if self.control_enabled or self.enable_pending:
            messagebox.showwarning(
                "Disable before closing",
                "Control is active or a service request is pending. Use Disable and wait for confirmation "
                "before closing this panel.",
            )
            return
        self.root.destroy()


def main(argv: Optional[Sequence[str]] = None) -> None:
    parser = argparse.ArgumentParser(description="Safety-gated Wuji Hand ROS 2 control GUI")
    parser.add_argument("--hand-name", default="hand_0", help="ROS namespace without a leading slash")
    parsed, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = HandControlNode(parsed.hand_name)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, name="wuji-hand-gui-ros", daemon=True)
    spin_thread.start()

    root = tk.Tk()
    HandControlGui(root, node)
    try:
        root.mainloop()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
