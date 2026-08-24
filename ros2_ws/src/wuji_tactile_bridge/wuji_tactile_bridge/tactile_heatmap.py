#!/usr/bin/env python3
"""Fixed-grid tactile heatmap and pressure-history UI.

The window visualizes the raw paired-tactile matrix strictly as a 2D row-major
grid.  It intentionally contains no PointCloud2, MarkerArray, TF, or inferred
taxel-to-URDF mapping because no official mapping was found for this stream.
"""

from __future__ import annotations

import argparse
import math
import statistics
import threading
import time
import tkinter as tk
from collections import deque
from dataclasses import dataclass
from tkinter import messagebox, ttk
from typing import Deque, Optional, Sequence, Tuple

from matplotlib import colormaps
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from wuji_tactile_msgs.msg import TactilePressureFrame


@dataclass(frozen=True)
class TactileSnapshot:
    sequence: int
    timestamp_ms: int
    handedness: int
    rows: int
    cols: int
    pressure: Tuple[float, ...]


class TactileMonitorNode(Node):
    """ROS-only consumer for raw tactile frames published by tactile_bridge."""

    def __init__(self, hand_name: str) -> None:
        super().__init__("tactile_heatmap")
        self.hand_name = hand_name.strip("/")
        self._latest: Optional[TactileSnapshot] = None
        self._lock = threading.Lock()
        self._bad_shape_count = 0
        self.create_subscription(
            TactilePressureFrame,
            f"/{self.hand_name}/tactile/pressure",
            self._on_frame,
            qos_profile_sensor_data,
        )

    def _on_frame(self, message: TactilePressureFrame) -> None:
        rows = int(message.rows)
        cols = int(message.cols)
        pressure = tuple(float(value) for value in message.pressure)
        if rows <= 0 or cols <= 0 or rows * cols != len(pressure):
            self._bad_shape_count += 1
            if self._bad_shape_count == 1 or self._bad_shape_count % 100 == 0:
                self.get_logger().warn(
                    f"Ignoring malformed tactile frame: rows={rows}, cols={cols}, values={len(pressure)}"
                )
            return
        with self._lock:
            self._latest = TactileSnapshot(
                sequence=int(message.sequence),
                timestamp_ms=int(message.device_timestamp_ms),
                handedness=int(message.handedness),
                rows=rows,
                cols=cols,
                pressure=pressure,
            )

    def latest(self) -> Optional[TactileSnapshot]:
        with self._lock:
            return self._latest


class TactileHeatmapApp:
    """Tk display with fixed cell positions and deliberately fixed color scale."""

    CELL_SIZE = 17
    GRID_LEFT = 38
    GRID_TOP = 28
    HEATMAP_WIDTH = 650
    HEATMAP_HEIGHT = 470
    PLOT_WIDTH = 530
    PLOT_HEIGHT = 260
    MAX_HISTORY_SAMPLES = 3600
    RAW_VMIN = 0.0
    DELTA_VMIN = 0.0
    TEMPORAL_VMIN = 0.0
    DEFAULT_RAW_VMAX = 1.0
    DEFAULT_DELTA_VMAX = 0.25
    DEFAULT_TEMPORAL_VMAX = 0.05
    DEFAULT_DELTA_THRESHOLD = 0.10
    MIN_DELTA_VMAX = 0.05
    MIN_TEMPORAL_VMAX = 0.005
    MIN_BASELINE_SAMPLES = 5
    INVALID_COLOR = "#3f3f46"
    INACTIVE_COLOR = "#111827"
    INFERNO = colormaps["inferno"]

    def __init__(self, root: tk.Tk, node: TactileMonitorNode) -> None:
        self.root = root
        self.node = node
        self.root.title("Wuji Hand Tactile — 只读二维可视化")
        self.root.geometry("2200x990")
        self.root.minsize(1960, 780)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self.latest_snapshot: Optional[TactileSnapshot] = None
        self._grid_shape: Optional[Tuple[int, int]] = None
        self._cell_items: list[int] = []
        self._last_history_sequence: Optional[int] = None
        self._history: Deque[Tuple[float, float, float]] = deque(maxlen=self.MAX_HISTORY_SAMPLES)

        # Data layer.  These values never receive visualization clipping or
        # thresholding.  valid_mask means "current raw taxel is finite".
        self.raw_pressure: Optional[Tuple[float, ...]] = None
        self.valid_mask: Optional[Tuple[bool, ...]] = None
        self.baseline: Optional[Tuple[float, ...]] = None
        self.baseline_valid: Optional[Tuple[bool, ...]] = None
        self.baseline_sample_counts: Optional[Tuple[int, ...]] = None
        self.signed_delta: Optional[Tuple[float, ...]] = None
        self.positive_delta: Optional[Tuple[float, ...]] = None
        self.prev_raw: Optional[Tuple[float, ...]] = None
        self.temporal_valid_mask: Optional[Tuple[bool, ...]] = None
        self.temporal_signed_delta: Optional[Tuple[float, ...]] = None
        self.temporal_positive_delta: Optional[Tuple[float, ...]] = None
        self._last_temporal_sequence: Optional[int] = None

        # Visualization layer.  It is derived from the data layer but never
        # written back into raw_pressure, signed_delta, or positive_delta.
        self.display_delta_source: Optional[Tuple[float, ...]] = None
        self.active_mask: Optional[Tuple[bool, ...]] = None
        self._delta_capture_started_at: Optional[float] = None
        self._delta_capture_first_sample_at: Optional[float] = None
        self._delta_capture_shape: Optional[Tuple[int, int]] = None
        self._delta_capture_samples: Optional[list[list[float]]] = None
        self._delta_capture_last_sequence: Optional[int] = None
        self._delta_capture_frame_count = 0
        self._delta_grid_shape: Optional[Tuple[int, int]] = None
        self._delta_cell_items: list[int] = []
        self._temporal_grid_shape: Optional[Tuple[int, int]] = None
        self._temporal_cell_items: list[int] = []

        self.layout_var = tk.StringVar(
            value=f"等待 /{self.node.hand_name}/tactile/pressure …"
        )
        self.stats_var = tk.StringVar(value="尚未收到原始 pressure frame。")
        self.status_var = tk.StringVar(
            value="纯二维网格：当前没有可靠的 paired tactile → 手模型表面坐标映射。"
        )
        self.raw_vmax_var = tk.StringVar(value=f"{self.DEFAULT_RAW_VMAX:.3f}")
        self.delta_vmax_var = tk.StringVar(value=f"{self.DEFAULT_DELTA_VMAX:.3f}")
        self.delta_threshold_var = tk.StringVar(value=f"{self.DEFAULT_DELTA_THRESHOLD:.3f}")
        self.temporal_vmax_var = tk.StringVar(value=f"{self.DEFAULT_TEMPORAL_VMAX:.3f}")
        self.delta_baseline_offset_var = tk.StringVar(value="0.000")
        self.window_s_var = tk.StringVar(value="10")
        self.delta_stats_var = tk.StringVar(
            value="Delta 未初始化：请先 Capture Baseline（连续 2 秒中位数）。"
        )
        self.delta_state_var = tk.StringVar(
            value="Delta baseline 尚未采集；Delta 图不会把 0 当作已校准数据。"
        )
        self.temporal_stats_var = tk.StringVar(
            value="Temporal Delta 等待首个前序 raw frame。"
        )

        self._build_layout()
        self.root.after(33, self._refresh)

    def _build_layout(self) -> None:
        top = ttk.Frame(self.root, padding=10)
        top.pack(fill=tk.X)
        ttk.Label(top, text="Wuji Hand paired tactile（只读）", font=("TkDefaultFont", 12, "bold")).pack(
            anchor=tk.W
        )
        ttk.Label(
            top,
            text=(
                "Raw Pressure 显示 SDK 原始值；Baseline Residual 显示相对 captured baseline 的变化；"
                "Temporal Delta 严格显示 raw_t − raw_{t−1}。三图均固定颜色范围、不会按帧自动缩放；"
                "此窗口不发布 joint command。"
            ),
            wraplength=2140,
        ).pack(anchor=tk.W, pady=(4, 0))
        ttk.Label(top, textvariable=self.layout_var).pack(anchor=tk.W, pady=(4, 0))

        settings = ttk.LabelFrame(
            self.root, text="Raw / Baseline Residual / Temporal 显示与基线设置（只读）", padding=(10, 5)
        )
        settings.pack(fill=tk.X, padx=10, pady=(0, 8))
        ttk.Label(settings, text="Raw vmin（固定）：0.00").grid(row=0, column=0, sticky=tk.W)
        ttk.Label(settings, text="Raw vmax：").grid(row=0, column=1, sticky=tk.W, padx=(14, 0))
        ttk.Entry(settings, textvariable=self.raw_vmax_var, width=8).grid(row=0, column=2, padx=(2, 14))
        ttk.Label(settings, text="Baseline vmin（固定）：0.00").grid(row=0, column=3, sticky=tk.W)
        ttk.Label(settings, text="Baseline vmax：").grid(row=0, column=4, sticky=tk.W, padx=(14, 0))
        ttk.Entry(settings, textvariable=self.delta_vmax_var, width=8).grid(row=0, column=5, padx=(2, 14))
        ttk.Label(settings, text="Temporal vmin（固定）：0.00").grid(row=0, column=6, sticky=tk.W)
        ttk.Label(settings, text="Temporal vmax：").grid(row=0, column=7, sticky=tk.W, padx=(14, 0))
        ttk.Entry(settings, textvariable=self.temporal_vmax_var, width=8).grid(row=0, column=8, padx=(2, 14))
        ttk.Label(settings, text="曲线窗口 (s)：").grid(row=0, column=9, sticky=tk.W)
        ttk.Entry(settings, textvariable=self.window_s_var, width=6).grid(row=0, column=10, padx=(2, 14))

        ttk.Label(settings, text="Baseline threshold（仅视觉 activity）：").grid(
            row=1, column=0, columnspan=2, sticky=tk.W, pady=(5, 0)
        )
        ttk.Entry(settings, textvariable=self.delta_threshold_var, width=8).grid(
            row=1, column=2, padx=(2, 14), pady=(5, 0)
        )
        ttk.Label(settings, text="Baseline offset（仅视觉）：").grid(
            row=1, column=3, columnspan=2, sticky=tk.W, pady=(5, 0)
        )
        ttk.Entry(settings, textvariable=self.delta_baseline_offset_var, width=8).grid(
            row=1, column=5, padx=(2, 14), pady=(5, 0)
        )
        ttk.Label(
            settings,
            text="Temporal = raw_t − raw_{t−1}；不使用 baseline、offset 或 Baseline threshold。",
        ).grid(row=1, column=6, columnspan=5, sticky=tk.W, pady=(5, 0))

        ttk.Separator(settings, orient=tk.HORIZONTAL).grid(
            row=2, column=0, columnspan=11, sticky=tk.EW, pady=(7, 5)
        )
        ttk.Button(
            settings,
            text="Capture Baseline（2 秒 median）",
            command=self._capture_delta_baseline,
        ).grid(row=3, column=0, columnspan=3, sticky=tk.W)
        ttk.Button(settings, text="Reset Baseline", command=self._reset_delta_baseline).grid(
            row=3, column=3, sticky=tk.W, padx=(8, 12)
        )
        ttk.Label(
            settings,
            text="Baseline display = max(signed Δ − offset, 0)；仅绘图按 Baseline vmax 饱和。",
        ).grid(row=3, column=4, columnspan=7, sticky=tk.W)
        ttk.Label(settings, textvariable=self.delta_state_var, wraplength=2120).grid(
            row=4, column=0, columnspan=11, sticky=tk.W, pady=(4, 0)
        )

        content = ttk.Frame(self.root, padding=(10, 0, 10, 4))
        content.pack(fill=tk.BOTH, expand=True)
        heatmaps = ttk.Frame(content)
        heatmaps.pack(fill=tk.BOTH, expand=True)
        heatmap_frame = ttk.LabelFrame(
            heatmaps, text="Raw Pressure（SDK 原始值；inferno；固定 [0, Raw vmax]）", padding=5
        )
        heatmap_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 6))
        self.heatmap_canvas = tk.Canvas(
            heatmap_frame,
            width=self.HEATMAP_WIDTH,
            height=self.HEATMAP_HEIGHT,
            background="#101010",
            highlightthickness=0,
        )
        self.heatmap_canvas.pack(fill=tk.BOTH, expand=True)

        delta_frame = ttk.LabelFrame(
            heatmaps, text="Baseline Residual（signed Δ − offset；inferno；固定 [0, Baseline vmax]）", padding=5
        )
        delta_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.delta_heatmap_canvas = tk.Canvas(
            delta_frame,
            width=self.HEATMAP_WIDTH,
            height=self.HEATMAP_HEIGHT,
            background="#101010",
            highlightthickness=0,
        )
        self.delta_heatmap_canvas.pack(fill=tk.BOTH, expand=True)

        temporal_frame = ttk.LabelFrame(
            heatmaps,
            text="Temporal Delta（raw_t − raw_{t−1} positive；inferno；固定 [0, Temporal vmax]）",
            padding=5,
        )
        temporal_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.temporal_heatmap_canvas = tk.Canvas(
            temporal_frame,
            width=self.HEATMAP_WIDTH,
            height=self.HEATMAP_HEIGHT,
            background="#101010",
            highlightthickness=0,
        )
        self.temporal_heatmap_canvas.pack(fill=tk.BOTH, expand=True)

        statistics_row = ttk.Frame(content)
        statistics_row.pack(fill=tk.X, pady=(6, 6))
        stats_frame = ttk.LabelFrame(statistics_row, text="Raw Pressure 当前帧统计", padding=8)
        stats_frame.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 6))
        ttk.Label(stats_frame, textvariable=self.stats_var, justify=tk.LEFT, wraplength=620).pack(
            anchor=tk.W
        )

        delta_stats_frame = ttk.LabelFrame(statistics_row, text="Baseline Residual 当前帧统计", padding=8)
        delta_stats_frame.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 6))
        ttk.Label(delta_stats_frame, textvariable=self.delta_stats_var, justify=tk.LEFT, wraplength=650).pack(
            anchor=tk.W
        )

        temporal_stats_frame = ttk.LabelFrame(statistics_row, text="Temporal Delta 当前帧统计", padding=8)
        temporal_stats_frame.pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Label(
            temporal_stats_frame, textvariable=self.temporal_stats_var, justify=tk.LEFT, wraplength=650
        ).pack(anchor=tk.W)

        plot_frame = ttk.LabelFrame(content, text="Raw Pressure vs time（真实 max 红 / mean 青；不使用显示裁剪）", padding=5)
        plot_frame.pack(fill=tk.X)
        self.plot_canvas = tk.Canvas(
            plot_frame,
            width=self.PLOT_WIDTH,
            height=self.PLOT_HEIGHT,
            background="#101010",
            highlightthickness=0,
        )
        self.plot_canvas.pack(anchor=tk.W)

        ttk.Label(
            self.root,
            textvariable=self.status_var,
            relief=tk.SUNKEN,
            anchor=tk.W,
            padding=8,
            wraplength=2140,
        ).pack(fill=tk.X, padx=10, pady=(0, 10))

    @staticmethod
    def _finite_values(values: Sequence[float]) -> list[float]:
        return [value for value in values if math.isfinite(value)]

    def _numeric(self, variable: tk.StringVar, default: float, minimum: float) -> float:
        try:
            value = float(variable.get())
        except ValueError:
            return default
        if not math.isfinite(value) or value < minimum:
            return default
        return value

    def _build_grid(self, rows: int, cols: int) -> None:
        self.heatmap_canvas.delete("all")
        self._cell_items.clear()
        self._grid_shape = (rows, cols)
        width = self.GRID_LEFT + cols * self.CELL_SIZE + 10
        height = self.GRID_TOP + rows * self.CELL_SIZE + 10
        self.heatmap_canvas.configure(width=max(self.HEATMAP_WIDTH, width), height=max(self.HEATMAP_HEIGHT, height))

        for col in range(cols):
            if col % 5 == 0 or col == cols - 1:
                self.heatmap_canvas.create_text(
                    self.GRID_LEFT + col * self.CELL_SIZE + self.CELL_SIZE / 2,
                    12,
                    text=str(col),
                    fill="#cbd5e1",
                    font=("TkDefaultFont", 8),
                )
        for row in range(rows):
            self.heatmap_canvas.create_text(
                20,
                self.GRID_TOP + row * self.CELL_SIZE + self.CELL_SIZE / 2,
                text=str(row),
                fill="#cbd5e1",
                font=("TkDefaultFont", 8),
            )
            for col in range(cols):
                x0 = self.GRID_LEFT + col * self.CELL_SIZE
                y0 = self.GRID_TOP + row * self.CELL_SIZE
                item = self.heatmap_canvas.create_rectangle(
                    x0,
                    y0,
                    x0 + self.CELL_SIZE,
                    y0 + self.CELL_SIZE,
                    outline="#252525",
                    fill="#202020",
                )
                self._cell_items.append(item)

    @classmethod
    def _inferno_color(cls, value: float, maximum: float) -> str:
        if not math.isfinite(value):
            return cls.INVALID_COLOR
        # This is display-only clipping.  The caller retains the original
        # pressure / delta values for statistics and later analysis.
        ratio = max(0.0, min(1.0, value / maximum))
        red, green, blue, _ = cls.INFERNO(ratio)
        red, green, blue = round(red * 255), round(green * 255), round(blue * 255)
        return f"#{red:02x}{green:02x}{blue:02x}"

    def _render_raw_heatmap(self, snapshot: TactileSnapshot, raw: Sequence[float]) -> Tuple[float, float]:
        """Render direct SDK raw pressure with fixed [0, Raw vmax] inferno."""
        if self._grid_shape != (snapshot.rows, snapshot.cols):
            self._build_grid(snapshot.rows, snapshot.cols)

        raw_vmax = self._numeric(self.raw_vmax_var, self.DEFAULT_RAW_VMAX, 1e-6)
        for item, value in zip(self._cell_items, raw):
            self.heatmap_canvas.itemconfigure(item, fill=self._inferno_color(value, raw_vmax))

        finite = self._finite_values(raw)
        return (max(finite), sum(finite) / len(finite)) if finite else (float("nan"), float("nan"))

    def _build_delta_grid(self, rows: int, cols: int) -> None:
        """Build the Delta grid with the same fixed coordinates as Raw."""
        self.delta_heatmap_canvas.delete("all")
        self._delta_cell_items.clear()
        self._delta_grid_shape = (rows, cols)
        width = self.GRID_LEFT + cols * self.CELL_SIZE + 10
        height = self.GRID_TOP + rows * self.CELL_SIZE + 10
        self.delta_heatmap_canvas.configure(
            width=max(self.HEATMAP_WIDTH, width), height=max(self.HEATMAP_HEIGHT, height)
        )

        for col in range(cols):
            if col % 5 == 0 or col == cols - 1:
                self.delta_heatmap_canvas.create_text(
                    self.GRID_LEFT + col * self.CELL_SIZE + self.CELL_SIZE / 2,
                    12,
                    text=str(col),
                    fill="#cbd5e1",
                    font=("TkDefaultFont", 8),
                )
        for row in range(rows):
            self.delta_heatmap_canvas.create_text(
                20,
                self.GRID_TOP + row * self.CELL_SIZE + self.CELL_SIZE / 2,
                text=str(row),
                fill="#cbd5e1",
                font=("TkDefaultFont", 8),
            )
            for col in range(cols):
                x0 = self.GRID_LEFT + col * self.CELL_SIZE
                y0 = self.GRID_TOP + row * self.CELL_SIZE
                item = self.delta_heatmap_canvas.create_rectangle(
                    x0,
                    y0,
                    x0 + self.CELL_SIZE,
                    y0 + self.CELL_SIZE,
                    outline="#252525",
                    fill="#202020",
                )
                self._delta_cell_items.append(item)

    def _build_temporal_grid(self, rows: int, cols: int) -> None:
        """Build Temporal Delta with the exact same row/column geometry as Raw."""
        self.temporal_heatmap_canvas.delete("all")
        self._temporal_cell_items.clear()
        self._temporal_grid_shape = (rows, cols)
        width = self.GRID_LEFT + cols * self.CELL_SIZE + 10
        height = self.GRID_TOP + rows * self.CELL_SIZE + 10
        self.temporal_heatmap_canvas.configure(
            width=max(self.HEATMAP_WIDTH, width), height=max(self.HEATMAP_HEIGHT, height)
        )

        for col in range(cols):
            if col % 5 == 0 or col == cols - 1:
                self.temporal_heatmap_canvas.create_text(
                    self.GRID_LEFT + col * self.CELL_SIZE + self.CELL_SIZE / 2,
                    12,
                    text=str(col),
                    fill="#cbd5e1",
                    font=("TkDefaultFont", 8),
                )
        for row in range(rows):
            self.temporal_heatmap_canvas.create_text(
                20,
                self.GRID_TOP + row * self.CELL_SIZE + self.CELL_SIZE / 2,
                text=str(row),
                fill="#cbd5e1",
                font=("TkDefaultFont", 8),
            )
            for col in range(cols):
                x0 = self.GRID_LEFT + col * self.CELL_SIZE
                y0 = self.GRID_TOP + row * self.CELL_SIZE
                item = self.temporal_heatmap_canvas.create_rectangle(
                    x0,
                    y0,
                    x0 + self.CELL_SIZE,
                    y0 + self.CELL_SIZE,
                    outline="#252525",
                    fill="#202020",
                )
                self._temporal_cell_items.append(item)

    def _render_temporal_waiting(self, snapshot: TactileSnapshot) -> None:
        """Show first-frame state without manufacturing a zero previous frame."""
        if self._temporal_grid_shape != (snapshot.rows, snapshot.cols):
            self._build_temporal_grid(snapshot.rows, snapshot.cols)

        for item, raw in zip(self._temporal_cell_items, snapshot.pressure):
            color = self.INACTIVE_COLOR if math.isfinite(raw) else self.INVALID_COLOR
            self.temporal_heatmap_canvas.itemconfigure(item, fill=color)

        self.temporal_heatmap_canvas.delete("temporal_status")
        self.temporal_heatmap_canvas.create_text(
            self.GRID_LEFT + snapshot.cols * self.CELL_SIZE / 2,
            self.GRID_TOP + snapshot.rows * self.CELL_SIZE / 2,
            text="Temporal Delta waiting\nfor next raw frame",
            fill="#e2e8f0",
            font=("TkDefaultFont", 11, "bold"),
            justify=tk.CENTER,
            tags="temporal_status",
        )
        self.temporal_stats_var.set(
            "Temporal Delta waiting：首帧仅保存 prev_raw；未生成伪造 delta。"
        )

    def _render_delta_uninitialized(self, snapshot: TactileSnapshot) -> None:
        """Render a deliberate uninitialized state; never synthesize zero data."""
        if self._delta_grid_shape != (snapshot.rows, snapshot.cols):
            self._build_delta_grid(snapshot.rows, snapshot.cols)

        for item, raw in zip(self._delta_cell_items, snapshot.pressure):
            color = self.INACTIVE_COLOR if math.isfinite(raw) else self.INVALID_COLOR
            self.delta_heatmap_canvas.itemconfigure(item, fill=color)

        self.delta_heatmap_canvas.delete("delta_status")
        message = (
            "正在采集 Delta baseline…\n"
            "保持 tactile glove 无外力"
            if self._delta_capture_started_at is not None
            else "Baseline not captured\n请点击 Capture Baseline"
        )
        self.delta_heatmap_canvas.create_text(
            self.GRID_LEFT + snapshot.cols * self.CELL_SIZE / 2,
            self.GRID_TOP + snapshot.rows * self.CELL_SIZE / 2,
            text=message,
            fill="#e2e8f0",
            font=("TkDefaultFont", 11, "bold"),
            justify=tk.CENTER,
            tags="delta_status",
        )
        if self._delta_capture_started_at is not None:
            self.delta_stats_var.set(
                "Delta 未初始化：正在采集 2 秒 baseline；采集完成前不会产生 delta。"
            )
        else:
            self.delta_stats_var.set(
                "Baseline not captured：Delta 未初始化；未使用全 0 baseline。"
            )

    def _delta_display_color(self, value: float, is_active: bool, maximum: float) -> str:
        if not math.isfinite(value):
            return self.INVALID_COLOR  # Current-raw or baseline-invalid taxel.
        if not is_active:
            return self.INACTIVE_COLOR
        return self._inferno_color(value, maximum)

    def _update_delta_values(self, snapshot: TactileSnapshot) -> Optional[Tuple[float, ...]]:
        """Build data and visualization layers without mutating raw Delta data."""
        self.raw_pressure = tuple(snapshot.pressure)
        self.valid_mask = tuple(math.isfinite(raw) for raw in self.raw_pressure)
        baseline = self.baseline
        baseline_valid = self.baseline_valid
        if (
            baseline is None
            or baseline_valid is None
            or len(baseline) != len(self.raw_pressure)
            or len(baseline_valid) != len(self.raw_pressure)
        ):
            self.signed_delta = None
            self.positive_delta = None
            self.display_delta_source = None
            self.active_mask = None
            return None

        signed: list[float] = []
        positive: list[float] = []
        display_source: list[float] = []
        active: list[bool] = []
        offset = self._numeric(self.delta_baseline_offset_var, 0.0, 0.0)
        threshold = self._numeric(
            self.delta_threshold_var, self.DEFAULT_DELTA_THRESHOLD, 0.0
        )
        for raw, baseline_value, raw_valid, baseline_is_valid in zip(
            self.raw_pressure, baseline, self.valid_mask, baseline_valid
        ):
            is_delta_valid = raw_valid and baseline_is_valid and math.isfinite(baseline_value)
            if not is_delta_valid:
                signed.append(float("nan"))
                positive.append(float("nan"))
                display_source.append(float("nan"))
                active.append(False)
                continue
            delta = raw - baseline_value
            display_value = max(delta - offset, 0.0)
            signed.append(delta)
            positive.append(max(delta, 0.0))
            display_source.append(display_value)
            active.append(display_value >= threshold)

        self.signed_delta = tuple(signed)
        self.positive_delta = tuple(positive)
        self.display_delta_source = tuple(display_source)
        self.active_mask = tuple(active)
        return self.display_delta_source

    def _render_delta_heatmap(self, snapshot: TactileSnapshot, display_values: Sequence[float]) -> None:
        if self._delta_grid_shape != (snapshot.rows, snapshot.cols):
            self._build_delta_grid(snapshot.rows, snapshot.cols)

        self.delta_heatmap_canvas.delete("delta_status")
        delta_vmax = self._numeric(
            self.delta_vmax_var, self.DEFAULT_DELTA_VMAX, self.MIN_DELTA_VMAX
        )
        active = self.active_mask or tuple(False for _ in display_values)
        for item, value, is_active in zip(self._delta_cell_items, display_values, active):
            self.delta_heatmap_canvas.itemconfigure(
                item, fill=self._delta_display_color(value, is_active, delta_vmax)
            )

        # Delta statistics are intentionally from unclipped, unthresholded
        # positive_delta, not from display_values or the color scale.
        positive = self.positive_delta or ()
        finite = [(index, value) for index, value in enumerate(positive) if math.isfinite(value)]
        if not finite:
            self.delta_stats_var.set("Delta 无有效 taxel：当前 raw 或 baseline 全部无效。")
            return

        max_index, max_value = max(finite, key=lambda item: item[1])
        max_row, max_col = divmod(max_index, snapshot.cols)
        mean_positive = sum(value for _, value in finite) / len(finite)
        threshold = self._numeric(
            self.delta_threshold_var, self.DEFAULT_DELTA_THRESHOLD, 0.0
        )
        self.delta_stats_var.set(
            f"delta max = {max_value:.5f} @ ({max_row}, {max_col})；"
            f"delta mean = {mean_positive:.5f}\n"
            f"有效 taxels={len(finite)}；visual threshold={threshold:.5f}；"
            f"Delta vmax={delta_vmax:.5f}（统计未裁剪）"
        )

    def _update_temporal_values(self, snapshot: TactileSnapshot) -> Optional[Tuple[float, ...]]:
        """Compute raw_t - raw_(t-1) once per new SDK frame, without filtering."""
        current_raw = tuple(snapshot.pressure)
        if self._last_temporal_sequence == snapshot.sequence:
            return self.temporal_positive_delta

        previous_raw = self.prev_raw
        if previous_raw is None or len(previous_raw) != len(current_raw):
            # First compatible frame: retain it only.  A synthetic zero previous
            # frame would incorrectly create an all-positive temporal map.
            self.prev_raw = current_raw
            self.temporal_valid_mask = tuple(False for _ in current_raw)
            self.temporal_signed_delta = None
            self.temporal_positive_delta = None
            self._last_temporal_sequence = snapshot.sequence
            return None

        signed: list[float] = []
        positive: list[float] = []
        temporal_valid: list[bool] = []
        for current_value, previous_value in zip(current_raw, previous_raw):
            is_valid = math.isfinite(current_value) and math.isfinite(previous_value)
            temporal_valid.append(is_valid)
            if not is_valid:
                signed.append(float("nan"))
                positive.append(float("nan"))
                continue
            value = current_value - previous_value
            signed.append(value)
            positive.append(max(value, 0.0))

        self.temporal_valid_mask = tuple(temporal_valid)
        self.temporal_signed_delta = tuple(signed)
        self.temporal_positive_delta = tuple(positive)
        # Update only after this frame's temporal difference is fully computed.
        self.prev_raw = current_raw
        self._last_temporal_sequence = snapshot.sequence
        return self.temporal_positive_delta

    def _render_temporal_heatmap(
        self, snapshot: TactileSnapshot, temporal_positive: Sequence[float]
    ) -> None:
        if self._temporal_grid_shape != (snapshot.rows, snapshot.cols):
            self._build_temporal_grid(snapshot.rows, snapshot.cols)

        self.temporal_heatmap_canvas.delete("temporal_status")
        temporal_vmax = self._numeric(
            self.temporal_vmax_var, self.DEFAULT_TEMPORAL_VMAX, self.MIN_TEMPORAL_VMAX
        )
        for item, value in zip(self._temporal_cell_items, temporal_positive):
            # _inferno_color does only local drawing-stage clipping. There is no
            # temporal threshold and no write-back into temporal_positive_delta.
            self.temporal_heatmap_canvas.itemconfigure(
                item, fill=self._inferno_color(value, temporal_vmax)
            )

        finite_positive = [
            (index, value) for index, value in enumerate(temporal_positive) if math.isfinite(value)
        ]
        signed = self.temporal_signed_delta or ()
        finite_signed = [value for value in signed if math.isfinite(value)]
        if not finite_positive or not finite_signed:
            self.temporal_stats_var.set(
                "Temporal Delta 无有效 taxel：当前或上一 raw frame 均需为 finite。"
            )
            return

        max_index, positive_max = max(finite_positive, key=lambda item: item[1])
        max_row, max_col = divmod(max_index, snapshot.cols)
        positive_mean = sum(value for _, value in finite_positive) / len(finite_positive)
        signed_negative_min = min(finite_signed)
        self.temporal_stats_var.set(
            f"temporal +max = {positive_max:.5f} @ ({max_row}, {max_col})；"
            f"+mean = {positive_mean:.5f}\n"
            f"signed negative min = {signed_negative_min:.5f}；有效 taxels={len(finite_positive)}；"
            f"Temporal vmax={temporal_vmax:.5f}（统计未裁剪、无 threshold）"
        )

    def _draw_plot(self, now: float) -> None:
        canvas = self.plot_canvas
        canvas.delete("all")
        left, top, right, bottom = 45, 18, self.PLOT_WIDTH - 15, self.PLOT_HEIGHT - 30
        canvas.create_rectangle(left, top, right, bottom, outline="#64748b")
        scale_max = self._numeric(self.raw_vmax_var, self.DEFAULT_RAW_VMAX, 1e-6)
        window_s = self._numeric(self.window_s_var, 10.0, 0.5)
        start = now - window_s
        samples = [sample for sample in self._history if sample[0] >= start]

        for fraction, label in ((0.0, "0"), (0.5, f"{scale_max * 0.5:.2f}"), (1.0, f"{scale_max:.2f}")):
            y = bottom - fraction * (bottom - top)
            canvas.create_line(left, y, right, y, fill="#273244", dash=(2, 2))
            canvas.create_text(left - 6, y, text=label, anchor=tk.E, fill="#cbd5e1", font=("TkDefaultFont", 8))
        canvas.create_text(left, bottom + 14, text=f"-{window_s:g}s", anchor=tk.W, fill="#cbd5e1")
        canvas.create_text(right, bottom + 14, text="now", anchor=tk.E, fill="#cbd5e1")

        def points(value_index: int) -> list[float]:
            result: list[float] = []
            for timestamp, maximum, mean in samples:
                value = maximum if value_index == 1 else mean
                x = left + (timestamp - start) / window_s * (right - left)
                y = bottom - max(0.0, min(1.0, value / scale_max)) * (bottom - top)
                result.extend((x, y))
            return result

        max_points = points(1)
        mean_points = points(2)
        if len(max_points) >= 4:
            canvas.create_line(*max_points, fill="#ef4444", width=2)
        if len(mean_points) >= 4:
            canvas.create_line(*mean_points, fill="#22d3ee", width=2)
        canvas.create_text(left + 4, top + 8, text="max", anchor=tk.W, fill="#ef4444")
        canvas.create_text(left + 44, top + 8, text="mean", anchor=tk.W, fill="#22d3ee")

    def _render_snapshot(self, snapshot: TactileSnapshot) -> None:
        self.latest_snapshot = snapshot
        self.raw_pressure = tuple(snapshot.pressure)
        self.valid_mask = tuple(math.isfinite(raw) for raw in self.raw_pressure)
        raw_finite = [
            (index, raw) for index, raw in enumerate(self.raw_pressure) if self.valid_mask[index]
        ]
        raw_max, raw_mean = self._render_raw_heatmap(snapshot, self.raw_pressure)
        if raw_finite:
            max_index, max_value = max(raw_finite, key=lambda item: item[1])
            max_row, max_col = divmod(max_index, snapshot.cols)
        else:
            max_value = float("nan")
            max_row = max_col = -1
        raw_vmax = self._numeric(self.raw_vmax_var, self.DEFAULT_RAW_VMAX, 1e-6)
        self.layout_var.set(
            f"布局：{snapshot.rows}×{snapshot.cols}，{len(snapshot.pressure)} values，"
            f"seq={snapshot.sequence}，device timestamp={snapshot.timestamp_ms} ms，"
            f"handedness={'left' if snapshot.handedness == 0 else 'right'}，"
            f"finite raw taxels={len(raw_finite)}"
        )
        self.stats_var.set(
            f"raw max = {raw_max:.5f} @ ({max_row}, {max_col})；raw mean = {raw_mean:.5f}\n"
            f"有效且 finite raw taxels={len(raw_finite)}；Raw 固定范围=[{self.RAW_VMIN:.2f}, {raw_vmax:.5f}]；"
            f"最大值={max_value:.5f}"
        )
        if self._last_history_sequence != snapshot.sequence:
            if raw_finite:
                self._history.append((time.monotonic(), raw_max, raw_mean))
            self._last_history_sequence = snapshot.sequence

        self._advance_delta_baseline_capture(snapshot)
        display_delta_source = self._update_delta_values(snapshot)
        if display_delta_source is None:
            self._render_delta_uninitialized(snapshot)
        else:
            self._render_delta_heatmap(snapshot, display_delta_source)

        temporal_positive = self._update_temporal_values(snapshot)
        if temporal_positive is None:
            self._render_temporal_waiting(snapshot)
        else:
            self._render_temporal_heatmap(snapshot, temporal_positive)

    def _clear_delta_capture(self) -> None:
        self._delta_capture_started_at = None
        self._delta_capture_first_sample_at = None
        self._delta_capture_shape = None
        self._delta_capture_samples = None
        self._delta_capture_last_sequence = None
        self._delta_capture_frame_count = 0

    def _capture_delta_baseline(self) -> None:
        """Start a fresh 2-second per-taxel median baseline capture."""
        snapshot = self.latest_snapshot
        if snapshot is None:
            self.delta_state_var.set("尚未收到 tactile frame，不能开始 Delta baseline 采集。")
            self.delta_stats_var.set("Delta 未初始化：等待有效 tactile frame。")
            return

        # A partially captured baseline is intentionally unusable. Delta stays
        # uninitialized until the complete capture below has finished.
        self.baseline = None
        self.baseline_valid = None
        self.baseline_sample_counts = None
        self.signed_delta = None
        self.positive_delta = None
        self.display_delta_source = None
        self.active_mask = None
        self._delta_capture_started_at = time.monotonic()
        self._delta_capture_first_sample_at = None
        self._delta_capture_shape = (snapshot.rows, snapshot.cols)
        self._delta_capture_samples = [[] for _ in snapshot.pressure]
        # Do not include the pre-button cached frame; wait for a new frame.
        self._delta_capture_last_sequence = snapshot.sequence
        self._delta_capture_frame_count = 0
        self.delta_state_var.set(
            "正在等待新的 tactile frame 以开始 2 秒 median baseline；请保持无外力。"
        )
        self.delta_stats_var.set("Delta 未初始化：正在等待 baseline 样本。")

    def _reset_delta_baseline(self) -> None:
        self.baseline = None
        self.baseline_valid = None
        self.baseline_sample_counts = None
        self.signed_delta = None
        self.positive_delta = None
        self.display_delta_source = None
        self.active_mask = None
        self._clear_delta_capture()
        self.delta_state_var.set("Delta baseline 已清除；请在无外力时点击 Capture Baseline。")
        self.delta_stats_var.set("Delta 未初始化：未使用全 0 baseline。")

    def _advance_delta_baseline_capture(self, snapshot: TactileSnapshot) -> None:
        """Accumulate fresh frames and finalize exactly after about two seconds."""
        started = self._delta_capture_started_at
        samples = self._delta_capture_samples
        capture_shape = self._delta_capture_shape
        if started is None or samples is None or capture_shape is None:
            return

        if capture_shape != (snapshot.rows, snapshot.cols) or len(samples) != len(snapshot.pressure):
            self._clear_delta_capture()
            self.delta_state_var.set(
                "Delta baseline 采集已取消：tactile 布局在采集期间改变；请重新 Capture Baseline。"
            )
            self.delta_stats_var.set("Delta 未初始化：布局改变后没有沿用旧 baseline。")
            return

        if snapshot.sequence == self._delta_capture_last_sequence:
            return

        self._delta_capture_last_sequence = snapshot.sequence
        now = time.monotonic()
        if self._delta_capture_first_sample_at is None:
            self._delta_capture_first_sample_at = now
        for index, raw in enumerate(snapshot.pressure):
            if math.isfinite(raw):
                samples[index].append(raw)
        self._delta_capture_frame_count += 1

        elapsed = now - self._delta_capture_first_sample_at
        if elapsed < 2.0:
            self.delta_state_var.set(
                f"正在采集 Delta baseline：{elapsed:.1f}/2.0 s，"
                f"{self._delta_capture_frame_count} 个新 frame；请保持无外力。"
            )
            return

        sample_counts = tuple(len(taxel_samples) for taxel_samples in samples)
        baseline_valid = tuple(
            sample_count >= self.MIN_BASELINE_SAMPLES for sample_count in sample_counts
        )
        baseline = tuple(
            statistics.median(taxel_samples) if is_valid else float("nan")
            for taxel_samples, is_valid in zip(samples, baseline_valid)
        )
        valid_count = sum(baseline_valid)
        frame_count = self._delta_capture_frame_count
        self.baseline = baseline
        self.baseline_valid = baseline_valid
        self.baseline_sample_counts = sample_counts
        self._clear_delta_capture()
        self.delta_state_var.set(
            f"Delta baseline 已完成：2.0 s median，{frame_count} 个 frame，"
            f"{valid_count}/{len(baseline)} 个有效 taxel（每个至少 {self.MIN_BASELINE_SAMPLES} 个 finite 样本）。"
        )

    def _refresh(self) -> None:
        snapshot = self.node.latest()
        if snapshot is not None:
            self._render_snapshot(snapshot)
        self._draw_plot(time.monotonic())
        self.root.after(33, self._refresh)

    def _on_close(self) -> None:
        if messagebox.askyesno("关闭 tactile 可视化", "关闭窗口只会停止本地 ROS 订阅，不会向 Wuji Hand 发送任何命令。是否关闭？"):
            self.root.destroy()


def main(argv: Optional[Sequence[str]] = None) -> None:
    parser = argparse.ArgumentParser(description="Read-only Wuji paired tactile heatmap")
    parser.add_argument("--hand-name", default="hand_0", help="ROS namespace without a leading slash")
    parsed, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = TactileMonitorNode(parsed.hand_name)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, name="wuji-tactile-heatmap-ros", daemon=True)
    spin_thread.start()

    root = tk.Tk()
    TactileHeatmapApp(root, node)
    try:
        root.mainloop()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
