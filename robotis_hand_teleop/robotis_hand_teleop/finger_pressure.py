#!/usr/bin/env python3
from __future__ import annotations
import threading
from dataclasses import dataclass
from typing import Dict, List, Tuple
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from control_msgs.msg import DynamicJointState
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# =========================
# Config
# =========================
@dataclass(frozen=True)
class VizConfig:
    topic: str
    sensor_prefix: str
    num_fingers: int
    num_taxels: int
    iface_prefix: str
    update_hz: float
    use_best_effort: bool

    # Filtering / baseline
    baseline_seconds: float
    ema_alpha: float
    deadband: float
    clip_negative: bool

    # Visualization
    viz_gain: float
    z_fixed_max: float
    z_tick_interval: int

def _make_qos(best_effort: bool) -> QoSProfile:
    if best_effort:
        return QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
        )
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        durability=DurabilityPolicy.VOLATILE,
    )

def _grid3x3_xy(dx: float = 0.8, dy: float = 0.8) -> Tuple[np.ndarray, np.ndarray, float, float]:
    """Return (x, y) coordinates for 3×3 bars, flattened."""
    xs, ys = np.meshgrid(np.arange(3), np.arange(3), indexing="xy")  # col, row
    return xs.ravel(), ys.ravel(), dx, dy

# Node
# =========================
class PressureViz(Node):
    def __init__(self) -> None:
        super().__init__("pressure_viz")

        self.cfg = self._declare_and_read_params()

        if self.cfg.num_taxels != 9:
            self.get_logger().warn(
                f"num_taxels={self.cfg.num_taxels}. This visualizer assumes 9 taxels (3×3)."
            )

        self._lock = threading.Lock()

        self._pressure = np.zeros((self.cfg.num_fingers, self.cfg.num_taxels), dtype=float)
        self._ema = np.zeros_like(self._pressure)

        self._baseline = np.zeros_like(self._pressure)
        self._baseline_sum = np.zeros_like(self._pressure)
        self._baseline_count = 0
        self._baseline_ready = False
        self._baseline_frames = max(1, int(self.cfg.update_hz * self.cfg.baseline_seconds))

        # --- ROS subscription ---
        qos = _make_qos(self.cfg.use_best_effort)
        self.sub = self.create_subscription(DynamicJointState, self.cfg.topic, self._cb, qos)

        log_msg = (
            f"Subscribing: {self.cfg.topic} | baseline {self.cfg.baseline_seconds:.2f}s (~{self._baseline_frames} frames) | "
            f"gain={self.cfg.viz_gain:.2f} | z=[0,{self.cfg.z_fixed_max:.0f}] tick={self.cfg.z_tick_interval}"
        )
        self.get_logger().info(log_msg)
        self._setup_figure()

        interval_ms = int(1000.0 / max(self.cfg.update_hz, 1.0))
        self.ani = FuncAnimation(self.fig, self._update_plot, interval=interval_ms)

    # -------------------------
    # Params
    # -------------------------
    def _declare_and_read_params(self) -> VizConfig:
        self.declare_parameter("topic", "/dynamic_joint_states")
        self.declare_parameter("sensor_prefix", "finger_r_sensor")
        self.declare_parameter("num_fingers", 5)
        self.declare_parameter("num_taxels", 9)
        self.declare_parameter("pressure_iface_prefix", "Present Pressure")
        self.declare_parameter("update_hz", 20.0)
        self.declare_parameter("use_best_effort", True)

        self.declare_parameter("baseline_seconds", 1.0)
        self.declare_parameter("ema_alpha", 0.25)
        self.declare_parameter("deadband", 1.0)
        self.declare_parameter("clip_negative", True)

        self.declare_parameter("viz_gain", 10.0)

        self.declare_parameter("z_fixed_max", 2500.0)
        self.declare_parameter("z_tick_interval", 500)

        return VizConfig(
            topic=str(self.get_parameter("topic").value),
            sensor_prefix=str(self.get_parameter("sensor_prefix").value),
            num_fingers=int(self.get_parameter("num_fingers").value),
            num_taxels=int(self.get_parameter("num_taxels").value),
            iface_prefix=str(self.get_parameter("pressure_iface_prefix").value),
            update_hz=float(self.get_parameter("update_hz").value),
            use_best_effort=bool(self.get_parameter("use_best_effort").value),
            baseline_seconds=float(self.get_parameter("baseline_seconds").value),
            ema_alpha=float(self.get_parameter("ema_alpha").value),
            deadband=float(self.get_parameter("deadband").value),
            clip_negative=bool(self.get_parameter("clip_negative").value),
            viz_gain=float(self.get_parameter("viz_gain").value),
            z_fixed_max=float(self.get_parameter("z_fixed_max").value),
            z_tick_interval=int(self.get_parameter("z_tick_interval").value),
        )

    # -------------------------
    # ROS callback / processing
    # -------------------------
    @staticmethod
    def _iface_map(iv) -> Dict[str, float]:
        return {n: float(v) for n, v in zip(iv.interface_names, iv.values)}

    def _finger_index_from_joint(self, joint_name: str) -> int | None:
        if not joint_name.startswith(self.cfg.sensor_prefix):
            return None
        suffix = joint_name[len(self.cfg.sensor_prefix) :]
        if not suffix.isdigit():
            return None
        idx = int(suffix) - 1
        return idx if 0 <= idx < self.cfg.num_fingers else None

    def _cb(self, msg: DynamicJointState) -> None:
        with self._lock:
            for j, joint_name in enumerate(msg.joint_names):
                finger_idx = self._finger_index_from_joint(joint_name)
                if finger_idx is None:
                    continue

                iface = self._iface_map(msg.interface_values[j])

                # _update_pressure(finger_idx, iface)
                if not self._baseline_ready:
                    self._accumulate_baseline(finger_idx, iface)
                else:
                    self._update_pressure(finger_idx, iface)

            if not self._baseline_ready:
                self._baseline_count += 1
                if self._baseline_count >= self._baseline_frames:
                    self._finalize_baseline()

    def _accumulate_baseline(self, finger_idx: int, iface: Dict[str, float]) -> None:
        for t in range(1, self.cfg.num_taxels + 1):
            key = f"{self.cfg.iface_prefix} {t}"
            val = iface.get(key)
            if val is not None:
                self._baseline_sum[finger_idx, t - 1] += val

    def _update_pressure(self, finger_idx: int, iface: Dict[str, float]) -> None:
        for t in range(1, self.cfg.num_taxels + 1):
            key = f"{self.cfg.iface_prefix} {t}"
            raw = iface.get(key)
            if raw is None:
                continue

            val = raw - self._baseline[finger_idx, t - 1]

            if self.cfg.clip_negative and val < 0.0:
                val = 0.0
            if val < self.cfg.deadband:
                val = 0.0

            prev = self._ema[finger_idx, t - 1]
            self._ema[finger_idx, t - 1] = (1.0 - self.cfg.ema_alpha) * prev + self.cfg.ema_alpha * val
            self._pressure[finger_idx, t - 1] = self._ema[finger_idx, t - 1]

    def _finalize_baseline(self) -> None:
        self._baseline = self._baseline_sum / float(self._baseline_count)
        self._baseline_ready = True
        self._ema.fill(0.0)
        self._pressure.fill(0.0)
        self.get_logger().info("Baseline calibrated. Visualizing baseline-subtracted pressure.")

    # -------------------------
    # Plotting
    # -------------------------
    def _setup_figure(self) -> None:
        self.fig = plt.figure(figsize=(20, 4))
        gs = self.fig.add_gridspec(1, self.cfg.num_fingers)

        self.axes = [self.fig.add_subplot(gs[0, i], projection="3d") for i in range(self.cfg.num_fingers)]

        self._grid_x, self._grid_y, self._dx, self._dy = _grid3x3_xy(dx=0.8, dy=0.8)
        self._z_top = self.cfg.z_fixed_max
        self._z_ticks = np.arange(0, int(self._z_top) + 1, self.cfg.z_tick_interval, dtype=int)

    @staticmethod
    def _taxels_to_3x3(taxels: np.ndarray) -> np.ndarray:
        """taxel 1..9 -> 3x3 row-major."""
        if taxels.size != 9:
            out = np.zeros((3, 3), dtype=float)
            n = min(taxels.size, 9)
            out.ravel()[:n] = taxels[:n]
            return out
        return taxels.reshape(3, 3)

    def _style_axis(self, ax, finger_idx: int) -> None:
        ax.set_title(f"Finger {finger_idx + 1}")
        ax.set_xlabel("")
        ax.set_ylabel("")
        ax.set_zlabel("Pressure")

        ax.set_xlim(-0.2, 2.8)
        ax.set_ylim(-0.2, 2.8)
        ax.set_zlim(0.0, self._z_top)

        ax.set_xticks([0, 1, 2])
        ax.set_xticklabels(["1", "2", "3"])
        ax.set_yticks([0, 1, 2])
        ax.set_yticklabels(["1", "2", "3"])
        ax.set_zticks(self._z_ticks)

        ax.invert_yaxis()
        ax.tick_params(axis="x", which="both", labelbottom=False)
        ax.tick_params(axis="y", which="both", labelleft=False)

    def _update_plot(self, _frame) -> None:
        with self._lock:
            data = self._pressure.copy()

        data_viz = data * self.cfg.viz_gain

        for f, ax in enumerate(self.axes):
            ax.cla()

            grid = self._taxels_to_3x3(data_viz[f, :])
            dz = np.clip(grid.ravel(), 0.0, self._z_top)

            self._style_axis(ax, f)
            ax.bar3d(
                self._grid_x,
                self._grid_y,
                np.zeros_like(self._grid_x, dtype=float),
                self._dx,
                self._dy,
                dz,
                shade=True,
            )
        self.fig.subplots_adjust(wspace=0.25)

def main() -> None:
    rclpy.init()
    node = PressureViz()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()