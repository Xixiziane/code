#!/usr/bin/env python3
# ============================================================================
# analyze_param_sweep.py — ChainWing 参数扫描日志分析与对比图生成
# ============================================================================
#
# 功能:
#   1. 读取 sweep_logs/ 下所有 .ulg 文件
#   2. 提取姿态跟踪、角速率、舵面输出、铰链状态
#   3. 计算性能指标 (RMS误差、超调量、振荡频率)
#   4. 生成多轮对比图 (PNG + PDF)
#
# 用法:
#   python3 scripts/analyze_param_sweep.py sweep_logs/
#   python3 scripts/analyze_param_sweep.py sweep_logs/ --output results/
#   python3 scripts/analyze_param_sweep.py sweep_logs/ --pdf
#
# 依赖: pip3 install pyulog matplotlib numpy
# ============================================================================

import argparse
import glob
import json
import os
import sys
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

try:
    from pyulog import ULog
except ImportError:
    print("Error: pyulog not installed. Run: pip3 install pyulog")
    sys.exit(1)

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_pdf import PdfPages
except ImportError:
    print("Error: matplotlib not installed. Run: pip3 install matplotlib")
    sys.exit(1)


# ── 数据结构 ────────────────────────────────────────────────────────────────

@dataclass
class FlightMetrics:
    """单次飞行的性能指标"""
    name: str = ""
    description: str = ""
    params: Dict[str, float] = field(default_factory=dict)
    # 跟踪误差 (RMS, deg)
    roll_rms_error: float = 0.0
    pitch_rms_error: float = 0.0
    yaw_rate_rms_error: float = 0.0
    # 最大偏差 (deg)
    roll_max_error: float = 0.0
    pitch_max_error: float = 0.0
    yaw_rate_max_error: float = 0.0
    # 振荡计数 (零交叉数 / 时间)
    roll_oscillation_freq: float = 0.0
    yaw_rate_oscillation_freq: float = 0.0
    # 铰链
    hinge_angle_rms: float = 0.0
    hinge_trim_rms: float = 0.0
    # 舵面活动量
    servo_activity: float = 0.0
    # 飞行时长
    flight_duration_sec: float = 0.0


@dataclass
class FlightData:
    """单次飞行的时序数据"""
    name: str = ""
    t_att: np.ndarray = field(default_factory=lambda: np.array([]))
    roll: np.ndarray = field(default_factory=lambda: np.array([]))
    pitch: np.ndarray = field(default_factory=lambda: np.array([]))
    yaw: np.ndarray = field(default_factory=lambda: np.array([]))
    roll_sp: np.ndarray = field(default_factory=lambda: np.array([]))
    pitch_sp: np.ndarray = field(default_factory=lambda: np.array([]))

    t_rate: np.ndarray = field(default_factory=lambda: np.array([]))
    roll_rate: np.ndarray = field(default_factory=lambda: np.array([]))
    pitch_rate: np.ndarray = field(default_factory=lambda: np.array([]))
    yaw_rate: np.ndarray = field(default_factory=lambda: np.array([]))

    t_rate_sp: np.ndarray = field(default_factory=lambda: np.array([]))
    roll_rate_sp: np.ndarray = field(default_factory=lambda: np.array([]))
    pitch_rate_sp: np.ndarray = field(default_factory=lambda: np.array([]))
    yaw_rate_sp: np.ndarray = field(default_factory=lambda: np.array([]))

    t_servo: np.ndarray = field(default_factory=lambda: np.array([]))
    servo_0: np.ndarray = field(default_factory=lambda: np.array([]))
    servo_1: np.ndarray = field(default_factory=lambda: np.array([]))
    servo_2: np.ndarray = field(default_factory=lambda: np.array([]))

    t_hinge: np.ndarray = field(default_factory=lambda: np.array([]))
    hinge_angle: np.ndarray = field(default_factory=lambda: np.array([]))
    hinge_trim_left: np.ndarray = field(default_factory=lambda: np.array([]))
    hinge_trim_right: np.ndarray = field(default_factory=lambda: np.array([]))

    metrics: FlightMetrics = field(default_factory=FlightMetrics)


# ── ULog 数据提取 ───────────────────────────────────────────────────────────

def quat_to_euler(q0, q1, q2, q3):
    """四元数 → 欧拉角 (roll, pitch, yaw) in degrees"""
    # Roll (x-axis rotation)
    sinr = 2.0 * (q0 * q1 + q2 * q3)
    cosr = 1.0 - 2.0 * (q1 * q1 + q2 * q2)
    roll = np.arctan2(sinr, cosr)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (q0 * q2 - q3 * q1)
    sinp = np.clip(sinp, -1.0, 1.0)
    pitch = np.arcsin(sinp)

    # Yaw (z-axis rotation)
    siny = 2.0 * (q0 * q3 + q1 * q2)
    cosy = 1.0 - 2.0 * (q2 * q2 + q3 * q3)
    yaw = np.arctan2(siny, cosy)

    return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)


def find_topic(ulog: ULog, topic_name: str) -> Optional[object]:
    """在 ULog 中查找指定 topic"""
    for d in ulog.data_list:
        if d.name == topic_name:
            return d
    return None


def extract_flight_data(ulg_path: str, name: str = "") -> Optional[FlightData]:
    """从 .ulg 文件提取飞行数据"""
    try:
        ulog = ULog(ulg_path)
    except Exception as e:
        print(f"  Warning: Cannot parse {ulg_path}: {e}")
        return None

    fd = FlightData(name=name)

    # ── 姿态 (vehicle_attitude) ──
    att = find_topic(ulog, 'vehicle_attitude')
    if att is not None:
        t = att.data['timestamp'] / 1e6  # μs → s
        fd.t_att = t - t[0]
        fd.roll, fd.pitch, fd.yaw = quat_to_euler(
            att.data['q[0]'], att.data['q[1]'],
            att.data['q[2]'], att.data['q[3]'])

    # ── 姿态设定值 (vehicle_attitude_setpoint) ──
    att_sp = find_topic(ulog, 'vehicle_attitude_setpoint')
    if att_sp is not None:
        fd.roll_sp = np.degrees(att_sp.data.get('roll_body', np.zeros(1)))
        fd.pitch_sp = np.degrees(att_sp.data.get('pitch_body', np.zeros(1)))

    # ── 角速率 (vehicle_angular_velocity) ──
    ang_vel = find_topic(ulog, 'vehicle_angular_velocity')
    if ang_vel is not None:
        t = ang_vel.data['timestamp'] / 1e6
        fd.t_rate = t - t[0]
        fd.roll_rate = np.degrees(ang_vel.data['xyz[0]'])
        fd.pitch_rate = np.degrees(ang_vel.data['xyz[1]'])
        fd.yaw_rate = np.degrees(ang_vel.data['xyz[2]'])

    # ── 角速率设定值 (vehicle_rates_setpoint) ──
    rate_sp = find_topic(ulog, 'vehicle_rates_setpoint')
    if rate_sp is not None:
        t = rate_sp.data['timestamp'] / 1e6
        fd.t_rate_sp = t - t[0]
        fd.roll_rate_sp = np.degrees(rate_sp.data['roll'])
        fd.pitch_rate_sp = np.degrees(rate_sp.data['pitch'])
        fd.yaw_rate_sp = np.degrees(rate_sp.data['yaw'])

    # ── 舵面输出 (actuator_servos) ──
    servos = find_topic(ulog, 'actuator_servos')
    if servos is not None:
        t = servos.data['timestamp'] / 1e6
        fd.t_servo = t - t[0]
        fd.servo_0 = servos.data.get('control[0]', np.zeros(len(t)))
        fd.servo_1 = servos.data.get('control[1]', np.zeros(len(t)))
        fd.servo_2 = servos.data.get('control[2]', np.zeros(len(t)))

    # ── 铰链状态 (chainwing_hinge_status) ──
    hinge = find_topic(ulog, 'chainwing_hinge_status')
    if hinge is not None:
        t = hinge.data['timestamp'] / 1e6
        fd.t_hinge = t - t[0]
        fd.hinge_angle = np.degrees(
            hinge.data.get('estimated_angle', np.zeros(len(t))))
        fd.hinge_trim_left = hinge.data.get('trim_left', np.zeros(len(t)))
        fd.hinge_trim_right = hinge.data.get('trim_right', np.zeros(len(t)))

    return fd


# ── 性能指标计算 ────────────────────────────────────────────────────────────

def compute_zero_crossings(signal: np.ndarray, dt: float) -> float:
    """计算信号零交叉频率 (Hz)"""
    if len(signal) < 2 or dt <= 0:
        return 0.0
    zero_crossings = np.sum(np.diff(np.sign(signal)) != 0)
    total_time = len(signal) * dt
    return zero_crossings / (2.0 * total_time) if total_time > 0 else 0.0


def compute_metrics(fd: FlightData) -> FlightMetrics:
    """从飞行数据计算性能指标"""
    m = FlightMetrics(name=fd.name)

    # 飞行时长
    if len(fd.t_att) > 0:
        m.flight_duration_sec = fd.t_att[-1]

    # 跳过起飞阶段 (前 15 秒)
    SKIP_SEC = 15.0

    # ── Roll 跟踪误差 ──
    if len(fd.roll) > 0 and len(fd.roll_sp) > 0:
        # 插值到相同时间轴
        mask = fd.t_att > SKIP_SEC
        if np.any(mask):
            roll_active = fd.roll[mask]
            if len(fd.roll_sp) == len(fd.roll):
                sp_active = fd.roll_sp[mask]
            else:
                sp_active = np.interp(fd.t_att[mask],
                                      np.linspace(0, fd.t_att[-1], len(fd.roll_sp)),
                                      fd.roll_sp)
            error = roll_active - sp_active
            m.roll_rms_error = float(np.sqrt(np.mean(error ** 2)))
            m.roll_max_error = float(np.max(np.abs(error)))

            dt = np.mean(np.diff(fd.t_att[mask])) if np.sum(mask) > 1 else 0.01
            m.roll_oscillation_freq = compute_zero_crossings(error, dt)

    # ── Pitch 跟踪误差 ──
    if len(fd.pitch) > 0 and len(fd.pitch_sp) > 0:
        mask = fd.t_att > SKIP_SEC
        if np.any(mask):
            pitch_active = fd.pitch[mask]
            if len(fd.pitch_sp) == len(fd.pitch):
                sp_active = fd.pitch_sp[mask]
            else:
                sp_active = np.interp(fd.t_att[mask],
                                      np.linspace(0, fd.t_att[-1], len(fd.pitch_sp)),
                                      fd.pitch_sp)
            error = pitch_active - sp_active
            m.pitch_rms_error = float(np.sqrt(np.mean(error ** 2)))
            m.pitch_max_error = float(np.max(np.abs(error)))

    # ── Yaw rate 跟踪误差 ──
    if len(fd.yaw_rate) > 0 and len(fd.yaw_rate_sp) > 0:
        mask = fd.t_rate > SKIP_SEC
        if np.any(mask):
            yr_active = fd.yaw_rate[mask]
            # 将设定值插值到实际角速率的时间轴
            t_active = fd.t_rate[mask]
            if len(fd.t_rate_sp) > 0:
                sp_mask = fd.t_rate_sp > SKIP_SEC
                if np.any(sp_mask):
                    sp_active = np.interp(t_active,
                                          fd.t_rate_sp[sp_mask],
                                          fd.yaw_rate_sp[sp_mask])
                else:
                    sp_active = np.zeros_like(yr_active)
            else:
                sp_active = np.zeros_like(yr_active)
            error = yr_active - sp_active
            m.yaw_rate_rms_error = float(np.sqrt(np.mean(error ** 2)))
            m.yaw_rate_max_error = float(np.max(np.abs(error)))

            dt = np.mean(np.diff(fd.t_rate[mask])) if np.sum(mask) > 1 else 0.01
            m.yaw_rate_oscillation_freq = compute_zero_crossings(error, dt)

    # ── 铰链指标 ──
    if len(fd.hinge_angle) > 0:
        mask = fd.t_hinge > SKIP_SEC
        if np.any(mask):
            m.hinge_angle_rms = float(np.sqrt(np.mean(fd.hinge_angle[mask] ** 2)))
            m.hinge_trim_rms = float(np.sqrt(np.mean(fd.hinge_trim_left[mask] ** 2)))

    # ── 舵面活动量 ──
    if len(fd.servo_0) > 1:
        mask = fd.t_servo > SKIP_SEC
        if np.any(mask):
            activity = (np.sum(np.abs(np.diff(fd.servo_0[mask]))) +
                        np.sum(np.abs(np.diff(fd.servo_2[mask]))))
            duration = fd.t_servo[mask][-1] - fd.t_servo[mask][0]
            m.servo_activity = float(activity / duration) if duration > 0 else 0.0

    fd.metrics = m
    return m


# ── 图表生成 ────────────────────────────────────────────────────────────────

# 字体配置: 优先使用已安装的字体
_preferred_fonts = ['DejaVu Sans', 'SimHei', 'Microsoft YaHei', 'WenQuanYi Zen Hei']
_available = matplotlib.font_manager.findSystemFonts()
plt.rcParams['font.sans-serif'] = _preferred_fonts
plt.rcParams['axes.unicode_minus'] = False

COLORS = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd',
          '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']


def plot_attitude_comparison(flights: List[FlightData], output_dir: str):
    """图1: 多轮姿态对比 (Roll + Pitch + Yaw)"""
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    fig.suptitle('Attitude Tracking Comparison', fontsize=14, fontweight='bold')

    for i, fd in enumerate(flights):
        color = COLORS[i % len(COLORS)]
        if len(fd.t_att) > 0:
            axes[0].plot(fd.t_att, fd.roll, color=color, alpha=0.7,
                         label=fd.name, linewidth=0.8)
            axes[1].plot(fd.t_att, fd.pitch, color=color, alpha=0.7,
                         linewidth=0.8)
            axes[2].plot(fd.t_att, fd.yaw, color=color, alpha=0.7,
                         linewidth=0.8)

    axes[0].set_ylabel('Roll (deg)')
    axes[0].legend(fontsize=7, ncol=3, loc='upper right')
    axes[0].grid(True, alpha=0.3)
    axes[1].set_ylabel('Pitch (deg)')
    axes[1].grid(True, alpha=0.3)
    axes[2].set_ylabel('Yaw (deg)')
    axes[2].set_xlabel('Time (s)')
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    path = os.path.join(output_dir, '01_attitude_comparison.png')
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"  Saved: {path}")
    return fig


def plot_yaw_rate_comparison(flights: List[FlightData], output_dir: str):
    """图2: 偏航角速率跟踪对比"""
    n = len(flights)
    cols = min(n, 3)
    rows = (n + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(5 * cols, 3.5 * rows), squeeze=False)
    fig.suptitle('Yaw Rate Tracking per Run', fontsize=14, fontweight='bold')

    for i, fd in enumerate(flights):
        ax = axes[i // cols][i % cols]
        if len(fd.t_rate) > 0:
            ax.plot(fd.t_rate, fd.yaw_rate, 'b-', alpha=0.5, linewidth=0.6,
                    label='Actual')
        if len(fd.t_rate_sp) > 0:
            ax.plot(fd.t_rate_sp, fd.yaw_rate_sp, 'r--', alpha=0.7,
                    linewidth=0.8, label='Setpoint')
        ax.set_title(fd.name, fontsize=9)
        ax.set_ylabel('deg/s')
        ax.grid(True, alpha=0.3)
        if i == 0:
            ax.legend(fontsize=7)

    # 隐藏多余子图
    for i in range(n, rows * cols):
        axes[i // cols][i % cols].set_visible(False)

    plt.tight_layout()
    path = os.path.join(output_dir, '02_yaw_rate_tracking.png')
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"  Saved: {path}")
    return fig


def plot_servo_comparison(flights: List[FlightData], output_dir: str):
    """图3: 舵面输出对比"""
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    fig.suptitle('Servo Output Comparison (S0=LeftElevon, S1=Elevator, S2=RightElevon)',
                 fontsize=13, fontweight='bold')

    labels = ['S0 (Left Elevon)', 'S1 (Elevator)', 'S2 (Right Elevon)']
    for i, fd in enumerate(flights):
        color = COLORS[i % len(COLORS)]
        if len(fd.t_servo) > 0:
            axes[0].plot(fd.t_servo, fd.servo_0, color=color, alpha=0.6,
                         label=fd.name, linewidth=0.6)
            axes[1].plot(fd.t_servo, fd.servo_1, color=color, alpha=0.6,
                         linewidth=0.6)
            axes[2].plot(fd.t_servo, fd.servo_2, color=color, alpha=0.6,
                         linewidth=0.6)

    for j in range(3):
        axes[j].set_ylabel(labels[j])
        axes[j].grid(True, alpha=0.3)
        axes[j].set_ylim(-1.1, 1.1)
    axes[0].legend(fontsize=7, ncol=3, loc='upper right')
    axes[2].set_xlabel('Time (s)')

    plt.tight_layout()
    path = os.path.join(output_dir, '03_servo_output.png')
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"  Saved: {path}")
    return fig


def plot_hinge_comparison(flights: List[FlightData], output_dir: str):
    """图4: 铰链角度 + Trim 对比 (仅包含有铰链数据的轮次)"""
    hinge_flights = [fd for fd in flights if len(fd.t_hinge) > 0]
    if not hinge_flights:
        print("  Skip: No hinge data found in any run")
        return None

    fig, axes = plt.subplots(2, 1, figsize=(14, 7), sharex=True)
    fig.suptitle('Hinge Correction Comparison', fontsize=14, fontweight='bold')

    for i, fd in enumerate(hinge_flights):
        color = COLORS[i % len(COLORS)]
        axes[0].plot(fd.t_hinge, fd.hinge_angle, color=color, alpha=0.7,
                     label=fd.name, linewidth=0.8)
        axes[1].plot(fd.t_hinge, fd.hinge_trim_left, color=color, alpha=0.7,
                     linewidth=0.8, linestyle='-')
        axes[1].plot(fd.t_hinge, fd.hinge_trim_right, color=color, alpha=0.5,
                     linewidth=0.8, linestyle='--')

    axes[0].set_ylabel('Hinge Angle (deg)')
    axes[0].legend(fontsize=8)
    axes[0].grid(True, alpha=0.3)
    axes[1].set_ylabel('Trim Output (norm)')
    axes[1].set_xlabel('Time (s)')
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    path = os.path.join(output_dir, '04_hinge_correction.png')
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"  Saved: {path}")
    return fig


def plot_metrics_bar(flights: List[FlightData], output_dir: str):
    """图5: 性能指标柱状图对比"""
    names = [fd.metrics.name for fd in flights]
    x = np.arange(len(names))
    width = 0.65

    fig, axes = plt.subplots(2, 3, figsize=(16, 8))
    fig.suptitle('Performance Metrics Comparison', fontsize=14, fontweight='bold')

    # Roll RMS Error
    vals = [fd.metrics.roll_rms_error for fd in flights]
    axes[0, 0].bar(x, vals, width, color=COLORS[:len(x)])
    axes[0, 0].set_title('Roll RMS Error (deg)')
    axes[0, 0].set_xticks(x)
    axes[0, 0].set_xticklabels(names, rotation=45, ha='right', fontsize=7)

    # Yaw Rate RMS Error
    vals = [fd.metrics.yaw_rate_rms_error for fd in flights]
    axes[0, 1].bar(x, vals, width, color=COLORS[:len(x)])
    axes[0, 1].set_title('Yaw Rate RMS Error (deg/s)')
    axes[0, 1].set_xticks(x)
    axes[0, 1].set_xticklabels(names, rotation=45, ha='right', fontsize=7)

    # Yaw Rate Max Error
    vals = [fd.metrics.yaw_rate_max_error for fd in flights]
    axes[0, 2].bar(x, vals, width, color=COLORS[:len(x)])
    axes[0, 2].set_title('Yaw Rate Max Error (deg/s)')
    axes[0, 2].set_xticks(x)
    axes[0, 2].set_xticklabels(names, rotation=45, ha='right', fontsize=7)

    # Yaw Rate Oscillation Freq
    vals = [fd.metrics.yaw_rate_oscillation_freq for fd in flights]
    axes[1, 0].bar(x, vals, width, color=COLORS[:len(x)])
    axes[1, 0].set_title('Yaw Rate Oscillation (Hz)')
    axes[1, 0].set_xticks(x)
    axes[1, 0].set_xticklabels(names, rotation=45, ha='right', fontsize=7)

    # Servo Activity
    vals = [fd.metrics.servo_activity for fd in flights]
    axes[1, 1].bar(x, vals, width, color=COLORS[:len(x)])
    axes[1, 1].set_title('Servo Activity (Σ|Δ|/s)')
    axes[1, 1].set_xticks(x)
    axes[1, 1].set_xticklabels(names, rotation=45, ha='right', fontsize=7)

    # Hinge Angle RMS
    vals = [fd.metrics.hinge_angle_rms for fd in flights]
    axes[1, 2].bar(x, vals, width, color=COLORS[:len(x)])
    axes[1, 2].set_title('Hinge Angle RMS (deg)')
    axes[1, 2].set_xticks(x)
    axes[1, 2].set_xticklabels(names, rotation=45, ha='right', fontsize=7)

    plt.tight_layout()
    path = os.path.join(output_dir, '05_metrics_comparison.png')
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"  Saved: {path}")
    return fig


def plot_metrics_table(flights: List[FlightData], output_dir: str):
    """图6: 性能指标汇总表"""
    fig, ax = plt.subplots(figsize=(14, max(3, 0.6 * len(flights) + 1.5)))
    ax.axis('off')

    headers = ['Run Name', 'Roll RMS\n(deg)', 'Pitch RMS\n(deg)',
               'Yaw Rate\nRMS (°/s)', 'Yaw Rate\nMax (°/s)',
               'Yaw Osc\n(Hz)', 'Servo\nActivity',
               'Hinge RMS\n(deg)']

    cell_data = []
    for fd in flights:
        m = fd.metrics
        cell_data.append([
            m.name,
            f'{m.roll_rms_error:.2f}',
            f'{m.pitch_rms_error:.2f}',
            f'{m.yaw_rate_rms_error:.2f}',
            f'{m.yaw_rate_max_error:.1f}',
            f'{m.yaw_rate_oscillation_freq:.2f}',
            f'{m.servo_activity:.2f}',
            f'{m.hinge_angle_rms:.2f}' if m.hinge_angle_rms > 0 else '-'
        ])

    table = ax.table(cellText=cell_data, colLabels=headers, loc='center',
                     cellLoc='center')
    table.auto_set_font_size(False)
    table.set_fontsize(8)
    table.scale(1.0, 1.5)

    # 表头样式
    for j in range(len(headers)):
        table[0, j].set_facecolor('#4472C4')
        table[0, j].set_text_props(color='white', fontweight='bold')

    # 交替行颜色
    for i in range(len(cell_data)):
        color = '#D9E2F3' if i % 2 == 0 else 'white'
        for j in range(len(headers)):
            table[i + 1, j].set_facecolor(color)

    fig.suptitle('Performance Metrics Summary', fontsize=14, fontweight='bold', y=0.98)
    plt.tight_layout()
    path = os.path.join(output_dir, '06_metrics_table.png')
    fig.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"  Saved: {path}")
    return fig


def generate_pdf_report(flights: List[FlightData], output_dir: str):
    """生成 PDF 合并报告"""
    pdf_path = os.path.join(output_dir, 'sweep_report.pdf')
    with PdfPages(pdf_path) as pdf:
        # 生成所有图表并添加到 PDF
        figs = []
        figs.append(plot_attitude_comparison(flights, output_dir))
        figs.append(plot_yaw_rate_comparison(flights, output_dir))
        figs.append(plot_servo_comparison(flights, output_dir))
        fig_hinge = plot_hinge_comparison(flights, output_dir)
        if fig_hinge:
            figs.append(fig_hinge)
        figs.append(plot_metrics_bar(flights, output_dir))
        figs.append(plot_metrics_table(flights, output_dir))

        # 重新打开 PNG 文件写入 PDF (因为 fig 已经 close)
        for png in sorted(glob.glob(os.path.join(output_dir, '*.png'))):
            img = plt.imread(png)
            fig_pdf, ax = plt.subplots(figsize=(14, 10))
            ax.imshow(img)
            ax.axis('off')
            pdf.savefig(fig_pdf, bbox_inches='tight')
            plt.close(fig_pdf)

    print(f"\n  PDF Report: {pdf_path}")


# ── 主程序 ──────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='ChainWing 参数扫描日志分析工具',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python3 scripts/analyze_param_sweep.py sweep_logs/
  python3 scripts/analyze_param_sweep.py sweep_logs/ --output results/ --pdf
  python3 scripts/analyze_param_sweep.py sweep_logs/ --sort yaw_rms
        """)

    parser.add_argument('log_dir', help='包含 .ulg 文件的目录')
    parser.add_argument('--output', '-o', default=None,
                        help='输出目录 (默认: <log_dir>/analysis)')
    parser.add_argument('--pdf', action='store_true',
                        help='生成 PDF 合并报告')
    parser.add_argument('--sort', default=None,
                        choices=['yaw_rms', 'roll_rms', 'servo', 'name'],
                        help='排序方式')
    parser.add_argument('--skip-sec', type=float, default=15.0,
                        help='跳过起飞后的秒数 (默认: 15)')
    args = parser.parse_args()

    # 查找 ULG 文件
    ulg_files = sorted(glob.glob(os.path.join(args.log_dir, '*.ulg')))
    if not ulg_files:
        print(f"Error: No .ulg files found in {args.log_dir}")
        sys.exit(1)

    output_dir = args.output or os.path.join(args.log_dir, 'analysis')
    os.makedirs(output_dir, exist_ok=True)

    print(f"===== ChainWing Parameter Sweep Analysis =====")
    print(f"Log directory: {args.log_dir}")
    print(f"Found {len(ulg_files)} .ulg files")
    print(f"Output: {output_dir}")
    print()

    # 提取数据
    flights: List[FlightData] = []
    for ulg_path in ulg_files:
        name = os.path.splitext(os.path.basename(ulg_path))[0]
        # 尝试从同名 JSON 加载参数信息
        json_path = ulg_path.replace('.ulg', '_params.json')
        if os.path.exists(json_path):
            with open(json_path) as f:
                info = json.load(f)
                name = info.get('name', name)

        print(f"Processing: {name}")
        fd = extract_flight_data(ulg_path, name)
        if fd is not None:
            compute_metrics(fd)
            flights.append(fd)
            m = fd.metrics
            print(f"  Duration: {m.flight_duration_sec:.0f}s | "
                  f"Roll RMS: {m.roll_rms_error:.2f}° | "
                  f"Yaw Rate RMS: {m.yaw_rate_rms_error:.2f}°/s | "
                  f"Servo Activity: {m.servo_activity:.2f}")

    if not flights:
        print("Error: No valid flight data extracted")
        sys.exit(1)

    # 排序
    if args.sort == 'yaw_rms':
        flights.sort(key=lambda f: f.metrics.yaw_rate_rms_error)
    elif args.sort == 'roll_rms':
        flights.sort(key=lambda f: f.metrics.roll_rms_error)
    elif args.sort == 'servo':
        flights.sort(key=lambda f: f.metrics.servo_activity)

    # 生成图表
    print(f"\nGenerating plots ({len(flights)} runs)...")
    plot_attitude_comparison(flights, output_dir)
    plot_yaw_rate_comparison(flights, output_dir)
    plot_servo_comparison(flights, output_dir)
    plot_hinge_comparison(flights, output_dir)
    plot_metrics_bar(flights, output_dir)
    plot_metrics_table(flights, output_dir)

    if args.pdf:
        generate_pdf_report(flights, output_dir)

    # 输出 CSV 汇总
    csv_path = os.path.join(output_dir, 'metrics_summary.csv')
    with open(csv_path, 'w') as f:
        headers = ['name', 'roll_rms_deg', 'pitch_rms_deg',
                   'yaw_rate_rms_deg_s', 'yaw_rate_max_deg_s',
                   'yaw_osc_hz', 'servo_activity',
                   'hinge_rms_deg', 'duration_s']
        f.write(','.join(headers) + '\n')
        for fd in flights:
            m = fd.metrics
            f.write(f'{m.name},{m.roll_rms_error:.3f},{m.pitch_rms_error:.3f},'
                    f'{m.yaw_rate_rms_error:.3f},{m.yaw_rate_max_error:.1f},'
                    f'{m.yaw_rate_oscillation_freq:.3f},{m.servo_activity:.3f},'
                    f'{m.hinge_angle_rms:.3f},{m.flight_duration_sec:.0f}\n')
    print(f"\n  CSV Summary: {csv_path}")

    print(f"\n===== Analysis Complete =====")
    print(f"  {len(flights)} runs analyzed")
    print(f"  6 PNG charts + 1 CSV saved to: {output_dir}/")
    if args.pdf:
        print(f"  1 PDF report saved")


if __name__ == '__main__':
    main()
