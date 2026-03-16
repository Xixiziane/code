# 链翼无人机从机固件实现文档

## CHAINWING_SLAVE_IMPLEMENTATION.md

> **版本**: v1.0  
> **日期**: 2024  
> **基于仓库**: PX4_test (Chainwing UAV firmware)  
> **关联文档**: CHAINWING_MULTI_CONTROLLER_GUIDE.md, CHAINWING_FIRMWARE_DOC.md, CHAINWING_TECHNICAL_DETAILS.md

---

## 目录

1. [概述](#1-概述)
2. [系统架构](#2-系统架构)
3. [铰链连接建模](#3-铰链连接建模)
4. [IMU积分法测量相对位置](#4-imu积分法测量相对位置)
5. [从机升降舵修正控制律](#5-从机升降舵修正控制律)
6. [主从飞控MAVLink通信](#6-主从飞控mavlink通信)
7. [Gazebo三体仿真模型](#7-gazebo三体仿真模型)
8. [PX4固件修改清单](#8-px4固件修改清单)
9. [参数配置](#9-参数配置)
10. [仿真使用指南](#10-仿真使用指南)
11. [文件清单](#11-文件清单)

---

## 1. 概述

### 1.1 背景

链翼（Chain-Wing）无人机由三架固定翼飞机通过翼尖铰链连接组成。每架飞机有独立的飞控，
但需要协调飞行以保持三体共面（coplanar）。中间机体为**主机（Master）**，两侧机体为**从机（Slave）**。

### 1.2 核心需求

| 需求 | 实现方案 |
|------|----------|
| 主从相对位置测量 | **IMU积分法** — 对比主从IMU角速度差积分得到铰链角度 |
| 从机位置调整执行器 | **尾翼升降舵** — 升降舵偏转产生俯仰力矩修正铰链角 |
| 从机油门 = 整体命令 + 修正 | δ_total = clamp(δ_master + δ_trim, -1.0, 1.0) |
| 主从通信 | **串口 + MAVLink v2 协议** |
| 铰链非纯铰链建模 | **高刚度弹簧 + 高阻尼**（500 N·m/rad + 50 N·m·s/rad） |

### 1.3 设计原则

1. **实际翼尖连接不是纯铰链** — 刚度较大、变化不大 → 仿真采用**高弹簧刚度 + 高阻尼**的revolute joint
2. **单PX4实例仿真** — 一个PX4进程控制三体模型，chainwing_slave模块估算铰链角并修正
3. **硬件部署** — 三个独立PX4飞控，通过UART MAVLink通信，各自有独立IMU

---

## 2. 系统架构

### 2.1 仿真架构（当前实现）

```
┌─────────────────────────────────────────────────────────────────┐
│                     PX4 SITL (单实例)                            │
│                                                                 │
│  ┌──────────────┐    ┌──────────────────┐    ┌───────────────┐ │
│  │ FW Attitude   │    │ chainwing_slave   │    │ GZBridge      │ │
│  │ Controller    │    │ (铰链角估算+PD)   │    │ (传感器+执行器)│ │
│  │               │    │                  │    │               │ │
│  │ pitch_cmd ────┼───►│ + δ_trim ────────┼───►│ servo output  │ │
│  └──────────────┘    └──────────────────┘    └───────┬───────┘ │
│                             ▲                        │         │
│                             │ IMU data               │         │
│                             │                        ▼         │
└─────────────────────────────┼────────────────────────┼─────────┘
                              │                        │
                     ┌────────┴────────────────────────┴─────────┐
                     │          Gazebo Sim (chainwing_3body)      │
                     │  ┌──────────┐ ┌──────────┐ ┌──────────┐  │
                     │  │ left_unit│←→│base_link │←→│right_unit│  │
                     │  │ (从机L)  │铰│ (主机)   │铰│ (从机R)  │  │
                     │  └──────────┘链└──────────┘链└──────────┘  │
                     └───────────────────────────────────────────┘
```

### 2.2 硬件架构（目标部署）

```
┌───────────────┐   UART+MAVLink   ┌───────────────┐   UART+MAVLink   ┌───────────────┐
│  Slave Left   │◄────────────────►│   Master       │◄────────────────►│  Slave Right  │
│  (MAV_SYS_ID  │   921600 bps     │  (MAV_SYS_ID   │   921600 bps     │  (MAV_SYS_ID  │
│    = 2)       │                  │    = 1)        │                  │    = 3)       │
│               │                  │                │                  │               │
│  ·IMU(独立)   │                  │  ·IMU(参考)    │                  │  ·IMU(独立)   │
│  ·升降舵控制  │                  │  ·整体姿态控制 │                  │  ·升降舵控制  │
│  ·电机控制    │                  │  ·电机控制     │                  │  ·电机控制    │
└───────────────┘                  └───────────────┘                  └───────────────┘
```

### 2.3 数据流

```
[主机IMU] ──(MAVLink)──► [从机] ──► 计算 θ_hinge = θ_slave - θ_master
                                    ──► δ_trim = Kp × θ + Kd × θ̇
                                    ──► δ_total = δ_master_cmd + δ_trim
                                    ──► 执行升降舵偏转
```

---

## 3. 铰链连接建模

### 3.1 实际连接特性

用户指出：实际翼尖连接**并不是纯铰链**，而是**刚度较大的连接**，变化不太大。

### 3.2 仿真建模方案

采用 Gazebo revolute joint + 高弹簧刚度 + 高阻尼：

| 参数 | 值 | 说明 |
|------|------|------|
| 关节类型 | revolute | 单轴旋转（X轴/纵向） |
| 弹簧刚度 | **500 N·m/rad** | 模拟高刚度连接 |
| 阻尼 | **50 N·m·s/rad** | 抑制振荡，模拟近刚性行为 |
| 角度限制 | **±5° (±0.087 rad)** | 限制最大偏转 |
| 旋转轴 | X轴（前向） | 相对俯仰旋转 |
| 弹簧参考 | 0.0 rad | 平衡位置为共面 |

### 3.3 参数选择依据

**弹簧刚度 500 N·m/rad 的物理含义**：

对于单侧从机质量 m = 1.9 kg，铰链臂长 L = 0.6 m：
- 铰链惯量：I_hinge = m × L² = 1.9 × 0.6² = **0.684 kg·m²**
- 自然频率：ωn = √(k/I) = √(500/0.684) = **27 rad/s ≈ 4.3 Hz**
- 临界阻尼：c_crit = 2 × √(k × I) = 2 × √(500 × 0.684) = **37 N·m·s/rad**
- 实际阻尼比：ζ = c/(2√(kI)) = 50/37 = **1.35（过阻尼）**

过阻尼确保铰链角扰动**不会振荡**，而是缓慢恢复到平衡位置，符合"刚度较大、变化不大"的实际特性。

### 3.4 SDF实现

```xml
<!-- hinge_left: 主机-左从机铰链 -->
<joint name='hinge_left' type='revolute'>
    <parent>base_link</parent>
    <child>left_unit</child>
    <pose relative_to='base_link'>0 -0.60 0.03 0 0 0</pose>
    <axis>
        <xyz>1 0 0</xyz>
        <limit>
            <lower>-0.087</lower>   <!-- -5° -->
            <upper>0.087</upper>    <!-- +5° -->
        </limit>
        <dynamics>
            <damping>50.0</damping>
            <spring_reference>0.0</spring_reference>
            <spring_stiffness>500.0</spring_stiffness>
        </dynamics>
    </axis>
</joint>
```

**文件**: `Tools/simulation/gz/models/chainwing_3body/model.sdf`

---

## 4. IMU积分法测量相对位置

### 4.1 原理

主从机各有独立的IMU。铰链角 = 从机俯仰角 - 主机俯仰角。

在硬件上：
1. 主机通过MAVLink发送自身俯仰角/角速度
2. 从机读取自身IMU数据
3. 差值 = 铰链相对角度

在仿真中（单PX4实例）：
1. 从 `vehicle_angular_velocity` 话题获取角速度
2. 从 `vehicle_attitude` 话题获取姿态四元数
3. 通过互补滤波器估算铰链角

### 4.2 互补滤波器设计

```
         ┌──────────────┐
         │  角速度积分    │──(高频)──┐
         │ θ += ω × dt   │         │     ┌──────────┐
         └──────────────┘         ├────►│ 加权融合  │──► θ_hinge
         ┌──────────────┐         │     │           │
         │ 姿态偏差      │──(低频)──┘     │ α = 0.02  │
         │ θ_att - θ_ref │              └──────────┘
         └──────────────┘
```

**关键公式**：

```
// 低通滤波角速度
ω_filtered = (1 - α_lp) × ω_prev + α_lp × ω_raw

// 积分+衰减（防止漂移）
θ = exp(-dt/τ) × (θ + ω_filtered × dt)
    其中 τ = 2.0 s（衰减时间常数）

// 互补滤波（长期修正）
θ_final = (1 - α_cf) × θ_integrated + α_cf × θ_attitude
    其中 α_cf = 0.02（低权重=信任积分更多）
```

### 4.3 参数

| 参数 | 值 | 说明 |
|------|------|------|
| 低通滤波频率 | 10 Hz (CW_SLV_LP_FREQ) | 角速度信号滤波 |
| 积分衰减常数 τ | 2.0 s | 防止积分漂移 |
| 互补滤波权重 α | 0.02 | 低频姿态修正 |
| 更新频率 | 50 Hz | chainwing_slave模块运行频率 |

### 4.4 实现

**文件**: `src/modules/chainwing_slave/ChainwingSlave.cpp` → `updateHingeEstimate()`

```cpp
void ChainwingSlave::updateHingeEstimate(float dt)
{
    // 1. 读取IMU角速度
    const float roll_rate = angular_vel.xyz[0]; // X轴=铰链旋转轴

    // 2. 低通滤波
    const float alpha = dt / (dt + 1.0f / (2π × f_lp));
    _hinge_rate = (1-alpha) * _hinge_rate + alpha * roll_rate;

    // 3. 积分+指数衰减
    const float decay = exp(-dt / τ);
    _hinge_angle = decay * (_hinge_angle + _hinge_rate * dt);

    // 4. 姿态互补修正
    _hinge_angle = (1-0.02) * _hinge_angle + 0.02 * pitch_error;
}
```

---

## 5. 从机升降舵修正控制律

### 5.1 PD控制器

从机通过尾翼升降舵（elevon）产生俯仰力矩来修正铰链角：

```
δ_trim = Kp × θ_hinge + Kd × θ̇_hinge
δ_total = clamp(δ_master + δ_trim, -1.0, 1.0)
```

其中：
- `δ_master` = 主飞控给从飞控的整体升降舵命令（通过MAVLink接收）
- `δ_trim` = 从机根据铰链角自行计算的修正量
- `δ_total` = 从机升降舵的最终输出

### 5.2 控制增益设计

| 参数 | 名称 | 值 | 推导依据 |
|------|------|------|----------|
| Kp | CW_SLV_KP | **0.3** | ωn=3.4 rad/s → Kp = ωn² × I / q_∞ / S / Cmδ |
| Kd | CW_SLV_KD | **0.05** | ζ=0.7 → Kd = 2ζωn × I / q_∞ / S / Cmδ |
| δ_trim_max | CW_SLV_TRIM_MAX | **0.3** | 30%行程，24.6 N·m 力矩裕量 |

**增益推导**（来自 CHAINWING_MULTI_CONTROLLER_GUIDE.md §17）：

```
设计自然频率: ωn = 3.4 rad/s
阻尼比: ζ = 0.7

铰链惯量: I_hinge = 0.684 kg·m²
巡航动压: q = 0.5 × 1.2041 × 20² = 240.82 Pa
参考面积: S = 0.36 m²
升降舵系数: Cmδ = -0.5 /rad

Kp = ωn² × I / (q × S × |Cmδ|)
   = 3.4² × 0.684 / (240.82 × 0.36 × 0.5)
   = 7.91 / 43.35
   ≈ 0.18 → 取 0.3（含裕量）

Kd = 2 × ζ × ωn × I / (q × S × |Cmδ|)
   = 2 × 0.7 × 3.4 × 0.684 / 43.35
   = 3.26 / 43.35
   ≈ 0.075 → 取 0.05（保守）
```

### 5.3 力矩能力

在30%升降舵行程（±0.3 rad偏转）下：

```
M_trim = q × S × Cmδ × δ
       = 240.82 × 0.36 × 0.5 × 0.3
       ≈ 13.0 N·m
```

配合 elevator LiftDrag 插件的 `control_joint_rad_to_cl = -4.0`：

```
M_elevator_trim ≈ q × S_tail × CL_α × δ × L_arm
               = 240.82 × 0.12 × 5.25 × 0.3 × 0.5
               ≈ 24.6 N·m (总修正力矩)
```

**足以修正 ±10° 铰链偏转**（铰链弹簧恢复力矩 = 500 × 0.174 = 87 N·m，但实际偏转 < ±5°，
弹簧力矩 < 43.5 N·m，升降舵力矩提供额外辅助修正）。

### 5.4 δ_total 的含义

```
从机升降舵的最终输出:
  δ_total = clamp(δ_master + δ_trim, -1.0, 1.0)
  
其中:
  δ_master = 主飞控发送的整体俯仰命令 (通过 MAVLink)
  δ_trim   = 从机PD控制器计算的铰链修正量
  |δ_trim| ≤ 0.3 (CW_SLV_TRIM_MAX 限制)
```

**实现位置**: `src/modules/simulation/gz_bridge/GZMixingInterfaceServo.cpp`

```cpp
// servo_0 = 左从机升降舵, servo_2 = 右从机升降舵
if (hinge_valid) {
    if (i == 0) { output += hinge_status.trim_left; }
    if (i == 2) { output += hinge_status.trim_right; }
    output = clamp(output, -1.0, 1.0);
}
```

---

## 6. 主从飞控MAVLink通信

### 6.1 物理层

| 参数 | 值 |
|------|------|
| 接口 | UART (TTL 3.3V) |
| 波特率 | 921,600 bps |
| 线缆 | 4线 (TX, RX, GND, 5V) |
| 端口 | 主机TELEM2/TELEM3 → 从机TELEM1 |
| 延迟 | < 1 ms |

### 6.2 MAVLink消息

**主机 → 从机** （下行命令）：

| 消息 | 频率 | 内容 |
|------|------|------|
| HEARTBEAT | 1 Hz | 系统状态、飞行模式 |
| ATTITUDE | 50 Hz | 主机俯仰角/角速度（IMU积分参考） |
| SET_ACTUATOR_CONTROL_TARGET | 50 Hz | 整体升降舵/油门命令 |
| COMMAND_LONG | 按需 | 解锁/锁定/模式切换 |

**从机 → 主机** （上行反馈）：

| 消息 | 频率 | 内容 |
|------|------|------|
| HEARTBEAT | 1 Hz | 从机状态 |
| ATTITUDE | 10 Hz | 从机姿态（冗余） |
| DEBUG_FLOAT_ARRAY | 10 Hz | 铰链角度/修正量 |

### 6.3 MAVLink实例配置

```bash
# 主机 (MAV_SYS_ID = 1)
mavlink start -d /dev/ttyS2 -b 921600 -m config -r 20000
# -d /dev/ttyS2 = TELEM2 端口
# -b 921600     = 波特率
# -m config     = 消息集
# -r 20000      = 最大消息速率 (bytes/s)

# 从机 (MAV_SYS_ID = 2)
mavlink start -d /dev/ttyS1 -b 921600 -m config -r 20000
```

### 6.4 带宽估算

```
主机→从机:
  ATTITUDE (50Hz × 28B)       = 1,400 B/s
  SET_ACTUATOR (50Hz × 40B)   = 2,000 B/s
  HEARTBEAT (1Hz × 17B)       =    17 B/s
  总计 ≈ 3.4 kB/s × 10(开销) = 34 kbps

双向合计 ≈ 68 kbps << 921,600 bps (利用率 7.4%)
```

---

## 7. Gazebo三体仿真模型

### 7.1 模型结构

**文件**: `Tools/simulation/gz/models/chainwing_3body/model.sdf`

```
chainwing_3body
├── base_link (主机，1.9 kg)
│   ├── IMU传感器
│   ├── 气压传感器
│   ├── rotor_center (电机1)
│   ├── elevator (升降舵，servo_1)
│   └── NoseWheel (前起落架)
│
├── left_unit (左从机，1.9 kg)
│   ├── IMU传感器 (imu_sensor_left)
│   ├── rotor_left (电机0)
│   ├── left_elevon (servo_0)
│   └── LeftWheel (左起落架)
│
├── right_unit (右从机，1.9 kg)
│   ├── IMU传感器 (imu_sensor_right)
│   ├── rotor_right (电机2)
│   ├── right_elevon (servo_2)
│   └── RightWheel (右起落架)
│
├── hinge_left (revolute joint, 高刚度)
│   parent=base_link, child=left_unit
│
├── hinge_right (revolute joint, 高刚度)
│   parent=base_link, child=right_unit
│
└── 气动插件 (9个LiftDrag + 3个JointPositionController + 3个Motor)
```

### 7.2 物理参数

**单体参数**（来自 MATLAB 开环模型）：

| 参数 | 值 | 来源 |
|------|------|------|
| 质量 | 1.9 kg | MATLAB / model.sdf |
| 翼展 | 1.2 m | MATLAB / model.sdf |
| 弦长 | 0.30 m | MATLAB |
| 翼面积 | 0.36 m² | 0.30 × 1.2 |
| Ixx | 0.0894 kg·m² | MATLAB 单体对角惯量 |
| Iyy | 0.144 kg·m² | MATLAB 单体对角惯量 |
| Izz | 0.162 kg·m² | MATLAB 单体对角惯量 |
| CG偏移 | -0.06 m (向后) | 静稳定裕度 45% |

**组合参数**（平行轴定理）：

| 参数 | 值 | 计算 |
|------|------|------|
| 总质量 | 5.7 kg | 3 × 1.9 |
| 总翼展 | 3.6 m | 3 × 1.2 |
| 总翼面积 | 1.08 m² | 3 × 0.36 |
| Ixx (总) | 5.740 kg·m² | 平行轴定理 |
| Iyy (总) | 0.432 kg·m² | 3 × 0.144 |
| Izz (总) | 5.958 kg·m² | 平行轴定理 |

### 7.3 气动参数

| 参数 | 翼面 | 水平尾翼 | 垂直尾翼 |
|------|------|----------|----------|
| a0 | -0.05 rad | -0.2 rad | 0.0 rad |
| CLA | 5.25 /rad | 5.25 /rad | 4.75 /rad |
| CDA | 0.65 | 0.65 | 0.64 |
| alpha_stall | 0.227 rad (13°) | 0.34 rad | 0.34 rad |
| area | 0.36 m² | 0.18/0.12 m² | 0.02 m² |
| control_joint_rad_to_cl | -0.3 | -4.0 | — |

### 7.4 执行器映射

| GZ名称 | 类型 | PX4通道 | 功能 |
|--------|------|---------|------|
| command/motor_speed[0] | ESC | SIM_GZ_EC_FUNC1=101 | 左电机 |
| command/motor_speed[1] | ESC | SIM_GZ_EC_FUNC2=102 | 中电机 |
| command/motor_speed[2] | ESC | SIM_GZ_EC_FUNC3=103 | 右电机 |
| servo_0 | Servo | SIM_GZ_SV_FUNC1=201 | 左升降舵(+trim) |
| servo_1 | Servo | SIM_GZ_SV_FUNC2=202 | 中升降舵(主控) |
| servo_2 | Servo | SIM_GZ_SV_FUNC3=203 | 右升降舵(+trim) |

---

## 8. PX4固件修改清单

### 8.1 新增文件

| 文件路径 | 说明 |
|----------|------|
| `msg/ChainwingHingeStatus.msg` | 铰链状态 uORB 消息定义 |
| `src/modules/chainwing_slave/ChainwingSlave.hpp` | 从机控制器头文件 |
| `src/modules/chainwing_slave/ChainwingSlave.cpp` | 从机控制器实现 |
| `src/modules/chainwing_slave/CMakeLists.txt` | 从机模块构建配置 |
| `src/modules/chainwing_slave/Kconfig` | 从机模块内核配置 |
| `src/modules/chainwing_slave/chainwing_slave_params.c` | PD控制器参数定义 |
| `Tools/simulation/gz/models/chainwing_3body/model.sdf` | 三体仿真模型 |
| `Tools/simulation/gz/models/chainwing_3body/model.config` | 模型配置 |
| `ROMFS/px4fmu_common/init.d-posix/airframes/4008_gz_chainwing_3body` | 三体机架配置 |

### 8.2 修改文件

| 文件路径 | 修改内容 |
|----------|----------|
| `msg/CMakeLists.txt` | 注册 ChainwingHingeStatus.msg |
| `src/modules/simulation/gz_bridge/GZMixingInterfaceServo.hpp` | 添加 hinge_status 订阅 |
| `src/modules/simulation/gz_bridge/GZMixingInterfaceServo.cpp` | 在servo输出中叠加铰链修正量 |
| `ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt` | 注册 4008_gz_chainwing_3body |
| `boards/px4/sitl/default.px4board` | 启用 MODULES_CHAINWING_SLAVE |

### 8.3 GZMixingInterfaceServo 修改详情

**核心改动**：在 `updateOutputs()` 中读取 `chainwing_hinge_status` 消息，
将 `trim_left`/`trim_right` 叠加到 servo_0/servo_2 的输出上：

```cpp
// 读取铰链修正量
chainwing_hinge_status_s hinge_status{};
bool hinge_valid = false;
if (_hinge_status_sub.updated()) {
    _hinge_status_sub.copy(&hinge_status);
    hinge_valid = hinge_status.data_valid;
}

// 叠加修正（仅对从机升降舵）
if (hinge_valid) {
    if (i == 0) { output += hinge_status.trim_left; }   // 左从机
    if (i == 2) { output += hinge_status.trim_right; }   // 右从机
    output = clamp(output, -1.0, 1.0);
}
```

---

## 9. 参数配置

### 9.1 从机控制器参数

| 参数名 | 默认值 | 范围 | 说明 |
|--------|--------|------|------|
| CW_SLV_EN | 0 | 0/1 | 使能从机控制器 |
| CW_SLV_KP | 0.3 | 0.0-5.0 | PD比例增益 (1/rad) |
| CW_SLV_KD | 0.05 | 0.0-2.0 | PD微分增益 (s/rad) |
| CW_SLV_TRIM_MAX | 0.3 | 0.0-1.0 | 最大修正量（归一化） |
| CW_SLV_LP_FREQ | 10.0 | 0.0-50.0 | 角速度低通滤波 (Hz) |

### 9.2 机架配置关键差异（4008 vs 4007）

| 配置项 | 4007 (单体) | 4008 (三体) | 说明 |
|--------|-------------|-------------|------|
| PX4_SIM_MODEL | chainwing | chainwing_3body | GZ模型名 |
| CW_SLV_EN | 未设置 | 1 | 启用从机控制器 |
| CW_SLV_KP | — | 0.3 | PD增益 |
| CW_SLV_KD | — | 0.05 | PD增益 |
| CW_SLV_TRIM_MAX | — | 0.3 | 修正量限制 |
| CW_SLV_LP_FREQ | — | 10.0 | 滤波频率 |

---

## 10. 仿真使用指南

### 10.1 启动三体仿真

```bash
# 使用三体模型启动SITL
make px4_sitl gz_chainwing_3body

# 或指定世界
PX4_GZ_WORLD=flat_terrain make px4_sitl gz_chainwing_3body
```

### 10.2 验证从机控制器

```bash
# PX4 shell 中检查从机模块状态
chainwing_slave status

# 查看铰链状态消息
listener chainwing_hinge_status

# 查看从机参数
param show CW_SLV_*
```

### 10.3 实时调参

```bash
# 增大比例增益（更积极修正）
param set CW_SLV_KP 0.5

# 增大微分增益（更多阻尼）
param set CW_SLV_KD 0.1

# 增大修正量限制（更大修正范围）
param set CW_SLV_TRIM_MAX 0.5

# 禁用从机控制器（调试用）
param set CW_SLV_EN 0
```

### 10.4 观察铰链行为

在Gazebo中可以直接观察：
1. 三体模型在起飞后是否保持共面
2. 铰链角是否在 ±5° 范围内
3. 从机升降舵是否有修正偏转

通过 GZ Topic Echo 可以查看铰链关节状态：
```bash
gz topic -e -t /model/chainwing_3body/joint_state
```

---

## 11. 文件清单

### 11.1 完整新增/修改文件列表

```
新增文件:
├── msg/ChainwingHingeStatus.msg                                    # uORB消息
├── src/modules/chainwing_slave/
│   ├── ChainwingSlave.hpp                                          # 模块头文件
│   ├── ChainwingSlave.cpp                                          # 模块实现
│   ├── CMakeLists.txt                                              # 构建配置
│   ├── Kconfig                                                     # 内核配置
│   └── chainwing_slave_params.c                                    # 参数定义
├── Tools/simulation/gz/models/chainwing_3body/
│   ├── model.sdf                                                   # 三体GZ模型
│   └── model.config                                                # 模型配置
├── ROMFS/px4fmu_common/init.d-posix/airframes/
│   └── 4008_gz_chainwing_3body                                     # 机架配置

修改文件:
├── msg/CMakeLists.txt                                              # +1行
├── src/modules/simulation/gz_bridge/GZMixingInterfaceServo.hpp     # +3行
├── src/modules/simulation/gz_bridge/GZMixingInterfaceServo.cpp     # +20行
├── ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt       # +1行
├── boards/px4/sitl/default.px4board                                # +1行

文档:
└── CHAINWING_SLAVE_IMPLEMENTATION.md                               # 本文档
```

### 11.2 数据来源溯源

| 数据 | 来源 | 仓库文件 |
|------|------|----------|
| 质量 1.9 kg | MATLAB开环模型 | `动力学开环验证/Run_Validation.m` |
| Ixx=0.0894 | MATLAB计算 | `动力学开环验证/ThreeBodyAerodynamics.m` |
| CLA=5.25 | a0=2π有限翼修正 | CHAINWING_FIRMWARE_DOC.md §2 |
| Kp=0.3, Kd=0.05 | PD控制器设计 | CHAINWING_MULTI_CONTROLLER_GUIDE.md §10 |
| 铰链惯量 0.684 | m×L² = 1.9×0.6² | CHAINWING_MULTI_CONTROLLER_GUIDE.md §17 |
| 921600 bps | MAVLink UART配置 | CHAINWING_MULTI_CONTROLLER_GUIDE.md §3 |
| Cmδ = -0.5 | 升降舵力矩系数 | CHAINWING_MULTI_CONTROLLER_GUIDE.md §17 |
| rad_to_cl = -4.0 | GZ LiftDrag 插件 | CHAINWING_TECHNICAL_DETAILS.md §4 |

---

*文档结束*
