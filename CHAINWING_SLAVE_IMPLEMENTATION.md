# 链翼无人机从机固件实现文档

## CHAINWING_SLAVE_IMPLEMENTATION.md

> **版本**: v1.1  
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
12. [代码验证完整步骤](#12-代码验证完整步骤)

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

## 12. 代码验证完整步骤

本章节提供从编译到飞行测试的**完整验证流程**，分为 6 个阶段，每个阶段包含具体命令、预期输出和通过/失败判定标准。

---

### 12.1 阶段一：编译验证

#### 12.1.1 完整 SITL 编译

```bash
# 在 PX4 根目录执行
cd ~/PX4-Autopilot    # 或你的 PX4 项目根目录

# 清理旧构建（首次验证建议执行）
make clean

# 编译 SITL 目标
make px4_sitl_default
```

**预期输出**：
```
[100%] Built target px4
```

**通过标准**：
- ✅ 编译完成，无 error
- ✅ 无与 `chainwing_slave`、`ChainwingHingeStatus`、`GZMixingInterfaceServo` 相关的 warning
- ✅ `build/px4_sitl_default/bin/px4` 可执行文件生成

**常见失败及解决**：

| 错误 | 原因 | 解决 |
|------|------|------|
| `ChainwingHingeStatus.msg not found` | msg/CMakeLists.txt 未注册 | 检查是否有 `ChainwingHingeStatus.msg` 行 |
| `undefined reference to chainwing_slave` | px4board 未注册 | 检查 `CONFIG_MODULES_CHAINWING_SLAVE=y` |
| `'remove_result' may be used uninitialized` | GZBridge.cpp 变量未初始化 | 确保 `bool remove_result = false;` |
| `fatal error: uORB/topics/chainwing_hinge_status.h` | 消息未生成 | 先 `make clean` 再重新编译 |

#### 12.1.2 验证模块已编译

```bash
# 检查模块是否存在于构建产物中
ls build/px4_sitl_default/src/modules/chainwing_slave/
```

**预期输出**：应该看到 `.o` 目标文件和库文件。

#### 12.1.3 验证 uORB 消息已生成

```bash
# 检查消息头文件
ls build/px4_sitl_default/uORB/topics/chainwing_hinge_status.h
```

**预期输出**：文件存在。

---

### 12.2 阶段二：模型加载验证

#### 12.2.1 启动 3body 仿真

```bash
# 启动仿真（使用 flat_terrain 世界）
PX4_GZ_WORLD=flat_terrain make px4_sitl gz_chainwing_3body
```

**预期输出**（PX4 控制台）：
```
INFO  [gz_bridge] Connected to Gazebo
INFO  [gz_bridge] Spawn model: chainwing_3body
INFO  [init] Mixer: etc/mixers/...
pxh>
```

**通过标准**：
- ✅ Gazebo 窗口中显示三体模型（中间体 + 左右两翼）
- ✅ PX4 控制台无 `ERROR` 或 `WARN`（忽略初始化过程中的短暂警告）
- ✅ 无 `[Err] Entity named [chainwing_3body] of type [2] not found` 错误

**故障排除**：

| 症状 | 原因 | 解决 |
|------|------|------|
| GZ窗口空白/无模型 | model.sdf 路径错误 | 检查 `Tools/simulation/gz/models/chainwing_3body/` 是否存在 |
| `EntityFactory timeout` | 世界文件问题 | 确认 `flat_terrain.sdf` 存在于 `Tools/simulation/gz/worlds/` |
| 模型生成后立即爆炸 | 质量/惯量异常 | 检查 model.sdf 中各 link 的 inertial 参数 |
| 车轮穿过地面 | collision 缺失 | 检查 model.sdf 中的 collision 几何体 |

#### 12.2.2 验证 GZ 模型关节

在**另一个终端**中执行：

```bash
# 列出所有GZ模型
gz model --list

# 查看模型详情（验证3个link + 2个hinge joint存在）
gz model -m chainwing_3body

# 实时查看铰链关节角度
gz topic -e -t /world/flat_terrain/model/chainwing_3body/joint_state
```

**预期输出**（joint_state）：
```yaml
joint {
  name: "hinge_left"
  axis1 { position: 0.0  velocity: 0.0 }
}
joint {
  name: "hinge_right"
  axis1 { position: 0.0  velocity: 0.0 }
}
```

**通过标准**：
- ✅ 显示 `hinge_left` 和 `hinge_right` 两个关节
- ✅ 初始角度接近 0（±0.001 rad）
- ✅ 关节角度在 ±0.087 rad（±5°）范围内

---

### 12.3 阶段三：从机模块功能验证

#### 12.3.1 检查模块启动状态

在 **PX4 shell (pxh>)** 中执行：

```bash
# 检查从机模块是否运行
chainwing_slave status
```

**预期输出**：
```
chainwing_slave
  Running
  hinge_angle_left:  0.000 rad
  hinge_angle_right: 0.000 rad
  hinge_rate_left:   0.000 rad/s
  hinge_rate_right:  0.000 rad/s
  trim_left:         0.000
  trim_right:        0.000
  ref_initialized:   yes
```

**通过标准**：
- ✅ 显示 `Running`
- ✅ `ref_initialized: yes`（参考姿态已捕获）
- ✅ 初始铰链角接近 0

**如果模块未运行**：
```bash
# 手动启动模块
chainwing_slave start

# 检查参数是否启用
param show CW_SLV_EN
# 应显示: CW_SLV_EN = 1

# 如果为0，手动设置
param set CW_SLV_EN 1
chainwing_slave start
```

#### 12.3.2 验证 uORB 消息发布

```bash
# 监听铰链状态消息（应以 50 Hz 发布）
listener chainwing_hinge_status
```

**预期输出**：
```
TOPIC: chainwing_hinge_status
  timestamp:          12345678
  hinge_angle_left:   0.001
  hinge_angle_right: -0.001
  hinge_rate_left:    0.003
  hinge_rate_right:  -0.002
  trim_left:          0.000
  trim_right:         0.000
  data_valid:         1
```

**通过标准**：
- ✅ `data_valid = 1`（数据有效）
- ✅ `timestamp` 在持续更新（每次 listener 调用值不同）
- ✅ 静止状态下角度接近 0，修正量接近 0

#### 12.3.3 验证参数可用

```bash
# 显示所有从机参数
param show CW_SLV_*
```

**预期输出**：
```
CW_SLV_EN       [1]    : 1
CW_SLV_KP       [0.3]  : 0.3000
CW_SLV_KD       [0.05] : 0.0500
CW_SLV_TRIM_MAX [0.3]  : 0.3000
CW_SLV_LP_FREQ  [10.0] : 10.0000
```

**通过标准**：
- ✅ 所有 5 个参数存在且有默认值

---

### 12.4 阶段四：铰链响应验证（地面静态测试）

#### 12.4.1 手动施加铰链扰动

在**另一个终端**中，通过 GZ 命令对铰链施加外力矩：

```bash
# 对左铰链施加一个正方向力矩（使左翼抬起）
gz service -s /world/flat_terrain/wrench \
  --reqtype gz.msgs.EntityWrench \
  --reptype gz.msgs.Boolean \
  --req 'entity: {name: "left_unit", type: MODEL}, wrench: {torque: {x: 5.0}}'

# 等待2秒后施加反向力矩恢复
sleep 2
gz service -s /world/flat_terrain/wrench \
  --reqtype gz.msgs.EntityWrench \
  --reptype gz.msgs.Boolean \
  --req 'entity: {name: "left_unit", type: MODEL}, wrench: {torque: {x: -5.0}}'
```

> **注意**：如果上述 `wrench` 服务不可用，可以在 GZ GUI 中手动拖拽模型左翼来产生扰动，或在 PX4 shell 中动态调整 `CW_SLV_KP`（如 `param set CW_SLV_KP 1.0`）来观察不同增益下的响应变化。

#### 12.4.2 同时在 PX4 shell 中观察

```bash
# 持续监听铰链状态（每0.5秒刷新）
listener chainwing_hinge_status -r 2
```

**预期响应序列**：
```
# 施加外力矩后
hinge_angle_left:   0.012 rad (≈0.7°)      ← 角度偏离
hinge_rate_left:    0.035 rad/s             ← 有角速率
trim_left:          0.005                   ← PD控制器输出修正

# 弹簧恢复后
hinge_angle_left:   0.001 rad              ← 恢复到接近0
trim_left:          0.000                  ← 修正量回零
```

**通过标准**：
- ✅ 施加外力后 `hinge_angle_left` 偏移（>0.005 rad）
- ✅ `trim_left` 随角度同方向变化（PD 控制正确）
- ✅ 铰链弹簧恢复后，角度和修正量回到 0 附近
- ✅ 无发散振荡

#### 12.4.3 验证 PD 增益效果

```bash
# 测试1：增大 Kp（更大修正量）
param set CW_SLV_KP 1.0
# → 同样角度偏差，trim_left 应该增大约3.3倍

# 测试2：禁用控制器
param set CW_SLV_EN 0
# → trim_left 和 trim_right 应该立即变为 0

# 测试3：恢复默认
param set CW_SLV_EN 1
param set CW_SLV_KP 0.3
```

---

### 12.5 阶段五：飞行验证（SITL 自动飞行）

#### 12.5.1 准备 QGC 地面站

1. 启动 QGroundControl
2. 等待连接到仿真 PX4 实例
3. 确认无 preflight 错误（偶尔的短暂 EKF 警告在数秒后消失为正常，详见 §12.8 问题 #10：已配置 `COM_ARM_EKF_POS=1.0`、`EKF2_REQ_GPS_H=1.0` 放宽收敛阈值）

#### 12.5.2 执行起飞

方式一：QGC 界面操作
1. 在 QGC 中，点击"起飞"
2. 设置起飞高度 30m
3. 点击滑块确认起飞

方式二：MAVLink shell 命令
```bash
# PX4 shell 中
commander takeoff
```

**预期行为**：
- ✅ 三体模型地面滑行加速
- ✅ 达到起飞速度后离地（约 10-12 m/s）
- ✅ 爬升过程中，三个翼段保持大致共面
- ✅ `chainwing_hinge_status` 中铰链角度在 ±3° 以内

#### 12.5.3 巡航阶段铰链观察

```bash
# 持续监听铰链状态
listener chainwing_hinge_status -r 2
```

**巡航中预期值**：
```
hinge_angle_left:   ±0.005~0.02 rad (±0.3°~1.2°)   ← 小幅波动正常
hinge_angle_right:  ±0.005~0.02 rad
trim_left:          ±0.002~0.008                      ← 小修正量
trim_right:         ±0.002~0.008
```

**通过标准**：
- ✅ 铰链角度在 ±3°（±0.052 rad）以内
- ✅ 修正量 < 10%（< 0.1 归一化值）
- ✅ 飞机稳定巡航，不出现显著俯仰/滚转振荡

#### 12.5.4 转弯验证

在 QGC 中规划一个包含转弯的航线任务（loiter 或 waypoint mission），观察转弯时铰链的行为：

```bash
# 观察转弯期间的铰链角度
listener chainwing_hinge_status -r 5
```

**预期行为**：
- 转弯时由于气动不对称，铰链角度会短暂增大（可能到 2-3°）
- PD 控制器产生相应修正量
- 出弯后铰链角度恢复到接近 0

**通过标准**：
- ✅ 转弯时铰链角度不超过 ±5°（关节限位）
- ✅ 无结构性持续偏移（长期平均接近 0）
- ✅ 修正量不饱和（|trim| < trim_max = 0.3）

#### 12.5.5 着陆验证

```bash
commander land
```

**通过标准**：
- ✅ 进近过程平稳
- ✅ 接地时三体模型不发生铰链碰撞极限
- ✅ 地面减速后铰链角度归零

---

### 12.6 阶段六：对比验证（有/无从机控制器）

#### 12.6.1 禁用从机控制器飞行

```bash
# PX4 shell
param set CW_SLV_EN 0
```

重新起飞并观察：
```bash
listener chainwing_hinge_status -r 2
```

**预期**：
- `trim_left` 和 `trim_right` 始终为 0
- 铰链角度波动**更大**（因为没有主动修正）
- 飞行仍然稳定（因为高刚度铰链本身可以维持结构）

#### 12.6.2 启用从机控制器对比

```bash
param set CW_SLV_EN 1
```

**预期改善**：
- 铰链角度波动**减小**（PD 控制器在主动修正）
- 尤其在转弯、阵风扰动时，角度峰值明显降低

#### 12.6.3 量化对比方法

在 PX4 shell 中通过 logger 记录数据：

```bash
# 开始记录日志
logger on

# 飞行一段时间（含直线+转弯）
# ...

# 停止记录
logger off
```

日志文件位于 `build/px4_sitl_default/rootfs/log/` 目录下。使用 [Flight Review](https://review.px4.io/) 或 [PlotJuggler](https://github.com/facontidavide/PlotJuggler) 分析 `chainwing_hinge_status` 话题：

**分析指标**：

| 指标 | 无控制器 | 有控制器 | 通过标准 |
|------|----------|----------|----------|
| 铰链角度 RMS | ~0.03 rad | < 0.015 rad | 降低 50%+ |
| 铰链角度峰值 | ~0.06 rad | < 0.03 rad | 降低 50%+ |
| trim 修正量 RMS | 0 | < 0.05 | 修正量合理 |

---

### 12.7 验证清单（Checklist）

以下是完整的验证清单，可以打印使用：

```
═══════════════════════════════════════════════════════
  链翼从机固件验证清单 (Chainwing Slave Verification)
═══════════════════════════════════════════════════════

阶段一：编译验证
  □ make px4_sitl_default 编译成功（无 error）
  □ 无 chainwing_slave/ChainwingHingeStatus 相关 warning
  □ chainwing_slave .o 文件已生成
  □ chainwing_hinge_status.h 头文件已生成

阶段二：模型加载
  □ GZ 正常加载 chainwing_3body 模型
  □ GZ 中可见三体结构（中+左+右）
  □ gz model 显示 hinge_left 和 hinge_right 关节
  □ 初始关节角度 ≈ 0
  □ PX4 控制台无持续 ERROR

阶段三：模块功能
  □ chainwing_slave status 显示 Running
  □ ref_initialized = yes
  □ listener chainwing_hinge_status 有数据（data_valid=1）
  □ 所有 5 个 CW_SLV_* 参数存在
  □ 消息以 ~50 Hz 持续更新

阶段四：铰链响应（地面）
  □ 施加外力后 hinge_angle 偏移
  □ trim 随角度同方向变化（PD 极性正确）
  □ 外力移除后角度恢复接近 0
  □ 无发散振荡
  □ Kp 调大后修正量增大（线性关系）
  □ CW_SLV_EN=0 时修正量为 0

阶段五：飞行验证
  □ 起飞过程稳定
  □ 巡航铰链角度 < ±3°
  □ 巡航修正量 < 10%
  □ 转弯时铰链角度 < ±5°（不触碰限位）
  □ 修正量不饱和（|trim| < 0.3）
  □ 着陆过程平稳

阶段六：对比验证
  □ 禁用控制器后铰链角度波动增大
  □ 启用控制器后铰链角度波动减小
  □ 日志分析：铰链角 RMS 降低 50%+

═══════════════════════════════════════════════════════
  全部通过 = 从机固件验证完成 ✓
═══════════════════════════════════════════════════════
```

---

### 12.8 常见问题汇总

| # | 问题 | 原因 | 解决方案 |
|---|------|------|----------|
| 1 | `chainwing_slave: command not found` | 模块未编译或未注册 | 检查 `boards/px4/sitl/default.px4board` 中 `CONFIG_MODULES_CHAINWING_SLAVE=y` |
| 2 | `TOPIC chainwing_hinge_status not found` | uORB 消息未注册 | 检查 `msg/CMakeLists.txt` 中有 `ChainwingHingeStatus.msg`，并 `make clean && make` |
| 3 | 铰链角度始终为 0 | 参考姿态未初始化 或 IMU 数据未订阅 | `chainwing_slave status` 检查 `ref_initialized`；确认 `vehicle_angular_velocity` 话题有数据 |
| 4 | 修正量为 0 但角度不为 0 | `CW_SLV_EN=0` 或增益为 0 | `param show CW_SLV_*` 检查所有参数 |
| 5 | 铰链持续振荡 | Kp 过大或 Kd 过小 | 减小 `CW_SLV_KP` 至 0.1，增大 `CW_SLV_KD` 至 0.1 |
| 6 | 修正量始终饱和 | `CW_SLV_TRIM_MAX` 过小或角度偏差过大 | 增大 `CW_SLV_TRIM_MAX` 或检查铰链刚度是否正确 |
| 7 | GZ 模型爆炸 | 铰链参数（惯量/刚度）不匹配 | 检查 model.sdf 中 inertial 和 joint dynamics 参数 |
| 8 | 编译 `-Werror=maybe-uninitialized` | 变量未初始化 | 确保所有可能未赋值的变量有初始值（如 `bool result = false;`） |
| 9 | `MAG #0 TIMEOUT` 仿真启动时 | sensor_mag_sim 等待 GPS | 已在 SensorMagSim::init() 中修复（默认磁场初始化），确保使用最新代码 |
| 10 | `Preflight Fail: position estimate error` | EKF 收敛慢 | 已设置 `COM_ARM_EKF_POS=1.0, EKF2_REQ_GPS_H=1.0`，等待 5-10s |

---

*文档结束*
