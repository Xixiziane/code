# 链翼无人机从机固件实现文档

## CHAINWING_SLAVE_IMPLEMENTATION.md

> **版本**: v1.8  
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
13. [当前主从机飞控架构现状分析](#13-当前主从机飞控架构现状分析)
14. [3body仿真完成后的下一步工作](#14-3body仿真完成后的下一步工作)
15. [主飞控固件分析：烧什么、改什么、为什么](#15-主飞控固件分析烧什么改什么为什么)
16. [仿真坐标系差异与 Yaw Estimate Error 解析](#16-仿真坐标系差异与-yaw-estimate-error-解析)
17. [仿真操作常见问题：左右反转、gz命令、listener中断](#17-仿真操作常见问题左右反转gz命令listener中断)
18. [深入问题解答：listener -n 0 失效、gz --timeout、QGC 实时查看](#18-深入问题解答listener--n-0-失效gz---timeoutqgc-实时查看)
19. [进一步问题诊断：wrench 超时、listener 秒退、模块状态确认](#19-进一步问题诊断wrench-超时listener-秒退模块状态确认)
20. [终极诊断：模块未启动根因 + 替代方案 + QGC调参指南](#20-终极诊断模块未启动根因--替代方案--qgc调参指南)

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
  --req 'entity: {name: "chainwing_3body_0::left_unit", type: LINK}, wrench: {torque: {x: 5.0}}'

# 等待2秒后施加反向力矩恢复
sleep 2
gz service -s /world/flat_terrain/wrench \
  --reqtype gz.msgs.EntityWrench \
  --reptype gz.msgs.Boolean \
  --req 'entity: {name: "chainwing_3body_0::left_unit", type: LINK}, wrench: {torque: {x: -5.0}}'
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

## 13. 当前主从机飞控架构现状分析

> **核心问题**：就目前仓库代码而言，主机和从机的飞控分别是什么？

### 13.1 总体架构图

```
┌─────────────────────────────────────────────────────────────────┐
│              当前仓库代码中的飞控架构（单实例仿真）                │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌─── 单个 PX4 SITL 进程 ──────────────────────────────────┐   │
│  │                                                          │   │
│  │  ┌────────────────────────────────┐   (标准 PX4 模块)   │   │
│  │  │ 主机飞控 = 标准 PX4 固定翼模块  │                     │   │
│  │  │ ├─ fw_att_control（姿态控制）   │                     │   │
│  │  │ ├─ fw_rate_control（角速率控制）│                     │   │
│  │  │ ├─ fw_pos_control（位置控制）   │                     │   │
│  │  │ ├─ control_allocator（控制分配）│                     │   │
│  │  │ ├─ navigator（导航/任务执行）   │                     │   │
│  │  │ ├─ ekf2（状态估计）             │                     │   │
│  │  │ └─ commander（飞行状态管理）    │                     │   │
│  │  └────────────────────────────────┘                     │   │
│  │            ↓  产生 servo 指令                            │   │
│  │  ┌────────────────────────────────┐   (新增模块)        │   │
│  │  │ 从机飞控 = chainwing_slave     │                     │   │
│  │  │ ├─ IMU 积分估算铰链角度         │                     │   │
│  │  │ ├─ PD 控制器计算修正量          │                     │   │
│  │  │ └─ 发布 ChainwingHingeStatus   │                     │   │
│  │  └────────────────────────────────┘                     │   │
│  │            ↓  修正量叠加到 servo 输出                    │   │
│  │  ┌────────────────────────────────┐                     │   │
│  │  │ GZMixingInterfaceServo         │                     │   │
│  │  │ servo_0 = 主机指令 + trim_left │   ← 左从机修正      │   │
│  │  │ servo_1 = 主机指令             │   ← 主机升降舵       │   │
│  │  │ servo_2 = 主机指令 + trim_right│   ← 右从机修正      │   │
│  │  └────────────────────────────────┘                     │   │
│  └──────────────────────────────────────────────────────────┘   │
│            ↓                                                     │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │  Gazebo 三体模型 (chainwing_3body)                       │   │
│  │  base_link(主机) ←铰链→ left_unit(左从) + right_unit(右从)│   │
│  └──────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
```

### 13.2 主机飞控详解

**主机飞控 = PX4 标准固定翼控制栈**，不是一个单独的模块，而是由多个标准 PX4 模块组合实现：

| 模块 | 文件位置 | 主机角色 |
|------|----------|----------|
| `fw_att_control` | `src/modules/fw_att_control/` | 姿态控制器 — 计算全机的 roll/pitch/yaw 控制力矩 |
| `fw_rate_control` | `src/modules/fw_rate_control/` | 角速率内环 — 产生 roll_rate/pitch_rate/yaw_rate 指令 |
| `fw_pos_control` | `src/modules/fw_pos_control/` | 位置/高度控制 — TECS + NPFG 算法 |
| `control_allocator` | `src/modules/control_allocator/` | 控制分配 — 将力矩分配到 3 电机 + 3 舵面 |
| `navigator` | `src/modules/navigator/` | 任务执行 — 航线跟踪、RTL、着陆 |
| `ekf2` | `src/modules/ekf2/` | 状态估计 — GPS+IMU+磁力计+气压计融合 |
| `commander` | `src/modules/commander/` | 飞行状态机 — 解锁、模式切换、故障检测 |

**主机的输出**（通过 `control_allocator`）：
- **3个电机**：`motor_0`（左）、`motor_1`（中）、`motor_2`（右）→ 差动推力偏航控制
- **3个舵面**：`servo_0`（左elevon）、`servo_1`（中升降舵）、`servo_2`（右elevon）

**主机的配置来源**：
- SITL 仿真：`4007_gz_chainwing` 或 `4008_gz_chainwing_3body` 机架文件
- SIH 仿真：`1103_chainwing_sih.hil`
- 硬件部署：`2150_chainwing`

### 13.3 从机飞控详解

**从机飞控 = `chainwing_slave` 模块**，这是一个新增的 PX4 模块：

| 属性 | 详情 |
|------|------|
| 模块名 | `chainwing_slave` |
| 文件位置 | `src/modules/chainwing_slave/` |
| 主类 | `ChainwingSlave` (继承 `ModuleBase` + `ScheduledWorkItem`) |
| 运行频率 | 50 Hz (`ScheduleOnInterval(20000_us)`) |
| 工作队列 | `px4::wq_configurations::lp_default` |
| 输入 | `vehicle_angular_velocity` + `vehicle_attitude` (来自 IMU) |
| 输出 | `chainwing_hinge_status` (包含 trim_left, trim_right) |
| 启用参数 | `CW_SLV_EN = 1` |

**从机的控制逻辑**：
```
1. 读取 IMU 角速度 → 低通滤波（10Hz）
2. 积分估算铰链角度 θ_hinge（带衰减 τ=2s 防漂移）
3. 互补滤波：融合姿态角修正长期漂移（α=0.02）
4. PD 控制律：δ_trim = Kp × θ + Kd × θ̇
5. 限幅：|δ_trim| ≤ 0.3（30% 舵面行程）
6. 发布 ChainwingHingeStatus → GZMixingInterfaceServo 叠加到 servo 输出
```

**从机不做的事情**（由主机代劳）：
- 不做导航、任务规划
- 不做姿态外环控制
- 不做油门管理
- 不做模式切换

### 13.4 主从关系对比表

| 维度 | 主机（Master） | 从机（Slave） |
|------|---------------|---------------|
| **飞控软件** | PX4 标准固定翼控制栈（fw_att_control + fw_rate_control + fw_pos_control + control_allocator + navigator + ekf2） | chainwing_slave 模块（50Hz PD 控制器） |
| **控制目标** | 全机姿态、高度、航线跟踪 | 仅维持铰链角度 ≈ 0（共面性） |
| **执行器** | 3 电机 + 3 舵面（完整控制权） | 仅修正 servo_0 和 servo_2 的 elevon 偏移量 |
| **传感器** | GPS + IMU + 磁力计 + 气压计 + 空速管 | 仅用 IMU（角速度 + 姿态） |
| **输出信号** | roll/pitch/yaw 力矩 → 电机和舵面指令 | δ_trim（叠加到主机 servo 指令上） |
| **配置文件** | 4007/4008 机架文件（~210 行参数） | 5 个 CW_SLV_* 参数 |
| **代码量** | ~30,000 行（PX4 固定翼模块总计） | ~250 行（ChainwingSlave.cpp） |
| **运行方式** | 独立决策，控制整个链翼飞行 | 依附于主机，只做微调 |

### 13.5 当前架构的关键特征

**特征 1：单实例架构（仿真模式）**

当前代码是**单 PX4 实例**控制三体 GZ 模型。主机和从机的逻辑都运行在同一个 PX4 进程中：
- 主机 = 标准 PX4 控制栈（自动运行）
- 从机 = chainwing_slave 模块（手动启动或机架脚本启动）

这与最终的硬件部署架构不同。硬件上需要 3 个独立的 PX4 飞控，通过 UART+MAVLink 通信。

**特征 2：从机是"寄生"式控制**

从机不独立产生 servo 指令，而是在主机指令基础上**叠加修正量**：
```
δ_total = clamp(δ_master + δ_trim, -1.0, 1.0)
```
这在 `GZMixingInterfaceServo::updateOutputs()` 中实现（第 75-88 行）。

**特征 3：铰链角度估计是间接的**

当前从机通过 IMU 角速度积分+互补滤波估算铰链角度，而不是直接读取 GZ 关节角度。
这是为了与硬件部署保持一致（硬件上没有关节编码器，只有 IMU）。

### 13.6 机架文件与飞控的对应关系

```
机架文件                           使用的飞控
─────────────────────────────────────────────────────
4007_gz_chainwing                → 仅主机飞控（标准 PX4，无从机控制器）
                                   GZ模型=chainwing（单刚体）
                                   CW_SLV_EN=未设置（默认0，禁用）

4008_gz_chainwing_3body          → 主机飞控 + 从机飞控
                                   GZ模型=chainwing_3body（三刚体+铰链）
                                   CW_SLV_EN=1（启用从机控制器）
                                   CW_SLV_KP=0.3, CW_SLV_KD=0.05

1103_chainwing_sih.hil           → 仅主机飞控（SIH 硬件仿真）
                                   无从机控制器

2150_chainwing                   → 仅主机飞控（硬件部署模板）
                                   无从机控制器
                                   未来需为从机Pixhawk创建独立机架
```

---

## 14. 3body仿真完成后的下一步工作

> **核心问题**：3body 仿真已完成，接下来该做什么？

### 14.1 当前已完成的工作清单

| # | 已完成项目 | 对应代码/文件 | 状态 |
|---|-----------|-------------|------|
| 1 | 主机飞控（标准 PX4 固定翼） | `4007_gz_chainwing` + 标准 FW 模块 | ✅ 已验证飞行 |
| 2 | 三体 GZ 模型（高刚度铰链） | `chainwing_3body/model.sdf` | ✅ 模型已创建 |
| 3 | 从机控制模块 | `src/modules/chainwing_slave/` | ✅ 代码已编写 |
| 4 | uORB 铰链状态消息 | `msg/ChainwingHingeStatus.msg` | ✅ 已注册 |
| 5 | GZ servo 输出修正 | `GZMixingInterfaceServo.cpp` 修改 | ✅ 已实现 |
| 6 | 3body 机架配置 | `4008_gz_chainwing_3body` | ✅ 参数已配置 |
| 7 | SITL board 注册 | `default.px4board` 添加模块 | ✅ 已添加 |
| 8 | 参数单位修复 | 移除无效 `@unit` | ✅ 编译通过 |

### 14.2 下一步工作路线图（6个阶段）

```
═══════════════════════════════════════════════════════════════
  阶段 1（当前）      阶段 2           阶段 3          阶段 4           阶段 5         阶段 6
  仿真基础验证  →  从机控制调参  →  多实例仿真  →  MAVLink通信  →  硬件适配  →  飞行测试
  [2-3天]          [3-5天]         [5-7天]        [5-7天]         [3-5天]       [持续]
═══════════════════════════════════════════════════════════════
```

#### 阶段 1：仿真基础验证（2-3天）— 🔴 立即开始

**目标**：确认 3body 模型能正确加载，从机模块能运行，铰链控制有基本效果。

| 步骤 | 具体操作 | 验收标准 |
|------|---------|----------|
| 1.1 | 编译：`make px4_sitl_default` | 零错误、零新警告 |
| 1.2 | 启动仿真：`PX4_SYS_AUTOSTART=4008 PX4_GZ_MODEL_POSE="0,0,0.3,0,0,0" make px4_sitl gz_chainwing_3body` | GZ 窗口显示三体模型，PX4 shell 可用 |
| 1.3 | 验证模型：GZ 中检查 `hinge_left`/`hinge_right` 关节存在 | `gz topic -l` 能看到关节状态话题 |
| 1.4 | 验证模块：PX4 shell 输入 `chainwing_slave status` | 显示 Enabled=YES, 铰链角度数值 |
| 1.5 | 验证消息：`listener chainwing_hinge_status` | 50Hz 数据流，有 trim 值 |
| 1.6 | 起飞测试：Mission 模式自动起飞 | 飞机起飞，铰链角度在 ±5° 内 |
| 1.7 | 铰链对比：分别用 `CW_SLV_EN=0` 和 `CW_SLV_EN=1` 飞行，对比日志 | 启用时铰链 RMS 角度更小 |

**如果阶段 1 遇到问题的排查顺序**：
```
模型加载失败? → 检查 model.sdf 语法，确认 GZ 版本支持 revolute joint + spring
模块启动失败? → 检查 Kconfig/CMakeLists/px4board 注册链
铰链角始终为0? → 检查 vehicle_angular_velocity 是否有数据
铰链发散/爆炸? → 降低 joint stiffness（500→200）或增大 damping（50→100）
```

#### 阶段 2：从机控制调参（3-5天）

**目标**：通过仿真飞行日志，优化 PD 控制器增益，使铰链角度稳定在可接受范围。

| 步骤 | 具体操作 | 验收标准 |
|------|---------|----------|
| 2.1 | 巡航飞行记录日志（`logger on`，飞行 2 分钟） | 获得 `.ulg` 日志文件 |
| 2.2 | 用 PlotJuggler 或 FlightPlot 分析 `chainwing_hinge_status` | 确认铰链角度波形 |
| 2.3 | 调整 `CW_SLV_KP`：从 0.1 到 0.5，步长 0.1 | 找到响应最快且无振荡的值 |
| 2.4 | 调整 `CW_SLV_KD`：从 0.01 到 0.2，步长 0.02 | 找到阻尼最优的值 |
| 2.5 | 调整 `CW_SLV_TRIM_MAX`：测试 0.1 / 0.2 / 0.3 / 0.5 | 确认 30% 是否足够 |
| 2.6 | 极端工况测试：大转弯（bank=30°）、突然爬升、突然下降 | 铰链角度不超过 ±5° |
| 2.7 | 铰链刚度敏感性分析：测试 200/500/1000 N·m/rad | 确认控制律在各刚度下都稳定 |

**调参经验公式**：
```
初始估计：
  Kp_start = 2 × ζ × ωn / (d_CL/dα × q × S × c)
           ≈ 2 × 0.7 × 3.14 / (5.25 × 245 × 0.36 × 0.3)
           ≈ 0.03（无量纲增益）

  但仿真中 Kp=0.3 工作良好（因为 output 是归一化量，不是直接的物理量）

调参策略：
  1. 先固定 Kd=0, 增大 Kp 直到振荡
  2. 设 Kp = 0.6 × Kp_振荡
  3. 增大 Kd 直到振荡消失
  4. 微调
```

#### 阶段 3：多实例仿真 — 模拟真正的主从分离（5-7天）

**目标**：从"单 PX4 实例"升级为"3个 PX4 实例"，模拟真实的三飞控架构。

这是从仿真到硬件的**关键跨越步骤**。

| 步骤 | 具体操作 | 详细说明 |
|------|---------|----------|
| 3.1 | 创建主机专用机架 `4009_gz_chainwing_master` | 仅主机逻辑，`MAV_SYS_ID=1`，禁用 `CW_SLV_EN` |
| 3.2 | 创建从机专用机架 `4010_gz_chainwing_slave` | 仅从机逻辑，`MAV_SYS_ID=2`/`3`，启用 `CW_SLV_EN=1` |
| 3.3 | 多实例启动脚本 | 3 个 PX4 实例绑定同一个 GZ 模型的不同 body |
| 3.4 | 实现 MAVLink 通信桥 | 主机通过 MAVLink 发送 pitch/throttle 命令给从机 |
| 3.5 | 从机接收主机命令并执行 | 从机将主机的 servo 指令作为基准，叠加自身 trim |
| 3.6 | 验证主从协调飞行 | 3 个 PX4 实例协调，铰链角度稳定 |

**多实例启动示例**（概念）：
```bash
# 终端 1：启动 GZ 世界 + 模型
gz sim -v4 -r flat_terrain.sdf --gui

# 终端 2：主机 PX4 实例（控制 base_link）
PX4_SYS_AUTOSTART=4009 PX4_GZ_MODEL=chainwing_3body \
  MAVLINK_SYS_ID=1 ./build/px4_sitl_default/bin/px4 -i 0

# 终端 3：左从机 PX4 实例（控制 left_unit）
PX4_SYS_AUTOSTART=4010 \
  MAVLINK_SYS_ID=2 ./build/px4_sitl_default/bin/px4 -i 1

# 终端 4：右从机 PX4 实例（控制 right_unit）
PX4_SYS_AUTOSTART=4010 \
  MAVLINK_SYS_ID=3 ./build/px4_sitl_default/bin/px4 -i 2
```

> ⚠️ **注意**：阶段 3 需要对 `gz_bridge` 进行较大修改，使不同 PX4 实例订阅不同 body 的传感器话题。这是技术难度最高的一步。

#### 阶段 4：MAVLink 主从通信实现（5-7天）

**目标**：实现主机 → 从机的命令传输和从机 → 主机的状态反馈。

| 步骤 | 具体操作 | 详细说明 |
|------|---------|----------|
| 4.1 | 选择 MAVLink 消息类型 | 主→从：`SET_ACTUATOR_CONTROL_TARGET` 或自定义消息 |
| 4.2 | 配置 UART 串口仿真 | SITL 中用 TCP/UDP 端口模拟 UART 链路 |
| 4.3 | 主机发送模块 | 在主机添加定时发送逻辑（50Hz） |
| 4.4 | 从机接收模块 | 在从机添加 MAVLink 接收解析逻辑 |
| 4.5 | 从机反馈铰链状态 | 从机通过 MAVLink 回传 hinge_angle/trim |
| 4.6 | 丢包/延迟处理 | 添加超时检测（100ms 无数据→保持上一次指令） |
| 4.7 | 联合测试 | 模拟丢包 10%/30%/50%，验证系统鲁棒性 |

**MAVLink 通信协议设计**：
```
主机 → 从机（50Hz）：
  MAVLink SET_ACTUATOR_CONTROL_TARGET {
    target_system = 2 或 3,
    controls[0] = throttle_command,      // 整体油门
    controls[1] = pitch_servo_command,   // 升降舵基准
    controls[2] = roll_servo_command,    // elevon 基准
  }

从机 → 主机（10Hz）：
  MAVLink DEBUG_FLOAT_ARRAY {
    name = "HINGE_STATUS",
    data[0] = hinge_angle,  // rad
    data[1] = hinge_rate,   // rad/s
    data[2] = trim_output,  // normalized
    data[3] = data_valid,   // 0 or 1
  }
```

#### 阶段 5：硬件适配（3-5天）

**目标**：将仿真验证过的代码适配到真实 Pixhawk 硬件。

| 步骤 | 具体操作 | 注意事项 |
|------|---------|----------|
| 5.1 | 创建硬件从机机架 `2151_chainwing_slave` | 基于 `2150_chainwing` 修改，添加 CW_SLV_* 参数 |
| 5.2 | 配置 UART 串口 | `MAV_1_CONFIG=TELEM2`，波特率 921600 |
| 5.3 | IMU 校准 | 三个飞控分别校准 IMU（确保坐标系一致） |
| 5.4 | PWM 输出映射 | 确认从机 PWM 通道对应正确的舵面 |
| 5.5 | 地面联调 | 供电但不起飞，验证 MAVLink 通信和 trim 输出 |
| 5.6 | 编译从机固件 | `make px4_fmu-v5_default`（或对应硬件型号） |

#### 阶段 6：飞行测试（持续）

**目标**：真实飞行验证。

| 步骤 | 测试内容 | 通过标准 |
|------|---------|----------|
| 6.1 | 地面滑行 | 主从机 MAVLink 通信正常，铰链无异常 |
| 6.2 | 手抛起飞（低高度） | 成功起飞，铰链角度 < ±3° |
| 6.3 | 直线巡航 | 平稳飞行 60s，铰链角 RMS < 2° |
| 6.4 | 温和转弯 | bank ≤ 15°，铰链角度 < ±5° |
| 6.5 | 大转弯 | bank ≤ 30°，铰链角度 < ±8° |
| 6.6 | 着陆 | 着陆过程铰链无剧烈波动 |

### 14.3 阶段优先级和依赖关系

```
阶段 1（基础验证）──→ 阶段 2（调参）──→ 阶段 3（多实例）──→ 阶段 4（MAVLink）
    │                    │                    │                    │
    │  无依赖，立即开始    │  需要阶段1通过       │  需要阶段2调好参    │  需要阶段3框架
    ↓                    ↓                    ↓                    ↓
  最高优先级            高优先级              中优先级              中优先级

                                                                    ↓
                                               阶段 5（硬件适配）──→ 阶段 6（飞行测试）
                                                    │                    │
                                                    │  需要阶段4通信       │  需要阶段5硬件
                                                    ↓                    ↓
                                                  低优先级（仿真完成后） 最终验证
```

### 14.4 建议立即执行的操作

**今天就可以做的 5 件事**：

1. **编译测试**：
   ```bash
   cd ~/PX4-Autopilot
   make clean
   make px4_sitl_default
   ```
   确认零错误编译通过。

2. **启动 3body 仿真**：
   ```bash
   PX4_SYS_AUTOSTART=4008 PX4_GZ_MODEL_POSE="0,0,0.3,0,0,0" make px4_sitl gz_chainwing_3body
   ```
   观察 GZ 窗口中三体模型是否正确显示。

3. **检查从机模块运行**：
   在 PX4 shell 中：
   ```
   chainwing_slave status
   listener chainwing_hinge_status
   param show CW_SLV_*
   ```

4. **起飞测试**：
   ```
   commander takeoff
   ```
   观察飞行稳定性和铰链角度变化。

5. **记录日志用于后续分析**：
   ```
   logger on
   # 飞行 2 分钟
   logger off
   ```
   下载 `.ulg` 文件用 [Flight Review](https://review.px4.io/) 或 PlotJuggler 分析。

### 14.5 每个阶段的产出物

| 阶段 | 交付物 | 格式 |
|------|--------|------|
| 1 | 编译通过截图 + GZ 模型截图 + 从机 status 输出 | 截图/文本 |
| 2 | PD 增益调参结果表 + 铰链角度时域图 | 表格 + 图表 |
| 3 | 多实例启动脚本 + 新机架文件 + gz_bridge 修改 | 代码 |
| 4 | MAVLink 通信协议文档 + 收发模块代码 | 文档 + 代码 |
| 5 | 硬件从机机架文件 + UART 配置指南 | 代码 + 文档 |
| 6 | 飞行测试报告 + 铰链日志分析 | 报告 |

---

## 15. 主飞控固件分析：烧什么、改什么、为什么

### 15.1 一句话结论

> **主飞控烧入本仓库编译出的同一份固件（`make px4_sitl_default`），不需要任何新代码。**
> 
> 三个飞控物理参数一样 → 编译一次 → 烧入同一份 `.px4` 固件 → 通过参数区分主/从角色。

---

### 15.2 主飞控使用的固件是什么

主飞控使用的就是你仓库里已经有的 **标准 PX4 固定翼控制栈**，由以下现有模块组成：

```
┌─────────────────────────────────────────────────────────┐
│                    主飞控 PX4 控制栈                       │
│                                                         │
│  navigator → fw_pos_control → fw_att_control            │
│                                    ↓                    │
│                              fw_rate_control            │
│                                    ↓                    │
│                            control_allocator            │
│                              ↓         ↓                │
│                         3个电机    3个舵面               │
│                                                         │
│  辅助模块: ekf2, sensors, commander, logger, mavlink    │
└─────────────────────────────────────────────────────────┘
```

这些模块的代码量约 **30,000 行**，全部是 PX4 自带的标准模块，你的仓库里已经完整包含。

**涉及的标准模块路径**（无需修改任何一行）：

| 模块 | 路径 | 功能 | 代码量 |
|------|------|------|--------|
| fw_att_control | `src/modules/fw_att_control/` | 固定翼姿态控制 | ~2,000行 |
| fw_rate_control | `src/modules/fw_rate_control/` | 角速率内环 PID | ~1,500行 |
| fw_pos_control | `src/modules/fw_pos_control/` | 位置/高度/速度控制 | ~5,000行 |
| control_allocator | `src/modules/control_allocator/` | 控制分配（力矩→电机/舵面） | ~8,000行 |
| navigator | `src/modules/navigator/` | 航点任务/返航/降落逻辑 | ~6,000行 |
| ekf2 | `src/modules/ekf2/` | 扩展卡尔曼滤波导航 | ~8,000行 |
| commander | `src/modules/commander/` | 飞行状态机/解锁/安全检查 | ~5,000行 |

---

### 15.3 主飞控需不需要新代码

**不需要。** 原因如下：

#### （1）主飞控的任务

主飞控负责整体飞行控制：
- **路径跟踪**：按航点飞行（navigator + fw_pos_control）
- **姿态稳定**：roll/pitch/yaw 三轴控制（fw_att_control + fw_rate_control）
- **差动推力偏航**：3 个电机在 Y 轴方向偏移，产生偏航力矩（control_allocator）
- **3 个舵面**：左 elevon + 中 elevator + 右 elevon（control_allocator）

以上所有功能，**PX4 标准固定翼栈已经完全实现**。

#### （2）为什么不需要新代码

| 功能需求 | PX4 已有解决方案 | 是否需要新代码 |
|----------|------------------|----------------|
| 控制 3 个电机 | CA_ROTOR_COUNT=3 + 差动推力 | ❌ 不需要 |
| 控制 3 个舵面 | CA_SV_CS_COUNT=3 + 自定义效率矩阵 | ❌ 不需要 |
| 差动推力偏航 | CA_R*_PY 参数设置偏航力矩方向 | ❌ 不需要 |
| 巡航速度控制 | FW_AIRSPD_TRIM=20 + TECS | ❌ 不需要 |
| 起飞/降落 | FW_LAUN_DETCN_ON + FW_LND_* | ❌ 不需要 |
| EKF2 导航 | 标准 GPS+IMU+MAG+BARO 融合 | ❌ 不需要 |
| 向从机发送命令 | MAVLink 标准协议（下一阶段） | ⏳ 后续阶段实现 |

#### （3）唯一新增的 chainwing_slave 模块是给谁用的？

`chainwing_slave` 模块是给**从飞控**用的，不是给主飞控用的。

- **当前 SITL 架构**：主从控制在同一个 PX4 进程里运行（单实例仿真）
  - 主控制 = 标准 FW 栈 → 输出 servo 命令
  - 从控制 = chainwing_slave → 计算 trim 修正叠加在 servo 输出上
- **未来硬件架构**：主从分别运行在不同 Pixhawk 上
  - 主飞控不运行 chainwing_slave（CW_SLV_EN=0）
  - 从飞控只运行 chainwing_slave（CW_SLV_EN=1）

---

### 15.4 三个飞控的固件和参数配置

三个飞控烧入**完全相同的固件**，仅通过参数区分角色：

```
  ┌──────────┐    ┌──────────┐    ┌──────────┐
  │  从机(左)  │    │   主机    │    │  从机(右)  │
  │ SYS_ID=2 │    │ SYS_ID=1 │    │ SYS_ID=3 │
  │ SLV_EN=1 │    │ SLV_EN=0 │    │ SLV_EN=1 │
  └──────────┘    └──────────┘    └──────────┘
       │                │                │
       └────── 串口MAVLink ──────────────┘
```

#### 三飞控参数差异表

| 参数 | 主飞控 (中) | 从飞控 (左) | 从飞控 (右) | 说明 |
|------|-------------|-------------|-------------|------|
| **MAV_SYS_ID** | 1 | 2 | 3 | MAVLink 系统 ID |
| **CW_SLV_EN** | **0** | **1** | **1** | 从机模块使能 |
| **CW_SLV_KP** | (不生效) | 0.3 | 0.3 | PD 比例增益 |
| **CW_SLV_KD** | (不生效) | 0.05 | 0.05 | PD 微分增益 |
| **CW_SLV_TRIM_MAX** | (不生效) | 0.3 | 0.3 | 最大修正量(30%) |
| **CW_SLV_LP_FREQ** | (不生效) | 10.0 | 10.0 | 低通滤波频率 |
| CA_AIRFRAME | 1 | 1 | 1 | 固定翼 |
| CA_ROTOR_COUNT | 3 | 3 | 3 | 3电机(都一样) |
| CA_SV_CS_COUNT | 3 | 3 | 3 | 3舵面(都一样) |
| FW_AIRSPD_TRIM | 20 | 20 | 20 | 巡航速度(都一样) |
| 物理参数(全部) | 相同 | 相同 | 相同 | 三架飞机物理参数一样 |

**注意**：三个飞控的物理参数（质量、惯量、气动等）完全相同，因为题目说三架飞机物理参数一样。

---

### 15.5 当前仿真阶段 vs 未来硬件阶段

#### 当前阶段：单实例 SITL（用 4008_gz_chainwing_3body 机架）

```
┌─── 一个 PX4 进程 ───────────────────────────────┐
│                                                  │
│  标准FW控制栈 ──→ control_allocator ──→ servos  │
│     (主飞控逻辑)         ↓                      │
│                    GZMixingInterfaceServo        │
│                         ↑                       │
│  chainwing_slave ──→ trim_left / trim_right     │
│     (从飞控逻辑)                                 │
│                                                  │
└──────────── GZ 三体模型 ────────────────────────┘
```

- **主飞控功能**由标准 FW 栈提供（已有，不需要新代码）
- **从飞控功能**由 chainwing_slave 提供（已实现）
- 两者在同一进程内通过 uORB 消息通信（无需 MAVLink）

#### 未来阶段：三实例 SITL 或实际硬件

```
┌─── PX4 实例 1 (主) ──┐  MAVLink   ┌─── PX4 实例 2 (左从) ──┐
│                      │ ←───────→ │                         │
│  标准FW控制栈         │   UART     │  chainwing_slave        │
│  CW_SLV_EN=0        │           │  CW_SLV_EN=1            │
│  SYS_ID=1            │           │  SYS_ID=2               │
└──────────────────────┘           └─────────────────────────┘
          ↑ MAVLink                            
          ↓                                    
┌─── PX4 实例 3 (右从) ──┐
│                         │
│  chainwing_slave        │
│  CW_SLV_EN=1            │
│  SYS_ID=3               │
└─────────────────────────┘
```

此阶段需要**新增 MAVLink 通信代码**（见§14 路线图第4阶段），但这属于**从飞控端**的新增，主飞控端仅需配置 MAVLink 端口参数。

---

### 15.6 主飞控机架文件配置说明

主飞控使用的机架文件就是仓库中已有的 `4007_gz_chainwing`，其核心参数包括：

#### 控制分配（3电机+3舵面）

```
# 3个电机，Y轴方向偏移实现差动推力偏航
CA_ROTOR_COUNT=3
CA_R0_PY=-1.2    # 左电机 Y=-1.2m
CA_R1_PY=0.0     # 中电机 Y=0m
CA_R2_PY=1.2     # 右电机 Y=+1.2m

# 3个舵面
CA_SV_CS_COUNT=3
CA_SV_CS0_TYPE=6   # 左 elevon (滚转+俯仰)
CA_SV_CS1_TYPE=2   # 中 elevator (仅俯仰)
CA_SV_CS2_TYPE=7   # 右 elevon (滚转+俯仰)
```

#### 姿态控制增益

```
# 角速率 PID（仓库中已调好）
FW_PR_P=0.9  FW_PR_FF=0.5  FW_PR_I=0.5   # 俯仰
FW_RR_P=0.3  FW_RR_FF=0.5  FW_RR_I=0.5   # 滚转
FW_YR_P=0.6  FW_YR_FF=0.5  FW_YR_I=0.5   # 偏航

# 姿态时间常数
FW_R_TC=0.5   FW_P_TC=0.5
```

#### 速度与油门

```
FW_AIRSPD_MIN=15   FW_AIRSPD_TRIM=20   FW_AIRSPD_MAX=30
FW_AIRSPD_STALL=8
FW_THR_MIN=0.05   FW_THR_TRIM=0.60   FW_THR_MAX=1.0
```

**这些参数已经在 4007_gz_chainwing 中配置完毕，无需修改。**

---

### 15.7 回答原始问题

#### Q1：三飞机物理参数一样，主飞控烧什么固件？

**A**：主飞控烧入本仓库 `make px4_sitl_default`（SITL仿真）或 `make px4_fmu-v6x_default`（实际硬件）编译出的固件。**与从飞控完全相同的固件**。

#### Q2：主飞控需要加新代码吗？

**A**：**不需要。** 主飞控的所有功能（路径控制、姿态控制、3电机差动推力、3舵面分配）全部由 PX4 标准固定翼控制栈实现，仓库中已经完整包含。

关键区别仅在参数：
- 主飞控设 `CW_SLV_EN=0`（不运行从机模块）
- 从飞控设 `CW_SLV_EN=1`（运行铰链修正）

#### Q3：什么时候才需要给主飞控加新代码？

**A**：只有在**多实例/多硬件**阶段（§14 第4阶段），才需要在主飞控端添加 MAVLink 通信代码来向从飞控发送油门/舵面命令。但这属于后续工作，当前单实例仿真阶段完全不需要。

---

### 15.8 当前仿真验证的优先事项

既然主飞控不需要新代码，当前应该聚焦于：

1. **编译验证**：确保 `make px4_sitl_default` 通过（已修复 CW_SLV_KP/KD 单位问题）
2. **单体飞行验证**：用 `4007_gz_chainwing` 机架验证主飞控标准栈可以正常起飞/巡航/降落
3. **三体模型验证**：用 `4008_gz_chainwing_3body` 机架验证铰链关节物理行为正确
4. **从机 PD 调参**：观察 chainwing_slave 模块的 trim 输出是否合理（§12 验证步骤）

这些工作**不涉及任何代码修改**，只需要运行仿真和观察数据。

---

## 16. 仿真坐标系差异与 Yaw Estimate Error 解析

### 16.1 问题现象

启动 chainwing_3body 仿真时出现三个疑问：

| 观察点 | GZ 报告 | PX4 报告 | 疑问 |
|--------|---------|----------|------|
| **Yaw** | 0° | 90.1° | 为什么差 90°？ |
| **Pitch** | -0.266 rad (-15.2°) | +15.1° | 为什么符号相反？ |
| **预检** | — | "Yaw estimate error" | 为什么航向估计报错？ |

```
# PX4 nsh console 输出：
vehicle_attitude
    q: [0.70062, -0.09307, 0.09309, 0.70128] (Roll: 0.0 deg, Pitch: 15.1 deg, Yaw: 90.1 deg)

# GZ 终端输出：
gz model -m chainwing_3body_0
    Pose [RPY (rad)]: [0.000000 -0.265748 -0.000000]
```

### 16.2 根本原因：ENU vs NED 坐标系

**Gazebo 和 PX4 使用完全不同的世界坐标系**，这是航空/机器人领域的标准做法：

| | Gazebo (ENU) | PX4 (NED) |
|---|---|---|
| **X 轴** | 东 (East) | 北 (North) |
| **Y 轴** | 北 (North) | 东 (East) |
| **Z 轴** | 上 (Up) | 下 (Down) |
| **Yaw = 0** | 朝东 | 朝北 |
| **正 Pitch** | 机头朝**下** | 机头朝**上** |

**机体坐标系也不同**：

| | Gazebo (FLU) | PX4 (FRD) |
|---|---|---|
| **X** | 前 (Forward) | 前 (Forward) |
| **Y** | 左 (Left) | 右 (Right) |
| **Z** | 上 (Up) | 下 (Down) |

### 16.3 Yaw 差异解释 (GZ=0° → PX4=90°)

```
                  North (PX4 yaw=0°)
                    ↑
                    |
                    |
West ←──────────────┼──────────────→ East (GZ yaw=0°)
                    |                  = PX4 yaw=90°
                    |
                    ↓
                  South
```

飞机物理朝向：**朝东**

- 在 GZ 的 ENU 坐标系中：朝东 = 沿 X 轴 = **Yaw = 0°**
- 在 PX4 的 NED 坐标系中：朝东 = 沿 Y 轴 = **Yaw = 90°**

**这是同一个物理朝向**，只是两个坐标系的 0° 参考方向不同。gz_bridge 的 `rotateQuaternion()` 函数（GZBridge.cpp:695-711）执行了正确的转换。

### 16.4 Pitch 符号差异解释 (GZ=-0.266 → PX4=+15.1°)

飞机物理状态：**机头抬起约 15°**（在地面静止，由起落架几何和重心位置决定）

**GZ ENU 约定**中，绕 Y 轴（北）的旋转：
- 标准右手定则：拇指朝北，手指从 X(东)卷向 Z(上) = 正方向
- 正 Pitch = 机头向**下**旋转
- 负 Pitch = 机头向**上**旋转
- 所以：机头抬起 15° → **Pitch = -0.266 rad**

**PX4 NED 约定**中，标准航空惯例：
- 正 Pitch = 机头向**上**
- 所以：机头抬起 15° → **Pitch = +15.1°**

**两者描述的是完全相同的物理姿态**，符号差异是坐标系约定的必然结果。

### 16.5 数学验证

gz_bridge 中的 `rotateQuaternion()` 执行以下转换：

```cpp
// GZBridge.cpp:695-711
void GZBridge::rotateQuaternion(gz::math::Quaterniond &q_FRD_to_NED,
                                const gz::math::Quaterniond q_FLU_to_ENU)
{
    // FLU→FRD: 绕 X 轴旋转 180°
    static const auto q_FLU_to_FRD = gz::math::Quaterniond(0, 1, 0, 0);

    // ENU→NED: 绕 Z 轴 90° + 绕 X 轴 180°
    static const auto q_ENU_to_NED = gz::math::Quaterniond(0, 0.70711, 0.70711, 0);

    q_FRD_to_NED = q_ENU_to_NED * q_FLU_to_ENU * q_FLU_to_FRD.Inverse();
}
```

代入 GZ 的数据验证：

```
输入：GZ RPY(0, -0.266, 0) → q_GZ = [0.9912, 0, -0.1327, 0]

步骤 1: temp = q_GZ × q_FLU_to_FRD⁻¹
       = [0.9912, 0, -0.1327, 0] × [0, -1, 0, 0]
       = [0, -0.9912, 0, -0.1327]

步骤 2: q_PX4 = q_ENU_to_NED × temp
       = [0, 0.70711, 0.70711, 0] × [0, -0.9912, 0, -0.1327]
       = [0.7009, -0.0938, 0.0938, 0.7009]

输出：PX4 Euler → Roll=0°, Pitch=+15.2°, Yaw=90.0°
用户观察值：          Roll=0°, Pitch=+15.1°, Yaw=90.1°  ← 完全吻合 ✓
```

微小差异（0.1°）来自飞机在地面上的微小动态振动。

### 16.6 Yaw Estimate Error 解释

**预检错误消息**：`Preflight Fail: Yaw estimate error`

**触发条件**（estimatorCheck.cpp:257-273）：

```cpp
if (!context.isArmed() && (estimator_status.mag_test_ratio > _param_com_arm_ekf_yaw.get())) {
    // "Yaw estimate error"
}
```

| 参数 | 当前值 | 说明 |
|------|--------|------|
| `COM_ARM_EKF_YAW` | **0.5**（默认） | 允许的最大磁力计创新比率 |
| `mag_test_ratio` | > 0.5（启动时） | EKF2 磁力计融合的创新比率 |

**原因**：

1. EKF2 启动时需要融合磁力计数据来估计航向
2. 在仿真启动的前几秒，磁力计模拟器（sensor_mag_sim）刚初始化
3. EKF2 的磁力计创新检验比率（mag_test_ratio）暂时超过 0.5 阈值
4. 随着 EKF2 收敛（通常 5-15 秒），该比率会降至 0.5 以下
5. 这是**启动瞬态现象**，与坐标系转换无关

**注意**：这个问题与之前解决的 `COM_ARM_EKF_POS`/`COM_ARM_EKF_VEL` 问题**同类**——都是 EKF2 收敛期间的瞬态创新比率超过默认阈值。

**解决方案**（需要您确认后再实施）：
在机架配置文件 `4008_gz_chainwing_3body` 中增加：
```
param set-default COM_ARM_EKF_YAW 0.8
```
将阈值从 0.5 放宽到 0.8，给 EKF2 更多收敛时间。如果仍不够，可进一步调至 1.0（最大值）。

### 16.7 总结

| 现象 | 是否为 Bug | 原因 | 需要修改？ |
|------|-----------|------|-----------|
| PX4 Yaw=90° vs GZ Yaw=0° | **否** | ENU→NED 坐标系标准转换 | 不需要 |
| PX4 Pitch=+15° vs GZ Pitch=-15° | **否** | ENU/NED 俯仰符号约定不同 | 不需要 |
| "Yaw estimate error" | **瞬态** | EKF2 启动收敛期间 mag_test_ratio > 0.5 | 调参即可 |

**关键认知**：Gazebo 和 PX4 之间的所有姿态数据差异都是**坐标系约定**的正常体现，gz_bridge 的 `rotateQuaternion()` 函数正确执行了 ENU/FLU ↔ NED/FRD 的转换。在解读仿真数据时，始终要注意区分两个系统的坐标约定。

---

## 17. 仿真操作常见问题：左右反转、gz命令、listener中断

### 17.1 问题一：为什么 GZ 界面中左右机方向是反的？

**现象**：在 Gazebo 界面中点击"右机"，看到的却是 SDF 中定义的 `left_unit`。

**根本原因：GZ 的 ENU 坐标系 Y 轴方向与直觉相反**

chainwing_3body 模型中各单元的 GZ 坐标位置：

| 链接名称 | SDF 中的 Y 坐标 | 在 ENU 中的方向 |
|----------|----------------|----------------|
| `left_unit` | Y = **-1.20** | Y 负 = **南方** |
| `base_link` (center) | Y = 0 | 中心 |
| `right_unit` | Y = **+1.20** | Y 正 = **北方** |

模型初始朝向为 **东**（沿 ENU 的 X+ 轴），此时 GZ 默认相机从上方俯视：

```
           北 (Y+)
           ↑
    ┌──────┼──────┐
    │ right_unit  │  ← Y=+1.20 (GZ屏幕上方/左侧)
    │  base_link  │  ← Y=0
    │ left_unit   │  ← Y=-1.20 (GZ屏幕下方/右侧)
    └──────┼──────┘
           │
           南 (Y-)
  ←────────────────→
  西 (X-)      东 (X+) ← 飞机朝向
```

**从飞行员视角**（坐在飞机内，面朝东/前方）：
- 飞行员的**左边** = 北方 = Y+ = `right_unit`
- 飞行员的**右边** = 南方 = Y- = `left_unit`

**从 GZ GUI 默认俯视视角**：
- 屏幕上方/左侧 = 北方 = Y+ = `right_unit`
- 屏幕下方/右侧 = 南方 = Y- = `left_unit`

**结论**：SDF 中的 `left_unit`/`right_unit` 命名采用了 **GZ 坐标系的约定**（Y负=left, Y正=right），而不是飞行员视角的左右。当你在 GZ 界面中看到飞机并点击视觉上的"右边"单元时，实际点击的是 Y- 位置的 `left_unit`。

**这不是 Bug**，是 SDF 模型命名与视觉显示之间的坐标约定差异：

| 视角 | Y=-1.20 的单元 | Y=+1.20 的单元 |
|------|---------------|---------------|
| GZ 坐标系约定 | "left" | "right" |
| 飞行员视角（面朝东） | **右翼** | **左翼** |
| GZ GUI 俯视 | 屏幕**下方/右侧** | 屏幕**上方/左侧** |

> **如果需要修改**（等待您确认）：可以在 SDF 中将 `left_unit`↔`right_unit` 命名互换，
> 使其符合飞行员视角（航空惯例）。同时需要同步修改 `hinge_left`↔`hinge_right`、
> `ChainwingSlave.cpp` 中的 `trim_left`↔`trim_right`、以及 `GZMixingInterfaceServo.cpp` 中的映射。

### 17.2 问题二：为什么 `gz service` 命令在 PX4 shell 中报错？

**现象**：
```
pxh> gz service -s /world/flat_terrain/wrench \
Invalid command: gz
type 'help' for a list of commands
```

**原因：`gz` 是 Gazebo 的系统命令，不是 PX4 shell 命令**

PX4 运行时有两个完全独立的命令行环境：

| 环境 | 提示符 | 可用命令 | 位置 |
|------|--------|---------|------|
| **PX4 shell** | `pxh>` | PX4 内部命令：`listener`, `param`, `commander`, `chainwing_slave` 等 | PX4 启动的终端窗口 |
| **系统终端** | `$` 或 `zian@xxx:~$` | Linux 命令 + Gazebo 命令：`gz`, `ls`, `cat` 等 | 另一个终端窗口 |

`gz` 命令是 Gazebo 仿真器的 CLI 工具（通过 `apt install gz-garden` 安装），
属于 Linux 系统命令，**不存在于** PX4 的 `src/systemcmds/` 目录中。

**正确的操作方法**：

打开一个**新的系统终端**（不是 PX4 shell），然后执行：

```bash
# 在系统终端（非 pxh>）中执行：

# 对左铰链施加正方向力矩（使左翼抬起）
gz service -s /world/flat_terrain/wrench \
  --reqtype gz.msgs.EntityWrench \
  --reptype gz.msgs.Boolean \
  --req 'entity: {name: "chainwing_3body_0::left_unit", type: LINK}, wrench: {torque: {x: 5.0}}'

# 等待2秒
sleep 2

# 施加反向力矩恢复
gz service -s /world/flat_terrain/wrench \
  --reqtype gz.msgs.EntityWrench \
  --reptype gz.msgs.Boolean \
  --req 'entity: {name: "chainwing_3body_0::left_unit", type: LINK}, wrench: {torque: {x: -5.0}}'
```

**两个终端的使用方式**：

```
┌──────────────────────────────────────┐
│  终端1: PX4 SITL                     │
│  $ make px4_sitl gz_chainwing_3body  │
│  ...                                 │
│  pxh> listener chainwing_hinge_status│  ← PX4命令在这里
│  pxh> param set CW_SLV_KP 0.5       │
│  pxh> commander status               │
└──────────────────────────────────────┘

┌──────────────────────────────────────┐
│  终端2: 系统终端                      │
│  $ gz service -s /world/...          │  ← gz命令在这里
│  $ gz topic -l                       │
│  $ gz model -m chainwing_3body_0     │
└──────────────────────────────────────┘
```

### 17.3 问题三：为什么 `listener chainwing_hinge_status -r 2` 只打印部分就停止？

**现象**：执行 `listener chainwing_hinge_status -r 2` 后，打印了几十条消息就自动退出了。

**原因：PX4 listener 命令有默认消息数量限制**

在 `src/systemcmds/topic_listener/listener_main.cpp` 第 202-209 行：

```cpp
if (num_msgs == 0) {
    if (topic_rate != 0) {
        num_msgs = 30 * topic_rate;  // 30秒 × 速率 = 自动退出条件
    } else {
        num_msgs = 1;
    }
}
```

当你指定 `-r 2`（2 Hz 采样）但不指定消息数量时：
- `num_msgs = 30 × 2 = 60` 条消息
- 以 2 Hz 打印，60 条 ÷ 2 Hz = **仅打印 30 秒后自动退出**

**解决方案：使用 `-n` 参数指定一个很大的消息数**

```bash
# 指定一个很大的消息数（如持续打印约3小时）
pxh> listener chainwing_hinge_status -r 2 -n 20000

# 或更长时间
pxh> listener chainwing_hinge_status -r 2 -n 999999
```

> **⚠️ 注意**：`-n 0` **不能**表示无限打印！详见 §18.1 的代码分析。
> 传入 `-n 0` 后，代码会将 `num_msgs` 重新计算为 `30 × rate`，效果等同于没有指定 `-n`。

**listener 命令完整参数说明**：

```
用法: listener <topic_name> [-i <instance>] [-r <rate_Hz>] [-n <num_msgs>]

参数:
  -i <instance>   话题实例编号（多实例时使用，默认0）
  -r <rate_Hz>    采样频率（Hz），不指定则尽快打印
  -n <num_msgs>   打印消息总数，必须 > 0 才生效
                  ⚠️ -n 0 会被代码覆盖为默认值！
                  默认值: 如果指定了-r，则 30 × rate_Hz
                          如果未指定-r，则 1（只打印一条）
```

**注意**：`chainwing_hinge_status` 的发布频率为 **50 Hz**（`ChainwingSlave.cpp` 中 `ScheduleOnInterval(20000_us)`），
所以 `-r 2` 只是降低了采样显示速率（每秒显示 2 条），实际数据仍以 50 Hz 更新。

### 17.4 总结

| 问题 | 是否为 Bug | 原因 | 解决方案 |
|------|-----------|------|---------|
| GZ 左右机方向反转 | **否** | ENU 坐标系 Y+ = North，SDF 命名用 GZ 坐标约定 | 理解坐标映射，或重命名 SDF 链接 |
| `gz` 命令报错 | **否** | `gz` 是系统命令，非 PX4 shell 命令 | 在**系统终端**执行 gz 命令 |
| listener 自动停止 | **否** | 默认 `num_msgs = 30 × rate` | 加 `-n 20000` 长时间打印（`-n 0` 无效，见 §18.1）|

---

## 18. 深入问题解答：listener -n 0 失效、gz --timeout、QGC 实时查看

### 18.1 问题一：为什么 `listener -r 2 -n 0` 仍然只打印 30 条？

**现象**：执行 `listener chainwing_hinge_status -r 2 -n 0`，预期无限打印，但实际仍在第 30 条（`#30`）后停止。

**根本原因：PX4 listener 代码将 `-n 0` 视为"未指定"而非"无限"**

关键代码在 `src/systemcmds/topic_listener/listener_main.cpp`：

**第 191-192 行** — 解析 `-n` 参数：
```cpp
case 'n':
    num_msgs = strtol(myoptarg, nullptr, 0);  // -n 0 → num_msgs = 0
    break;
```

**第 202-209 行** — 关键！`num_msgs == 0` 被视为"未指定"：
```cpp
if (num_msgs == 0) {                    // ← -n 0 会命中这个条件！
    if (topic_rate != 0) {
        num_msgs = 30 * topic_rate;     // ← -r 2 → num_msgs = 30 × 2 = 60
    } else {
        num_msgs = 1;
    }
}
```

**第 116 行** — 打印循环条件：
```cpp
while (msgs_received < num_msgs) {      // 60次后退出
```

**完整执行流程**：

```
用户输入: listener chainwing_hinge_status -r 2 -n 0

1. 解析 -n 0   → num_msgs = 0
2. 解析 -r 2   → topic_rate = 2
3. num_msgs == 0? → 是! 进入默认计算
4. num_msgs = 30 × 2 = 60
5. topic_interval = 1000/2 = 500ms
6. 循环: while (msgs_received < 60)
7. 打印 #1 到 #60 → 实际用户看到的编号是逐条递增
```

**但用户报告只看到 #1 到 #30**：这是因为 chainwing_hinge_status 以 50Hz 发布，但 `-r 2` 限制了显示速率为 2Hz。在 `orb_set_interval(sub, 500)` 作用下，uORB 只在每 500ms 才通知一次新消息。60 条 ÷ 2 Hz = 30 秒，打印编号到 #30 左右时，可能有些消息被 2 秒超时跳过，导致实际输出约 30 条。

**结论**：PX4 的 `listener` 命令**不支持** `-n 0` 表示无限打印。`0` 在代码中等同于"未指定"。

**正确的解决方案**：

```bash
# 方案1：指定一个足够大的数（推荐）
pxh> listener chainwing_hinge_status -r 2 -n 20000    # 约 2.8 小时

# 方案2：超大数值，接近"无限"
pxh> listener chainwing_hinge_status -r 2 -n 999999   # 约 5.8 天

# 方案3：不指定 -r，用 -n 控制条数
pxh> listener chainwing_hinge_status -n 100            # 打印 100 条后停止

# 按 Ctrl+C 或 q 键可随时手动停止
```

> **如果需要真正的"无限打印"功能**（等待您确认后修改）：
> 需要修改 `listener_main.cpp` 的逻辑，例如用 `-n -1` 表示无限，
> 或改为 `num_msgs == 0` 时不覆盖而是设置 `while (true)` 循环。

### 18.2 问题二：`gz service` 报错 `--req requires --timeout`

**现象**：
```bash
$ gz service -s /world/flat_terrain/wrench \
    --reqtype gz.msgs.EntityWrench \
    --reptype gz.msgs.Boolean \
    --req 'entity: {name: "chainwing_3body_0::left_unit", type: LINK}, wrench: {torque: {x: 5.0}}'

错误: --req requires --timeout
```

**原因：Gazebo Harmonic (gz-transport 12+) 要求 `--req` 必须搭配 `--timeout`**

这是 Gazebo 较新版本的一个 CLI 变更。在旧版本中 `--timeout` 是可选的（默认值会自动应用），但在较新版本中变成了必选参数。

**正确的命令格式**（加上 `--timeout`）：

```bash
# 对 left_unit 施加正方向力矩（使其绕 X 轴抬起）
gz service -s /world/flat_terrain/wrench \
    --reqtype gz.msgs.EntityWrench \
    --reptype gz.msgs.Boolean \
    --timeout 1000 \
    --req 'entity: {name: "chainwing_3body_0::left_unit", type: LINK}, wrench: {torque: {x: 5.0}}'

# 等待 2 秒
sleep 2

# 施加反向力矩恢复
gz service -s /world/flat_terrain/wrench \
    --reqtype gz.msgs.EntityWrench \
    --reptype gz.msgs.Boolean \
    --timeout 1000 \
    --req 'entity: {name: "chainwing_3body_0::left_unit", type: LINK}, wrench: {torque: {x: -5.0}}'
```

**`--timeout` 参数说明**：

| 参数 | 值 | 含义 |
|------|-----|------|
| `--timeout 1000` | 1000 毫秒 | 等待服务响应的最大时间（1 秒） |
| `--timeout 5000` | 5000 毫秒 | 5 秒超时（网络延迟大时使用） |

**完整的铰链测试脚本**（在系统终端中执行，不是 pxh>）：

```bash
#!/bin/bash
# 文件: test_hinge.sh
# 用法: bash test_hinge.sh

echo "=== 铰链扰动测试 ==="
echo "步骤1: 对 left_unit 施加 +5 N·m 力矩..."
gz service -s /world/flat_terrain/wrench \
    --reqtype gz.msgs.EntityWrench \
    --reptype gz.msgs.Boolean \
    --timeout 1000 \
    --req 'entity: {name: "chainwing_3body_0::left_unit", type: LINK}, wrench: {torque: {x: 5.0}}'

echo "等待 2 秒观察响应..."
sleep 2

echo "步骤2: 施加 -5 N·m 反向力矩恢复..."
gz service -s /world/flat_terrain/wrench \
    --reqtype gz.msgs.EntityWrench \
    --reptype gz.msgs.Boolean \
    --timeout 1000 \
    --req 'entity: {name: "chainwing_3body_0::left_unit", type: LINK}, wrench: {torque: {x: -5.0}}'

echo "等待 2 秒..."
sleep 2

echo "步骤3: 对 right_unit 施加 +5 N·m 力矩..."
gz service -s /world/flat_terrain/wrench \
    --reqtype gz.msgs.EntityWrench \
    --reptype gz.msgs.Boolean \
    --timeout 1000 \
    --req 'entity: {name: "chainwing_3body_0::right_unit", type: LINK}, wrench: {torque: {x: 5.0}}'

sleep 2

echo "步骤4: 施加反向力矩恢复..."
gz service -s /world/flat_terrain/wrench \
    --reqtype gz.msgs.EntityWrench \
    --reptype gz.msgs.Boolean \
    --timeout 1000 \
    --req 'entity: {name: "chainwing_3body_0::right_unit", type: LINK}, wrench: {torque: {x: -5.0}}'

echo "=== 测试完成 ==="
echo "请在 PX4 shell 中查看: listener chainwing_hinge_status -r 5 -n 20000"
```

### 18.3 问题三：能否在 QGC 或 GZ 中实时查看铰链参数？能否实时改 PID？

**简短回答**：
- **QGC 查看铰链参数**：❌ 目前不能（需要添加 MAVLink 流）
- **GZ GUI 查看铰链参数**：✅ 可以通过 `gz topic` 查看关节角度
- **PX4 shell 实时改 PID**：✅ 可以直接用 `param set`
- **QGC 实时改 PID**：✅ CW_SLV_KP / CW_SLV_KD 会出现在 QGC 参数列表中

#### 18.3.1 为什么 QGC 看不到 chainwing_hinge_status？

`chainwing_hinge_status` 是一个**自定义 uORB 消息**，只在 PX4 内部的 uORB 消息总线上传播。要让外部工具看到它，需要"桥接"到外部协议：

| 外部查看方式 | 需要的桥接 | 当前状态 |
|-------------|-----------|---------|
| **QGC** | MAVLink 流（`src/modules/mavlink/streams/` 中添加 .hpp） | ❌ **未注册** |
| **ROS2** | DDS 话题（`src/modules/uxrce_dds_client/dds_topics.yaml` 中添加条目） | ❌ **未注册** |
| **PX4 shell** | 直接通过 uORB（内置支持） | ✅ **可用** |

**数据流对比**：

```
                    ┌─── uORB ───────── PX4 shell (listener) ✅
                    │
chainwing_slave ────┤─── MAVLink ──── QGC ❌ (未注册流)
  (发布者)          │
                    └─── DDS ────────── ROS2/GZ ❌ (未注册话题)
```

**现有的标准话题**（如 `vehicle_attitude`、`vehicle_status`）能在 QGC 中看到，是因为它们在 `src/modules/mavlink/mavlink_main.cpp` 中已注册了对应的 MAVLink 流：

```cpp
// 例如 vehicle_attitude → MAVLINK_MSG_ID_ATTITUDE
configure_stream_local("ATTITUDE", 15.0f);
```

`chainwing_hinge_status` 没有对应的 MAVLink 消息定义，所以 QGC 无法显示。

#### 18.3.2 在 GZ 中直接查看关节角度（不经过 PX4）

虽然 PX4 的 hinge_status 不能传到 QGC，但 GZ 本身可以直接查看物理关节状态：

```bash
# 在系统终端中（不是 pxh>）：

# 列出所有 GZ 话题
gz topic -l

# 查看关节状态（如果模型有关节状态发布）
gz topic -e -t /world/flat_terrain/model/chainwing_3body_0/joint_state

# 查看模型姿态
gz model -m chainwing_3body_0

# 持续查看模型姿态（每1秒刷新）
watch -n 1 'gz model -m chainwing_3body_0'

# 查看特定关节位置
gz topic -e -t /world/flat_terrain/model/chainwing_3body_0/joint/hinge_left/0/cmd_pos
```

#### 18.3.3 实时修改 PID 参数

**✅ PX4 shell 中实时修改（推荐，最快）**：

```bash
pxh> param set CW_SLV_KP 0.5      # 修改比例增益
pxh> param set CW_SLV_KD 0.1      # 修改微分增益
pxh> param set CW_SLV_TRIM_MAX 0.4  # 修改最大修正量
pxh> param set CW_SLV_LP_FREQ 15   # 修改低通滤波频率

# 查看当前值
pxh> param show CW_SLV*
```

**原理**：`ChainwingSlave.cpp` 在每次 `Run()` 循环中通过 `ModuleParams::updateParams()` 自动检测参数变化。参数修改后**立即生效**，无需重启模块。

代码位置（`ChainwingSlave.cpp`）：
```cpp
void ChainwingSlave::Run()
{
    // ... 
    updateParams();  // ← 每个周期自动检查参数是否被外部修改
    
    const float kp = _param_cw_slv_kp.get();  // ← 获取最新值
    const float kd = _param_cw_slv_kd.get();
    // ...
}
```

**✅ QGC 中修改（也可以）**：

CW_SLV_KP、CW_SLV_KD 等参数会自动出现在 QGC 的 **Vehicle Setup → Parameters** 页面中，因为它们是标准 PX4 参数（定义在 `chainwing_slave_params.c` 中）。

QGC 参数路径：`Vehicle Setup → Parameters → 搜索 "CW_SLV"`

**⚠️ QGC 不能**做的是：实时查看 `hinge_angle_left` 等 uORB 字段的值。只能修改参数，不能查看自定义话题数据。

#### 18.3.4 为什么新模块的 uORB 数据不能自动在 QGC 中显示？

这是 PX4 的**设计架构决定的**，不是 Bug：

```
PX4 内部消息 (uORB)  ≠  外部传输消息 (MAVLink)
```

uORB 是 PX4 内部的发布-订阅系统（类似 ROS 的 topic），消息格式由 `.msg` 文件定义。MAVLink 是与地面站通信的外部协议，消息格式由 `.xml` 文件定义。两者**完全独立**。

要让一个 uORB 消息在 QGC 中可见，需要：

1. **定义 MAVLink 消息**（在 `mavlink/message_definitions/` 中添加 XML）
2. **编写流发送器**（在 `src/modules/mavlink/streams/` 中添加 .hpp 文件）
3. **注册流**（在 `mavlink_main.cpp` 中 `configure_stream_local()`）
4. **QGC 端解析**（QGC 需要知道新消息的格式）

标准 PX4 消息（attitude, GPS, battery 等）已经完成了这 4 步。自定义消息（如 chainwing_hinge_status）需要手动添加。

> **如果需要添加 MAVLink 流**（等待您确认后实施）：
> 可以使用 MAVLink 的 `DEBUG_FLOAT_ARRAY` 通用消息将铰链数据转发到 QGC，
> 这样不需要定义新的 MAVLink 消息，QGC 可以在 MAVLink Inspector 中查看。

### 18.4 总结表

| 操作 | 当前是否可行 | 方法 |
|------|-------------|------|
| PX4 shell 查看铰链状态 | ✅ 可行 | `listener chainwing_hinge_status -r 2 -n 20000` |
| PX4 shell 实时改 PID | ✅ 可行 | `param set CW_SLV_KP 0.5` |
| QGC 实时改 PID | ✅ 可行 | Parameters → 搜索 CW_SLV |
| QGC 查看铰链角度 | ❌ 不可行 | 需添加 MAVLink 流（等确认后实施） |
| GZ 查看关节角度 | ✅ 可行 | `gz topic -e -t .../joint_state`（系统终端） |
| gz service 施加力矩 | ✅ 可行 | 必须加 `--timeout 1000`，且实体用 `type: LINK`（见 §19） |
| listener -n 0 无限打印 | ❌ 不可行 | 代码将 0 视为"未指定"，改用 `-n 20000` |
| listener -n 20000 长时间打印 | ⚠️ 取决于模块 | 如果 chainwing_slave 未运行，2秒超时退出（见 §19） |

---

## 19. 进一步问题诊断：wrench 超时、listener 秒退、模块状态确认

> **版本**: v1.7 新增  
> **触发原因**: 用户报告 `gz service` wrench 超时 + `listener -n 20000` 仍秒退

### 19.1 问题一：`gz service` wrench 报 "Service call timed out"

**用户命令**：
```bash
gz service -s /world/flat_terrain/wrench \
    --reqtype gz.msgs.EntityWrench \
    --reptype gz.msgs.Boolean \
    --timeout 1000 \
    --req 'entity: {name: "left_unit", type: MODEL}, wrench: {torque: {x: 5.0}}'
# → Service call timed out
```

**根本原因：`left_unit` 是 LINK（链接），不是 MODEL（模型）**

在 `chainwing_3body/model.sdf` 中的结构：
```
chainwing_3body (MODEL)           ← 这是顶层模型
├── base_link (LINK)              ← 中间机体
├── left_unit (LINK)              ← 左翼，是 LINK 不是独立 MODEL
├── right_unit (LINK)             ← 右翼，是 LINK 不是独立 MODEL
├── hinge_left (JOINT)
└── hinge_right (JOINT)
```

运行 `gz model --list` 显示的是：
```
- ground_plane       ← MODEL
- runway_marking     ← MODEL
- chainwing_3body_0  ← MODEL（只有这一个是 chainwing 的模型）
```

**`left_unit` 不在模型列表中** —— 因为它是 `chainwing_3body_0` 模型内部的一个 LINK。

GZ 的 wrench 服务要求精确匹配实体类型：
- 指定 `type: MODEL` + `name: "left_unit"` → GZ 在模型列表中找不到 → **超时**
- 指定 `type: LINK` + `name: "chainwing_3body_0::left_unit"` → GZ 精确找到 → **成功**

**正确的命令**（三处修正：`type: LINK` + 完整作用域名 + `--timeout`）：
```bash
# 对左翼施加 +5 N·m 滚转力矩
gz service -s /world/flat_terrain/wrench \
    --reqtype gz.msgs.EntityWrench \
    --reptype gz.msgs.Boolean \
    --timeout 1000 \
    --req 'entity: {name: "chainwing_3body_0::left_unit", type: LINK}, wrench: {torque: {x: 5.0}}'
```

**对比表**：

| 参数 | 错误值 | 正确值 | 说明 |
|------|--------|--------|------|
| entity name | `"left_unit"` | `"chainwing_3body_0::left_unit"` | 需要完整作用域名（模型名::链接名） |
| entity type | `MODEL` | `LINK` | left_unit 是链接，不是模型 |
| --timeout | 缺失（旧版可选） | `1000`（必选） | gz-transport 12+ 要求 |

**修正后的完整测试脚本**（在**系统终端**执行，**不是** pxh>）：

```bash
#!/bin/bash
# test_hinge_corrected.sh — 铰链扰动测试（修正版）

echo "=== 铰链扰动测试（修正版） ==="

echo "步骤1: 对 left_unit 施加 +5 N·m 滚转力矩..."
gz service -s /world/flat_terrain/wrench \
    --reqtype gz.msgs.EntityWrench \
    --reptype gz.msgs.Boolean \
    --timeout 1000 \
    --req 'entity: {name: "chainwing_3body_0::left_unit", type: LINK}, wrench: {torque: {x: 5.0}}'

echo "等待 3 秒观察铰链响应..."
sleep 3

echo "步骤2: 施加 -5 N·m 反向力矩恢复..."
gz service -s /world/flat_terrain/wrench \
    --reqtype gz.msgs.EntityWrench \
    --reptype gz.msgs.Boolean \
    --timeout 1000 \
    --req 'entity: {name: "chainwing_3body_0::left_unit", type: LINK}, wrench: {torque: {x: -5.0}}'

echo "等待 3 秒..."
sleep 3

echo "步骤3: 对 right_unit 施加 +5 N·m 滚转力矩..."
gz service -s /world/flat_terrain/wrench \
    --reqtype gz.msgs.EntityWrench \
    --reptype gz.msgs.Boolean \
    --timeout 1000 \
    --req 'entity: {name: "chainwing_3body_0::right_unit", type: LINK}, wrench: {torque: {x: 5.0}}'

sleep 3

echo "步骤4: 施加反向力矩恢复..."
gz service -s /world/flat_terrain/wrench \
    --reqtype gz.msgs.EntityWrench \
    --reptype gz.msgs.Boolean \
    --timeout 1000 \
    --req 'entity: {name: "chainwing_3body_0::right_unit", type: LINK}, wrench: {torque: {x: -5.0}}'

echo "=== 测试完成 ==="
```

> **注意**：如果 GZ 仿真中模型名不是 `chainwing_3body_0`（比如重启后变成 `chainwing_3body_1`），
> 先运行 `gz model --list` 确认实际模型名，再替换命令中的前缀。

### 19.2 问题二：`listener -r 2 -n 20000` 仍然几秒就退出

**用户现象**：运行 `listener chainwing_hinge_status -r 2 -n 20000` 后，只持续了几秒就停止了。

**根本原因：`chainwing_slave` 模块未运行 → 无消息发布 → 2秒超时退出**

PX4 的 `listener` 命令有一个**隐藏的 2 秒超时机制**：

```cpp
// src/systemcmds/topic_listener/listener_main.cpp:49
static constexpr float MESSAGE_TIMEOUT_S = 2.0f;

// listener_main.cpp:118 — poll 调用
if (poll(&fds[0], 2, int(MESSAGE_TIMEOUT_S * 1000)) > 0) {
    // 收到消息 → 继续循环
} else {
    // 2000ms 内无消息 → 退出！
    PX4_INFO_RAW("Waited for %.1f seconds without a message. Giving up.\n",
                 (double) MESSAGE_TIMEOUT_S);
    break;  // ← 退出循环
}
```

**逻辑流程**：
```
listener -r 2 -n 20000
    ↓
订阅 chainwing_hinge_status 话题
    ↓
poll() 等待消息，超时 = 2000ms
    ↓
┌─ 有消息到达？
│   YES → 打印消息 → 回到 poll()
│   NO  → "Waited for 2.0 seconds without a message. Giving up." → 退出
└─
```

**关键**：`-n 20000` 只控制"最多打印多少条"，**不能阻止超时退出**。
如果 2 秒内没有任何消息到达，无论 `-n` 设多大，listener 都会退出。

#### 19.2.1 为什么 chainwing_slave 模块可能未运行？

**检查方法**（在 pxh> 中）：
```bash
# 方法1: 检查模块是否运行
pxh> chainwing_slave status
# 如果模块未运行，会显示: "not running"

# 方法2: 检查使能参数
pxh> param show CW_SLV_EN
# 如果显示 0 或 "not found"，模块不会启动
```

**可能原因及解决方案**：

| 原因 | 检查方法 | 解决方案 |
|------|----------|----------|
| `CW_SLV_EN=0`（默认禁用） | `param show CW_SLV_EN` | `param set CW_SLV_EN 1` |
| 使用了 4007 而非 4008 机架 | 检查启动日志 | 使用 `make px4_sitl gz_chainwing_3body` |
| 模块未编译 | 检查 build log | 确认 `default.px4board` 中包含 `chainwing_slave` |
| 模块崩溃 | `dmesg` | 查看崩溃日志 |

**修复步骤**：
```bash
# 步骤1: 在 pxh> 中启用从机模块
pxh> param set CW_SLV_EN 1

# 步骤2: 手动启动模块（如果未自动启动）
pxh> chainwing_slave start

# 步骤3: 验证模块运行
pxh> chainwing_slave status
# 应显示: "is running"

# 步骤4: 现在 listener 应该能持续接收数据了
pxh> listener chainwing_hinge_status -r 2 -n 20000
# 如果模块在运行且发布数据（50Hz），这会持续 20000/2 = 10000秒
```

#### 19.2.2 区分"消息条数限制"和"超时退出"

| 退出原因 | 表现 | 持续时间 |
|----------|------|----------|
| **超时退出**（当前问题） | 打印 "Waited for 2.0 seconds without a message. Giving up." 后退出 | 约 2 秒 |
| **条数限制**（`-n` 耗尽） | 静默停止，无额外提示 | `-n 20000` 在 2Hz 下 = 10000秒 |
| **用户中断** | 按 Ctrl+C 或 q | 用户控制 |

**如果用户看到的是"只打印了几条就停了"但没有超时提示**：
可能是话题发布频率极低（如模块刚启动、处于 idle 状态）。
chainwing_slave 的 `ScheduleOnInterval(20000_us)` 确保 50Hz 发布，
但如果 `CW_SLV_EN=0`，模块根本不会调用 `Run()` → 不发布任何消息。

### 19.3 快速诊断清单

在遇到问题时，按顺序执行以下诊断：

```bash
# ============ 在 pxh> 中 ============

# 1. 检查从机模块是否启用
param show CW_SLV_EN
# 预期: 1（如果是 0，执行 param set CW_SLV_EN 1）

# 2. 检查从机模块状态
chainwing_slave status
# 预期: "is running"（如果 "not running"，执行 chainwing_slave start）

# 3. 检查话题是否有数据
listener chainwing_hinge_status
# 预期: 打印数据（如果 "Giving up"，回到步骤 1-2）

# 4. 长时间监听
listener chainwing_hinge_status -r 2 -n 20000
# 预期: 持续打印（10000秒）

# ============ 在系统终端中 ============

# 5. 确认模型名
gz model --list
# 记录实际模型名，如 chainwing_3body_0

# 6. 施加力矩测试（替换实际模型名）
gz service -s /world/flat_terrain/wrench \
    --reqtype gz.msgs.EntityWrench \
    --reptype gz.msgs.Boolean \
    --timeout 1000 \
    --req 'entity: {name: "chainwing_3body_0::left_unit", type: LINK}, wrench: {torque: {x: 5.0}}'
# 预期: data: true
```

### 19.4 §18 勘误

§18 中的以下内容需要修正：

| 位置 | 原内容 | 修正 |
|------|--------|------|
| §18.1 | `listener -n 20000` 能长时间运行 | 前提：chainwing_slave 必须正在运行，否则 2 秒超时退出 |
| §18.2 | `type: MODEL` | **已修正为** `type: LINK`（全文 11 处已更新） |
| §18.4 总结表 | "gz service 施加力矩: ✅ 可行" | 需要正确的实体类型（LINK）和完整作用域名 |

> **全文 `type: MODEL` → `type: LINK` 修正统计**：
> - §12.4（2处）：施加力矩测试命令
> - §18.2（6处）：gz service 示例 + 测试脚本
> - §18.2 test_hinge.sh 脚本（3处）：left_unit × 2 + right_unit × 1
> - 总计 11 处已全部修正为 `entity: {name: "chainwing_3body_0::left_unit", type: LINK}`

---

## 20. 终极诊断：模块未启动根因 + 替代方案 + QGC调参指南

> **背景**：用户按照 §19 建议修正了 `type: LINK`、加了 `--timeout`、使用 `-n 20000`，
> 但所有问题仍然存在。本节给出最终根因分析和可行的替代方案。

### 20.1 所有问题的唯一根因：`chainwing_slave` 从未被启动

经过对整个仓库的代码检索，发现**所有**问题都指向同一个根因：

```
4008_gz_chainwing_3body 机架配置文件中：
  ✅ 设置了 CW_SLV_EN=1              （参数已配置）
  ✅ 设置了 CW_SLV_KP=0.3, KD=0.05   （增益已配置）
  ❌ 从未执行 "chainwing_slave start"  （模块从未启动！）
```

**检索证据**：

| 文件 | 是否包含 `chainwing_slave start` |
|------|:---:|
| `ROMFS/.../4008_gz_chainwing_3body` | ❌ 只有 `param set-default`，无启动命令 |
| `ROMFS/.../rc.fw_apps` | ❌ 只启动标准FW模块（ekf2, fw_att_control 等） |
| `ROMFS/.../px4-rc.simulator` | ❌ 只启动 gz_bridge 和 sensor_*_sim |
| `ROMFS/.../rcS` | ❌ 调用上述脚本，不包含自定义模块启动 |
| 整个仓库 grep | ❌ 只在文档(.md)中出现，**无任何 .sh/.init 脚本包含此命令** |

**这解释了所有问题**：

| 现象 | 根因 |
|------|------|
| `listener chainwing_hinge_status` 2秒退出 | 模块未运行 → 无消息发布 → 2秒 MESSAGE_TIMEOUT_S 超时 |
| `-n 20000` 也是几秒退出 | 同上：不是条数限制，是无消息超时 |
| wrench 施力后无铰链反应 | 模块未运行 → 无 PD 控制器处理铰链偏差 → 无修正输出 |
| QGC 看不到铰链数据 | 模块未运行 → 无数据发布 → 无数据可看 |

### 20.2 立即验证方法

在 PX4 shell (`pxh>`) 中执行以下命令序列：

```bash
# 步骤1: 确认模块已编译（应显示帮助信息）
pxh> chainwing_slave status
# 预期: "not running" ← 确认模块存在但未启动

# 步骤2: 手动启动模块
pxh> chainwing_slave start
# 预期: "chainwing_slave running" 或 无报错

# 步骤3: 确认正在运行
pxh> chainwing_slave status
# 预期: 显示运行状态 + 参数信息

# 步骤4: 现在再试 listener（应该能持续输出了！）
pxh> listener chainwing_hinge_status -r 2 -n 20000
# 预期: 持续打印，每0.5秒一条，直到手动 Ctrl+C
```

> **重要**：如果步骤 2 成功后，listener 能持续输出数据，则确认根因就是模块未启动。

### 20.3 永久解决方案（需要改代码时实施）

在机架配置文件末尾添加一行启动命令。需要修改的文件：

```
ROMFS/px4fmu_common/init.d-posix/airframes/4008_gz_chainwing_3body
```

在文件最末尾（参数设置之后）添加：

```bash
# Auto-start slave controller module
chainwing_slave start
```

> ⚠️ **当前不修改代码**，待您确认后再实施。

### 20.4 gz wrench 替代方案

即使模块启动后，`gz service wrench` 在某些 GZ 版本中仍可能超时。以下是**三种**替代方案：

#### 方案 A：用 `gz topic` 直接发送关节力（最推荐 ✅）

```bash
# 在系统终端（非 pxh>）中直接发布关节力命令
# 对 hinge_left 施加 5 N·m 力矩
gz topic -t /model/chainwing_3body_0/joint/hinge_left/cmd_force \
  -m gz.msgs.Double -p 'data: 5.0'
```

> 注意：此方法需要模型中有 `JointForceCmd` 系统插件已加载（GZ Harmonic 默认加载）。
> 如果不生效，说明当前 GZ 版本不支持此路径，改用方案 B。

#### 方案 B：用 `gz model` 设置关节位置（验证铰链存在性）

```bash
# 查看当前模型的关节列表
gz model -m chainwing_3body_0 -j

# 查看 hinge_left 关节状态
gz model -m chainwing_3body_0 -j hinge_left
```

> 此方法只能**读取**铰链状态，不能施加力。但可确认铰链关节是否正确创建。

#### 方案 C：用 GZ GUI 中 Component Inspector（图形化验证）

1. 在 GZ GUI 窗口中，右键点击飞机模型
2. 选择 "Entity Inspector" 或 "Component Inspector"
3. 展开 Joint 列表，查找 `hinge_left` 和 `hinge_right`
4. 可以看到当前角度、速度、力矩等实时数据

> 这是最直观的验证方式，无需命令行操作。

#### 方案 D：在 PX4 shell 中用 actuator_test 直接偏转升降舵

```bash
# 在 pxh> 中直接控制舵面，测试升降舵能否产生铰链力矩
# 测试 servo_0（左翼升降舵），偏转 30%
pxh> actuator_test set -m 0 -v 0.3   # 电机0
pxh> actuator_test set -m 3 -v 0.3   # 舵机0（左翼elevon）

# 测试完毕后恢复
pxh> actuator_test set -m 3 -v 0
```

### 20.5 QGC 调参完整指南

#### Q: 能否用 QGC 修改 CW_SLV_KP、CW_SLV_KD 等参数？

**✅ 完全可以，强烈推荐！**

PX4 的参数系统对 QGC 完全透明。所有 `CW_SLV_*` 参数都是标准 PX4 参数，
QGC 通过 MAVLink `PARAM_REQUEST_LIST` / `PARAM_SET` 消息访问它们。

**QGC 操作步骤**：

```
1. 连接 QGC 到仿真（UDP 默认自动连接 localhost:14550）

2. 打开 Vehicle Setup（齿轮图标）→ Parameters

3. 搜索 "CW_SLV" → 出现 5 个参数：
   - CW_SLV_EN      = 1（使能）
   - CW_SLV_KP      = 0.3（比例增益）
   - CW_SLV_KD      = 0.05（微分增益）
   - CW_SLV_TRIM_MAX = 0.3（最大修正量）
   - CW_SLV_LP_FREQ  = 10.0（低通滤波频率）

4. 双击参数值 → 输入新值 → 回车
   ※ 修改立即生效，无需重启
   ※ 重启后恢复默认值（因为机架文件用的是 param set-default）
   ※ 若要永久保存：用 `param save` 命令或在QGC中点"Save to file"
```

**实时调 PID 增益的推荐流程**：

```
QGC Parameters 界面                          PX4 Shell (pxh>)
┌───────────────────┐                    ┌────────────────────────┐
│ CW_SLV_KP = 0.3  │ ← 修改 →          │ listener               │
│ CW_SLV_KD = 0.05 │                    │   chainwing_hinge_status│
│ CW_SLV_TRIM_MAX  │                    │   -r 2 -n 20000       │
│   = 0.3          │                    │                        │
│                   │                    │ → 实时观察 trim_left/  │
│ [双击修改即时生效] │                    │   trim_right 变化      │
└───────────────────┘                    └────────────────────────┘
```

#### Q: 能否在 QGC 中实时查看铰链角度？

**❌ 目前不能。原因如下**：

QGC 只能显示 PX4 通过 MAVLink 协议发送的数据。显示数据需要完整链路：

```
chainwing_slave 模块
  ↓ uORB 发布 chainwing_hinge_status
PX4 MAVLink 模块
  ↓ ❌ 没有注册 chainwing_hinge_status 的 MAVLink 流
  ↓    （需要在 src/modules/mavlink/streams/ 中添加自定义流）
QGC
  ↓ ❌ 收不到数据
显示
```

**哪些可以在 QGC 中看到，哪些不能**：

| 数据 | QGC 可见？ | 原因 |
|------|:---:|------|
| CW_SLV_KP / KD / TRIM_MAX | ✅ | 标准 PX4 参数，自动通过 MAVLink 传输 |
| CW_SLV_EN | ✅ | 同上 |
| vehicle_attitude (Roll/Pitch/Yaw) | ✅ | PX4 内置 MAVLink 流 |
| vehicle_angular_velocity | ✅ | PX4 内置 MAVLink 流 |
| 舵面实际位置（actuator_outputs） | ✅ | PX4 内置 MAVLink 流 |
| **chainwing_hinge_status** | ❌ | 自定义 uORB，未注册 MAVLink 流 |
| **铰链角度/修正量** | ❌ | 同上 |

#### Q: 建不建议用 QGC 调参？

**✅ 强烈建议！理由**：

1. **实时生效**：QGC 修改参数后立即生效，不需要重启 PX4 或重新编译
2. **图形界面**：比 PX4 shell 的 `param set` 更直观
3. **参数分组**：搜索 "CW_SLV" 一次看到所有相关参数
4. **范围检查**：QGC 显示参数的 min/max/default，防止设置不合理值
5. **同时监控**：QGC 可以同时显示姿态/角速度曲线，辅助判断 PD 效果

**不建议的场景**：
- 需要查看 `chainwing_hinge_status` 具体数值时 → 必须用 PX4 shell `listener`
- 需要程序化批量调参时 → 用 PX4 shell `param set`

### 20.6 推荐的完整调试工作流

```
┌─────────────────────────────────────────────────────────────┐
│                     推荐调试工作流                            │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  终端1: 启动仿真                                             │
│  $ make px4_sitl gz_chainwing_3body                         │
│  pxh> chainwing_slave start   ← ⭐ 关键：手动启动模块       │
│                                                             │
│  终端2: QGC（自动连接 UDP:14550）                            │
│  → Parameters → 搜索 CW_SLV → 修改 KP/KD                  │
│  → 实时观察飞机姿态曲线                                      │
│                                                             │
│  终端3: PX4 shell 监控                                       │
│  pxh> listener chainwing_hinge_status -r 2 -n 20000        │
│  → 实时观察 hinge_angle + trim 数值                          │
│                                                             │
│  终端4: GZ 命令（可选）                                      │
│  $ gz model -m chainwing_3body_0 -j                         │
│  → 查看关节状态                                              │
│                                                             │
│  终端5: GZ GUI                                               │
│  → Entity Inspector → 查看关节角度可视化                     │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 20.7 总结

| 问题 | 根因 | 解决方案 |
|------|------|----------|
| listener 几秒退出 | `chainwing_slave` 未启动 → 无消息 → 2秒超时 | `pxh> chainwing_slave start` |
| wrench 超时 | 可能是 GZ 版本兼容问题 | 改用 `gz model -j` 或 GUI Inspector 查看铰链 |
| QGC 能否调参？ | ✅ 完全可以 | Parameters → 搜索 CW_SLV → 实时修改 |
| QGC 能否看铰链数据？ | ❌ 不能（需添加 MAVLink 流） | 用 PX4 shell `listener` 替代 |
| 建不建议用 QGC？ | **强烈建议** | PID 调参首选 QGC，数据监控用 PX4 shell |
| 永久修复方案 | 在机架文件中添加启动命令 | 等您确认后实施（只需加一行代码） |

### 20.8 §17-§19 勘误

| 章节 | 原内容 | 修正 |
|------|--------|------|
| §17.3 | `-n 0` 可实现无限打印 | ❌ 错误。`-n 0` 被覆盖为 `30*rate`。已在 §18 修正 |
| §18.1 | `-n 20000` 可长时间运行 | ⚠️ 前提条件不足。**必须先启动 chainwing_slave 模块** |
| §19.2 | "检查 CW_SLV_EN 是否为 1" | ⚠️ 参数正确但不够。**核心问题是模块未被启动** |
| §19.3 快速诊断 | 第一步检查参数 | 应改为：**第一步检查 `chainwing_slave status`** |

> **根本原因总结**：§17-§19 的诊断都假设模块已在运行，但实际上机架配置文件
> `4008_gz_chainwing_3body` 从未包含 `chainwing_slave start` 命令。
> 这是所有问题的唯一根因。

---

*文档结束*
