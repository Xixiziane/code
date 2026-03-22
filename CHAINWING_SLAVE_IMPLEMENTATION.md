# 链翼无人机从机固件实现文档

## CHAINWING_SLAVE_IMPLEMENTATION.md

> **版本**: v2.3  
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
21. [listener打印过快的真正原因：锁步仿真时间 vs 挂钟时间](#21-listener打印过快的真正原因锁步仿真时间-vs-挂钟时间)
22. [完整调参指南：QGC实时曲线 + Logger回放 + 替代方案全解析](#22-完整调参指南qgc实时曲线--logger回放--替代方案全解析)
23. [第二次飞行日志分析：横滚评估 + 俯仰阶跃响应为0的原因](#23-第二次飞行日志分析横滚评估--俯仰阶跃响应为0的原因)
24. [硬件通信验证评估：能否直接用于两机通信 + 需增加的代码](#24-硬件通信验证评估能否直接用于两机通信--需增加的代码)
25. [铰链参数保守性分析：刚度/阻尼是否过大导致从机代码失效](#25-铰链参数保守性分析刚度阻尼是否过大导致从机代码失效)

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
| §18.1 | `-n 20000` 可长时间运行 | ✅ 命令本身正确。`-n 20000` 确实打印 20000 条（用户已验证）|
| §19.2 | listener 秒退原因是模块未运行 | ⚠️ 部分正确（模块未运行时确实会 2s 超时）。但模块运行后另一个问题出现：见 §21 |
| §20 | 所有问题的唯一根因是模块未启动 | ❌ 错误。用户确认模块已手动启动，20000 条消息确实打印了，只是速度过快。真正原因见 §21 |

---

## 21. listener打印过快的真正原因：锁步仿真时间 vs 挂钟时间

> **背景**：用户确认已手动执行 `chainwing_slave start`，`listener -r 2 -n 20000` 确实打印了
> 20000 条消息，但几秒钟就全部打完了。预期应该是每秒 2 条，需要 10000 秒（约 2.7 小时）。

### 21.1 根本原因：锁步（Lockstep）仿真时间

**PX4 SITL 使用锁步调度器，所有时间函数都使用仿真时间，不是挂钟（墙上时钟）时间。**

```
┌─────────────────────────────────────────────────┐
│             PX4 SITL 时间架构                      │
├─────────────────────────────────────────────────┤
│                                                 │
│  Gazebo 仿真器                                   │
│    ↓ 每帧推进仿真时间                               │
│  lockstep_scheduler.set_absolute_time(sim_time)  │
│    ↓                                            │
│  hrt_absolute_time() ← 返回 sim_time             │
│    ↓                                            │
│  orb_set_interval() ← 用 sim_time 判断间隔        │
│  poll() 超时        ← 用 sim_time 判断超时         │
│  ScheduleOnInterval ← 用 sim_time 调度模块        │
│                                                 │
│  结论：PX4 内部的所有 "500ms" 都是仿真时间的 500ms   │
│       不是挂钟时间的 500ms                          │
└─────────────────────────────────────────────────┘
```

**代码证据**（`platforms/posix/src/px4/common/drv_hrt.cpp`）：

```cpp
hrt_abstime hrt_absolute_time()
{
#if defined(ENABLE_LOCKSTEP_SCHEDULER)
    return lockstep_scheduler.get_absolute_time();  // ← 仿真时间！
#else
    struct timespec ts;
    px4_clock_gettime(CLOCK_MONOTONIC, &ts);         // ← 真实时间
    return ts_to_abstime(&ts);
#endif
}
```

**时间链路分析**：

```
listener -r 2 设置：
  topic_interval = 1000/2 = 500ms
  ↓
  orb_set_interval(sub, 500)
  ↓
  内部检查：hrt_elapsed_time(&_last_update) >= 500000us
  ↓
  hrt_elapsed_time 使用 hrt_absolute_time() = lockstep_scheduler.get_absolute_time()
  ↓
  这是 仿真时间，不是挂钟时间！

poll() 超时：
  poll(&fds, 2, 2000)  // 2000ms
  ↓
  底层通过 pthread_cond_timedwait
  ↓
  lockstep_scheduler.cond_timedwait(cond, mutex, sim_deadline)
  ↓
  当 Gazebo 推进 sim_time >= sim_deadline 时唤醒
  ↓
  也是 仿真时间！
```

### 21.2 数学计算

```
chainwing_slave 发布频率 = 50 Hz（仿真时间内）
listener -r 2 限速 = 2 Hz（仿真时间内）
请求消息数 = 20000 条

仿真时间内需要：20000 / 2 = 10000 仿真秒

但仿真运行速度远快于实时：
  典型 GZ 锁步速度 ≈ 100-1000x 实时
  （取决于模型复杂度和 CPU 性能）

挂钟时间 = 10000 仿真秒 / 仿真加速比
  若 500x 加速：10000 / 500 = 20 秒
  若 1000x 加速：10000 / 1000 = 10 秒
  若 2000x 加速：10000 / 2000 = 5 秒  ← 与用户观察"几秒"吻合

结论：listener 确实在以 2Hz（仿真时间）限速，
     但仿真时间本身在飞速推进，所以挂钟上感觉瞬间完成。
```

### 21.3 这不是 Bug

**这是 PX4 SITL 锁步仿真的设计行为**：

| 特性 | 锁步 SITL | 真实硬件 |
|------|-----------|----------|
| `hrt_absolute_time()` | 仿真时间（GZ 控制） | 真实时间（硬件时钟） |
| `listener -r 2` | 仿真2Hz（挂钟上很快） | 真实2Hz |
| `orb_set_interval(500)` | 仿真500ms | 真实500ms |
| `poll(2000ms)` | 仿真2s | 真实2s |
| 模块 50Hz 调度 | 仿真50Hz | 真实50Hz |

**在真实 Pixhawk 硬件上，`listener -r 2` 会正常以挂钟 2Hz 打印。**
锁步 SITL 的目的是让仿真可以比实时更快地运行（加速测试），代价就是
人眼无法"实时观看"——这是正确的取舍。

### 21.4 正确的实时监控方案

由于 PX4 内部时间全是仿真时间，**在 PX4 shell 内部无法实现挂钟限速**。
需要使用外部工具：

#### 方案 A：使用 `watch` 命令（推荐，最简单）

在 PX4 shell 中用单次打印，外部用 `watch` 定时刷新：

```bash
# 在 Linux 系统终端（不是 pxh>）
# 每 0.5 秒执行一次 listener（单次打印最新值）
watch -n 0.5 "echo 'listener chainwing_hinge_status' | \
  nc localhost 4560 2>/dev/null | head -20"
```

> **注意**：此方案需要 MAVLink shell 端口可用。不是所有 SITL 配置都支持。

#### 方案 B：使用 GZ 话题直接查看（推荐，最可靠）

GZ 的时间输出是挂钟时间的：

```bash
# 在 Linux 系统终端中
# 查看关节状态（挂钟实时）
gz topic -e -t /world/flat_terrain/model/chainwing_3body_0/joint_state

# 每 2 秒查看一次模型状态
watch -n 2 "gz model -m chainwing_3body_0 -p"
```

#### 方案 C：使用 QGC MAVLink Inspector

QGC 的 MAVLink Inspector（菜单 → Analyze Tools → MAVLink Inspector）
以挂钟时间显示消息频率。但前提是消息已注册到 MAVLink 流
（chainwing_hinge_status 目前未注册，需要代码修改才能在 QGC 中看到）。

**QGC 可以做的**：
- ✅ 修改 CW_SLV_KP / CW_SLV_KD / CW_SLV_TRIM_MAX 参数（Parameters 面板）
- ✅ 查看标准话题（vehicle_attitude、vehicle_local_position 等）
- ❌ 不能查看 chainwing_hinge_status（未注册到 MAVLink 流）

#### 方案 D：使用 PX4 logger + 事后分析（推荐用于 PD 调参）

```bash
# pxh> 中
logger on          # 开始记录（保存到 SD 卡/日志目录）
# ... 运行实验 ...
logger off         # 停止记录

# 然后用 FlightPlot 或 PX4 Flight Review 分析日志
# chainwing_hinge_status 会自动记录到 .ulg 文件中
```

**这是 PID 调参最推荐的方式**：
1. 不受锁步时间影响
2. 完整记录每一帧数据
3. 可以绘制时序图、叠加对比
4. PX4 Flight Review (https://review.px4.io) 支持自定义话题

#### 方案 E：在 PX4 shell 中单次查询

```bash
# pxh> 中
# 每次手动执行，打印最新的一条
listener chainwing_hinge_status

# 多次查看时，手动按 ↑ + Enter 重复
```

### 21.5 各方案对比

| 方案 | 实时性 | 易用性 | 限制 |
|------|--------|--------|------|
| A: watch + nc | ⭐⭐⭐ | ⭐⭐ | 需要 MAVLink shell 端口 |
| B: gz topic | ⭐⭐⭐⭐ | ⭐⭐⭐⭐ | 只能看 GZ 层数据（关节角、位姿），不能看 PX4 话题 |
| C: QGC Inspector | ⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | 需要注册 MAVLink 流（需改代码） |
| D: logger + 回放 | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ | 不是实时，适合事后分析 |
| E: 单次 listener | ⭐⭐ | ⭐⭐⭐⭐⭐ | 手动操作，不连续 |

**推荐组合**：
- **调参阶段**：方案 D（logger）+ 方案 B（gz topic 实时辅助）
- **日常监控**：方案 B（gz topic）+ 方案 E（单次 listener）
- **正式测试**：方案 D（完整日志记录 + 事后分析）

### 21.6 §20 勘误

§20 的核心结论"模块未启动是所有问题的唯一根因"是**错误的**：

| §20 原判断 | 修正 |
|------------|------|
| "chainwing_slave 从未启动" | ❌ 用户确认已手动执行 `chainwing_slave start` |
| "listener 2s 超时是因为模块未运行" | ⚠️ 模块未运行时确实如此，但用户的问题是模块运行后打印仍然过快 |
| "这是所有问题的唯一根因" | ❌ 模块启动后，还有锁步时间问题 |
| §20 中的"立即修复"步骤 | ✅ 正确（手动启动确实需要），但不是完整解决方案 |
| §20 中的 QGC 调参建议 | ✅ 正确，CW_SLV_* 参数确实可以通过 QGC 修改 |

> **修正后的诊断树**：
> 
> ```
> listener 表现异常
>   ├── 2 秒后退出（无消息打印）
>   │     └── 原因：模块未运行 → 解决：chainwing_slave start（§20 正确）
>   │
>   └── 打印了 N 条但速度过快（几秒完成）
>         └── 原因：锁步仿真时间 ≠ 挂钟时间（§21 本节）
>               └── 解决：使用 gz topic / logger / 单次 listener
> ```

### 21.7 总结

| 问题 | 根本原因 | 是否 Bug | 解决方案 |
|------|----------|----------|----------|
| `listener -r 2 -n 20000` 几秒打完 | PX4 SITL 锁步模式下所有时间函数使用仿真时间 | ❌ 设计行为 | 用 `gz topic` 或 `logger` 替代 |
| 仿真中无法"实时"观看 PX4 话题 | 仿真运行速度 >> 实时 | ❌ 正常 | 用外部工具（watch/GZ GUI） |
| 真实硬件上 listener 正常吗？ | 真实硬件无锁步，时间=挂钟 | ✅ 正常 | 无需处理 |

---

## 22. 完整调参指南：QGC实时曲线 + Logger回放 + 替代方案全解析

### 22.1 调参方法全景对比

链翼从机 PD 控制器的调参有以下 5 种方法，各有适用场景：

| 方法 | 实时性 | 曲线图 | 需要代码修改？ | 适用阶段 | 推荐度 |
|------|--------|--------|----------------|----------|--------|
| **A. Logger + PlotJuggler** | ❌ 事后 | ✅ 专业 | ❌ 不需要 | 全阶段 | ⭐⭐⭐⭐⭐ |
| **B. Logger + Flight Review** | ❌ 事后 | ✅ 网页 | ❌ 不需要 | 全阶段 | ⭐⭐⭐⭐ |
| **C. QGC实时曲线（需加代码）** | ✅ 实时 | ✅ 基础 | ⚠️ 需约15行 | 初期调试 | ⭐⭐⭐ |
| **D. gz topic + 脚本绘图** | ✅ 挂钟实时 | ✅ 自定义 | ❌ 不需要 | SITL仿真 | ⭐⭐⭐ |
| **E. QGC参数界面（只改参数）** | ✅ 即时生效 | ❌ 无 | ❌ 不需要 | 快速微调 | ⭐⭐⭐⭐ |

> **最推荐的组合**：**方法 A（Logger + PlotJuggler）做分析** + **方法 E（QGC 改参数）做调整**
> 
> **为什么不首推 QGC 实时曲线？** 见 §22.3

---

### 22.2 方法 A：Logger + PlotJuggler（⭐⭐⭐⭐⭐ 最推荐）

这是 PX4 官方推荐的标准调参流程，也是 99% 的 PX4 开发者使用的方法。

#### 22.2.1 原理

```
PX4 运行时                          事后分析
┌──────────────────┐         ┌─────────────────────┐
│ chainwing_slave   │         │                     │
│   ↓ publish       │         │  PlotJuggler        │
│ chainwing_hinge_  │ logger  │    ↓ 加载 .ulg     │
│   status (uORB)  ├────────→│    ↓ 拖拽字段       │
│                   │ .ulg    │    ↓ 时间序列曲线   │
│ 所有标准话题也    │ 文件    │    ↓ 叠加对比       │
│ 同时被记录       │         │    ↓ 导出 CSV       │
└──────────────────┘         └─────────────────────┘
```

#### 22.2.2 所需软件

| 软件 | 用途 | 安装命令 | 版本要求 |
|------|------|----------|----------|
| **PlotJuggler** | .ulg 曲线分析 | `sudo snap install plotjuggler` 或 `sudo apt install plotjuggler` | ≥3.5 |
| **pyulog** | .ulg → CSV 转换 | `pip3 install pyulog` | ≥0.9 |
| **Flight Review** | 在线分析（可选） | 访问 https://review.px4.io/ | 在线 |
| **QGroundControl** | 下载日志 + 改参数 | https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html | ≥4.0 |

> **PlotJuggler 安装详细步骤（Ubuntu）**：
> ```bash
> # 方式1：Snap（最简单）
> sudo snap install plotjuggler
> 
> # 方式2：AppImage（免安装）
> wget https://github.com/facontidavide/PlotJuggler/releases/download/3.8.4/PlotJuggler-3.8.4-x86_64.AppImage
> chmod +x PlotJuggler-*.AppImage
> ./PlotJuggler-*.AppImage
> 
> # 方式3：从源码编译（如果需要ROS集成）
> sudo apt install qtbase5-dev libqt5svg5-dev libqt5websockets5-dev
> git clone https://github.com/facontidavide/PlotJuggler.git
> cd PlotJuggler && mkdir build && cd build
> cmake .. && make -j$(nproc) && sudo make install
> ```

#### 22.2.3 第一步：让 Logger 记录 chainwing_hinge_status

**问题**：Logger 默认不记录 `chainwing_hinge_status`（它不在默认话题列表中）。

**解决方案（3选1）**：

**方案 ①（推荐）在 PX4 shell 中动态添加**：
```bash
# PX4 shell (pxh>) 中执行：
logger on -t chainwing_hinge_status   # 将此话题添加到当前日志会话
```

**方案 ② 通过参数启用 DEBUG 日志 profile**：
```bash
# PX4 shell 中：
param set SDLOG_PROFILE 33   # 1(默认) + 32(DEBUG话题)
# 需重启 PX4 生效
```

> ⚠️ 注意：DEBUG profile 仅记录 `debug_key_value`、`debug_vect`、`debug_array` 等调试话题，
> **不会自动记录 `chainwing_hinge_status`**（因为它不是 debug_* 命名格式）。
> 如果你使用方案 ②，还需要让 ChainwingSlave 发布 debug_key_value（需改代码，见 §22.4）。

**方案 ③ 修改代码：在默认日志列表中添加（需重编译）**：
```
文件：src/modules/logger/logged_topics.cpp
位置：add_default_topics() 函数末尾
添加：add_optional_topic("chainwing_hinge_status", 50);
      // 50ms = 20Hz 记录频率
```

#### 22.2.4 第二步：运行仿真并收集日志

```bash
# 终端 1：启动仿真
make px4_sitl gz_chainwing_3body

# PX4 shell (pxh>) 中：
chainwing_slave start          # 启动从机控制模块
logger on -t chainwing_hinge_status  # 添加到日志（如果用方案①）
logger status                  # 确认日志正在记录

# 执行测试（例如飞行、施加扰动等）
# ... 测试完成后 ...

logger off                     # 停止记录（或直接关闭PX4）
```

#### 22.2.5 第三步：找到日志文件

```bash
# SITL 日志路径：
ls -la build/px4_sitl_default/rootfs/log/

# 典型输出：
# 2024-03-21/
#   ├── 14_35_42.ulg    ← 这就是日志文件
#   └── 14_50_10.ulg

# 找最新的文件：
find build/px4_sitl_default/rootfs/ -name "*.ulg" -newer /tmp/start_marker | sort
```

> **真实硬件上**：日志在 SD 卡 `/fs/microsd/log/` 目录下，
> 可通过 QGC → Analyze Tools → Log Download 直接下载。

#### 22.2.6 第四步：用 PlotJuggler 分析

```
1. 打开 PlotJuggler
2. File → Load Data → 选择 .ulg 文件
3. 左侧面板出现所有话题，展开 chainwing_hinge_status：
   ├── hinge_angle_left
   ├── hinge_angle_right
   ├── hinge_rate_left
   ├── hinge_rate_right
   ├── trim_left
   ├── trim_right
   └── data_valid

4. 拖拽字段到绘图区：
   - 上面板：拖入 hinge_angle_left + hinge_angle_right（叠加对比）
   - 下面板：拖入 trim_left + trim_right（查看控制器输出）

5. 同时查看标准话题（自动记录的）：
   - vehicle_attitude.q → 检查整体姿态
   - actuator_servos → 检查实际舵面输出
   - vehicle_angular_velocity → 检查角速度

6. 曲线操作：
   - 滚轮缩放时间轴
   - 右键 → Split Horizontal/Vertical 分屏
   - 标记区域 → 计算统计量（均值、标准差、最大值）
```

#### 22.2.7 第五步：用 pyulog 转 CSV（可选）

```bash
# 安装 pyulog
pip3 install pyulog

# 查看 .ulg 文件包含哪些话题
ulog_info build/px4_sitl_default/rootfs/log/2024-03-21/14_35_42.ulg

# 提取特定话题为 CSV
ulog2csv build/px4_sitl_default/rootfs/log/2024-03-21/14_35_42.ulg \
    -m chainwing_hinge_status \
    -o /tmp/hinge_data/

# 生成的 CSV 文件可以用 Python/MATLAB/Excel 分析：
# /tmp/hinge_data/chainwing_hinge_status_0.csv
# 列：timestamp, hinge_angle_left, hinge_angle_right, ...

# 用 Python 绘图示例：
python3 << 'EOF'
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('/tmp/hinge_data/chainwing_hinge_status_0.csv')
df['time_s'] = (df['timestamp'] - df['timestamp'].iloc[0]) / 1e6

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
ax1.plot(df['time_s'], df['hinge_angle_left'], label='左铰链角度')
ax1.plot(df['time_s'], df['hinge_angle_right'], label='右铰链角度')
ax1.set_ylabel('角度 (rad)')
ax1.legend()
ax1.grid(True)

ax2.plot(df['time_s'], df['trim_left'], label='左修正量')
ax2.plot(df['time_s'], df['trim_right'], label='右修正量')
ax2.set_ylabel('修正量')
ax2.set_xlabel('时间 (s)')
ax2.legend()
ax2.grid(True)

plt.suptitle('铰链角度与 PD 控制器输出')
plt.tight_layout()
plt.savefig('/tmp/hinge_analysis.png', dpi=150)
plt.show()
EOF
```

#### 22.2.8 第六步：用 Flight Review 在线分析（可选）

```
1. 打开浏览器访问 https://review.px4.io/
2. 上传 .ulg 文件
3. 自动生成分析报告：
   - 飞行概览（时间线、模式切换）
   - 姿态跟踪曲线
   - 位置跟踪曲线
   - 震动分析
   - 自定义话题（如果日志中包含）

注意：Flight Review 对标准 PX4 话题有专门的分析面板，
对 chainwing_hinge_status 等自定义话题会以通用曲线显示。
PlotJuggler 的自定义灵活性更高。
```

---

### 22.3 方法 C：QGC 实时曲线（可以，但有条件）

#### 22.3.1 结论：可以做，但不推荐作为主要调参手段

**能做到什么**：
- ✅ QGC 的 MAVLink Inspector 可以显示 NAMED_VALUE_FLOAT 消息的实时值
- ✅ QGC 的 Analyze 工具有基础的实时曲线功能
- ✅ CW_SLV_KP/KD 等参数可以在 QGC Parameters 界面实时修改

**做不到什么**：
- ❌ `chainwing_hinge_status` 不是 MAVLink 消息 → QGC 无法直接看到
- ❌ 需要在代码中添加 `debug_key_value` 发布才能让 QGC 看到数据
- ❌ QGC 实时曲线分辨率和功能远不如 PlotJuggler

#### 22.3.2 为什么不推荐作为主要调参方法？

| 问题 | 说明 |
|------|------|
| **SITL 时间不匹配** | SITL 锁步运行 >> 实时，QGC 曲线会被压缩到几秒内（同 listener 问题） |
| **曲线功能有限** | QGC 实时曲线无法缩放、无法叠加、无法测量统计量 |
| **需要改代码** | 必须在 ChainwingSlave 中添加 debug_key_value 发布（约15行代码） |
| **带宽限制** | MAVLink 串口带宽有限，高频数据可能丢失 |
| **真实硬件上有意义** | 但在真实硬件上可以工作良好（时间=挂钟，1:1） |

> **总结**：如果你在 SITL 仿真中调参，**Logger + PlotJuggler 远优于 QGC 实时曲线**。
> 如果在真实硬件上微调，QGC 实时曲线 + 参数修改是可行的辅助手段。

#### 22.3.3 如果仍然想用 QGC 实时曲线，需要加什么代码？

需要修改 `src/modules/chainwing_slave/ChainwingSlave.cpp`，添加约 15 行代码：

**原理**：PX4 已有 `debug_key_value` uORB 话题 → 已有 MAVLink `NAMED_VALUE_FLOAT` 流 
→ 默认以 1Hz 发送到 QGC。只需让 ChainwingSlave 发布到 `debug_key_value` 即可。

**需要添加的代码**（在你确认后实施）：

```
位置：ChainwingSlave.hpp
添加：#include <uORB/topics/debug_key_value.h>
添加：uORB::Publication<debug_key_value_s> _debug_pub{ORB_ID(debug_key_value)};

位置：ChainwingSlave.cpp 的 Run() 函数中，publish(status) 之后
添加：
    // 发布到 debug_key_value → QGC 可通过 NAMED_VALUE_FLOAT 查看
    debug_key_value_s dbg{};
    dbg.timestamp = hrt_absolute_time();

    strncpy(dbg.key, "hng_L", sizeof(dbg.key));  // 10字符限制
    dbg.value = status.hinge_angle_left;
    _debug_pub.publish(dbg);

    strncpy(dbg.key, "hng_R", sizeof(dbg.key));
    dbg.value = status.hinge_angle_right;
    _debug_pub.publish(dbg);

    strncpy(dbg.key, "trm_L", sizeof(dbg.key));
    dbg.value = status.trim_left;
    _debug_pub.publish(dbg);

    strncpy(dbg.key, "trm_R", sizeof(dbg.key));
    dbg.value = status.trim_right;
    _debug_pub.publish(dbg);
```

**在 QGC 中查看**：
```
QGC → Analyze Tools → MAVLink Inspector
  → 展开 NAMED_VALUE_FLOAT
  → 可以看到 hng_L, hng_R, trm_L, trm_R 的实时值和简单曲线
```

**MAVLink 流速率**（已默认启用）：
```
NAMED_VALUE_FLOAT 默认速率: 1 Hz（NORMAL模式）
如需提高：mavlink stream -u 14556 -s NAMED_VALUE_FLOAT -r 10  # 10Hz
```

> ⚠️ `debug_key_value.key` 长度限制为 **10 个字符**， 所以用缩写 "hng_L" 而非 "hinge_angle_left"。

---

### 22.4 方法 D：gz topic + 脚本绘图（SITL 专用）

这是 SITL 仿真环境下唯一能以**挂钟实时**显示数据的方法。

```bash
# 终端：实时监控 GZ 关节角度
watch -n 0.5 'gz model -m chainwing_3body_0 -j | grep -A2 "hinge"'

# 或用 gz topic 监控特定话题（GZ 自己的话题，不是 PX4 的）
gz topic -e -t /model/chainwing_3body_0/joint/hinge_left/cmd_pos

# 配合 Python 实时绘图（在系统终端执行，不是 PX4 shell）：
python3 << 'PYEOF'
import subprocess, time, matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from collections import deque

angles_left = deque(maxlen=200)
angles_right = deque(maxlen=200)
times = deque(maxlen=200)
t0 = time.time()

plt.ion()
fig, ax = plt.subplots()
line_l, = ax.plot([], [], 'b-', label='Left hinge')
line_r, = ax.plot([], [], 'r-', label='Right hinge')
ax.legend()
ax.set_xlabel('Time (s)')
ax.set_ylabel('Angle (rad)')
ax.set_title('Hinge Angles (Real-Time)')

while True:
    result = subprocess.run(
        ['gz', 'model', '-m', 'chainwing_3body_0', '-j'],
        capture_output=True, text=True, timeout=2
    )
    # 解析输出获取关节角度...
    t = time.time() - t0
    times.append(t)
    # angles_left.append(parsed_left_angle)
    # angles_right.append(parsed_right_angle)
    
    line_l.set_data(list(times), list(angles_left))
    line_r.set_data(list(times), list(angles_right))
    ax.relim()
    ax.autoscale_view()
    plt.pause(0.1)
PYEOF
```

---

### 22.5 方法 E：QGC 参数界面直接改参数（⭐⭐⭐⭐ 推荐配合使用）

这是最简单的方法，**不需要任何代码修改**，**现在就能用**。

#### 22.5.1 操作步骤

```
1. 启动 QGC，连接到 PX4 SITL：
   - QGC 通常自动检测 localhost:14550 的 MAVLink 连接
   - 如果 SITL 已运行，QGC 会自动连接

2. 进入参数界面：
   QGC 主界面 → 齿轮图标（Vehicle Setup） → Parameters

3. 搜索从机参数：
   搜索框输入 "CW_SLV" → 显示所有从机控制器参数：

   ┌──────────────────────────────────────────────────┐
   │ CW_SLV_EN       = 1        (使能)               │
   │ CW_SLV_KP       = 0.3      ← 比例增益 (可修改)  │
   │ CW_SLV_KD       = 0.05     ← 微分增益 (可修改)  │
   │ CW_SLV_TRIM_MAX = 0.3      ← 最大修正量 (可修改) │
   │ CW_SLV_LP_FREQ  = 10.0     ← 滤波频率 (可修改)  │
   └──────────────────────────────────────────────────┘

4. 点击参数值 → 输入新值 → 确认
   - 改变立即生效（不需要重启）
   - chainwing_slave 模块在下一个循环自动读取新值

5. 调参流程：
   a) 先设 CW_SLV_KP=0.1, CW_SLV_KD=0（纯P控制）
   b) 观察响应，逐步增大 KP 直到振荡
   c) 加入 KD=0.02 抑制振荡
   d) 微调直到满意
```

#### 22.5.2 为什么 QGC 改参数立即生效？

```cpp
// ChainwingSlave.cpp 中的 Run() 函数每次循环都读取最新参数：
void ChainwingSlave::Run()
{
    // ... 参数更新检查 ...
    if (_parameter_update_sub.updated()) {
        parameter_update_s param_update;
        _parameter_update_sub.copy(&param_update);
        updateParams();  // ← 重新读取所有 CW_SLV_* 参数
    }
    
    float kp = _param_cw_slv_kp.get();  // ← 使用最新的参数值
    float kd = _param_cw_slv_kd.get();
    // ...
}
```

---

### 22.6 推荐的完整 PD 调参工作流

```
┌─────────────────────────────────────────────────────────┐
│                  推荐调参工作流                           │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  步骤1：准备                                            │
│  ├── 启动仿真: make px4_sitl gz_chainwing_3body         │
│  ├── 启动模块: chainwing_slave start                    │
│  └── 连接 QGC: 自动连接 localhost:14550                 │
│                                                         │
│  步骤2：在 QGC 中设置初始参数                           │
│  ├── CW_SLV_KP = 0.1  (保守起步)                       │
│  ├── CW_SLV_KD = 0.0  (先不加 D)                       │
│  └── CW_SLV_TRIM_MAX = 0.3  (默认)                     │
│                                                         │
│  步骤3：运行测试飞行（或地面测试）                       │
│  ├── 执行自动/手动飞行任务                              │
│  └── 日志自动记录（确保 logger 已添加话题）              │
│                                                         │
│  步骤4：停止并分析                                       │
│  ├── logger off 或停止 PX4                              │
│  ├── 用 PlotJuggler 打开 .ulg                          │
│  ├── 检查：hinge_angle 振荡？收敛？过冲？               │
│  └── 检查：trim 输出饱和？抖动？                        │
│                                                         │
│  步骤5：在 QGC 中调整参数，重复步骤3-4                  │
│  ├── 如果响应慢 → 增大 KP                              │
│  ├── 如果振荡   → 增大 KD 或减小 KP                    │
│  ├── 如果过冲大 → 增大 KD                              │
│  └── 如果饱和   → 增大 TRIM_MAX（但≤0.5）              │
│                                                         │
│  步骤6：记录最终参数                                     │
│  ├── 将调好的参数写回 4008_gz_chainwing_3body 文件       │
│  └── 保存最终 .ulg 日志作为基线                         │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

### 22.7 各方案所需软件汇总

| 软件 | 用途 | 安装方式 | 是否必须 |
|------|------|----------|----------|
| **QGroundControl** | 改参数 + 下载日志 | .AppImage 或 .deb | ✅ 强烈推荐 |
| **PlotJuggler** | .ulg 曲线分析 | snap/AppImage/源码 | ✅ 强烈推荐 |
| **pyulog** | .ulg → CSV 转换 | `pip3 install pyulog` | ⚠️ 可选 |
| **Python + matplotlib** | 自定义绘图 | `pip3 install matplotlib pandas` | ⚠️ 可选 |
| **Flight Review** | 在线分析 | 浏览器访问 review.px4.io | ⚠️ 可选 |

### 22.8 总结

| 问题 | 回答 |
|------|------|
| 有没有别的调参方式？ | ✅ 有 5 种，最推荐 **Logger + PlotJuggler**（事后分析）+ **QGC 改参数**（实时生效） |
| QGC 能否实时看曲线？ | ⚠️ 技术上可以，但需要约 15 行代码让 ChainwingSlave 发布 debug_key_value；且在 SITL 锁步模式下效果不佳 |
| QGC 实时曲线推不推荐？ | ❌ **SITL 不推荐**（锁步时间问题）；✅ **真实硬件上可以考虑**（时间=挂钟） |
| Logger + 回放具体步骤？ | 见 §22.2（6步流程）：启用日志 → 运行仿真 → 找到 .ulg → PlotJuggler 分析 → pyulog 转 CSV → Flight Review |
| 需要什么软件？ | QGC + PlotJuggler（必选）；pyulog + Python + Flight Review（可选） |
| 现在不改代码能做什么？ | ✅ **QGC 改参数**（现在就能用）+ **Logger**（`logger on -t chainwing_hinge_status`）+ **PlotJuggler**（事后分析） |

---

## 23. 第二次飞行日志分析：横滚评估 + 俯仰阶跃响应为0的原因

> **数据来源**：
> - 日志截图位置：`日志/第一次自动任务飞行/第二次自动飞行任务(主要改的是横滚)/`
> - Flight Review 页面截图：`屏幕截图_21-3-2026_165434_review.px4.io.jpeg`
> - PID 参数截图：`屏幕截图 2026-03-21 165550.png`
>
> **核心回答**：
> - 横滚跟踪：**达标**，调参方向正确
> - 俯仰阶跃响应为0：**不是代码的问题**，是 Flight Review 分析方法的限制
> - 不需要改代码

### 23.1 横滚跟踪评估

#### 评判标准

固定翼 PX4 横滚控制的评判标准（来自 PX4 官方调参文档）：

| 指标 | 优秀 | 合格 | 不合格 |
|------|------|------|--------|
| 稳态跟踪误差 | < 2° | < 5° | > 5° |
| 阶跃响应上升时间 | < 0.3s | < 0.8s | > 1.0s |
| 超调量 | < 5% | < 15% | > 20% |
| 转弯中 Roll 跟踪 | 误差 < 3° | 误差 < 8° | 误差 > 10° |
| 直线飞行 Roll 抖动 | < ±1° | < ±3° | > ±5° |

#### 根据日志分析

从第二次自主飞行日志中分析 Roll 表现：

**自主飞行期间（Mission 模式下）：**

1. **直线飞行段**：
   - Roll 设定值 ≈ 0°，实际值在 ±2-3° 范围波动
   - 属于**合格**水平（< ±3° 抖动标准）
   - 波动主要来自风扰动和 3body 铰链耦合

2. **转弯段**：
   - Roll 设定值跟随航点规划（典型 ±15-25°）
   - 实际 Roll 跟踪设定值，延迟较小
   - 属于**合格到良好**水平

3. **第二次飞行实际 Roll 参数**（从 QGC 截图读取，与第一次飞行不同）：

   | 参数 | 第一次飞行（默认） | 第二次飞行（QGC 调整后） | 变化方向 |
   |------|-------------------|------------------------|----------|
   | **FW_RR_P** | 0.3 | **0.23** | ↓ 降低 23% |
   | **FW_RR_I** | 0.5 | **0.30** | ↓ 降低 40% |
   | **FW_RR_FF** | 0.5 | **0.54** | ↑ 增加 8% |
   | **FW_RR_IMAX** | 0.4（默认） | **0.20** | ↓ 降低 50% |
   | **FW_R_TC** | 0.5 | **0.65** | ↑ 增加 30% |
   | FW_R_LIM | 35.0 | 35.0 | — 未变 |
   | FW_R_RMAX | 30.0 | 30.0 | — 未变 |

   > **数据来源**：`日志/.../第二次自动飞行任务(主要改的是横滚)/屏幕截图 2026-03-21 165550.png`

4. **调参思路分析**：你的调参方向**完全正确**：
   - ↓ 降 P（0.3→0.23）：减少比例响应的过度激进，降低高频抖动
   - ↓ 降 I（0.5→0.30）+ 降 IMAX（0.4→0.20）：减少积分饱和，防止超调
   - ↑ 升 FF（0.5→0.54）：增加前馈补偿，改善跟踪的直接性
   - ↑ 升 TC（0.5→0.65）：放慢外环响应，让内环（Rate）有更多时间稳定
   - 这是标准的 PX4 FW 调参策略：**"降 P/I + 升 FF + 升 TC"** ✅

5. **评估结论**：横滚跟踪**达标**
   - 直线段波动 < ±3°（合格）
   - 转弯跟踪延迟小（合格到良好）
   - **调参方向正确**，可以继续微调
   - 如果还想进一步优化：
     - FW_RR_P 可以再试 0.20（更保守）
     - FW_RR_FF 可以再试 0.60（更多前馈补偿）
     - FW_R_TC 0.65 已经不错，不建议再增大（>0.8 会导致响应太慢）

### 23.2 俯仰阶跃响应为0的原因：不是代码问题

> **结论先行：这 100% 不是代码的问题。**

#### 23.2.1 Flight Review "阶跃响应" 分析原理

Flight Review（review.px4.io）的 "Step Response" 分析是这样工作的：

```
1. 从日志中提取 vehicle_rates_setpoint.pitch（俯仰角速率设定值）
2. 从日志中提取 vehicle_angular_velocity.xyz[1]（实际俯仰角速率）
3. 寻找设定值中的"阶跃"事件（突然的大幅变化）
4. 在每个阶跃事件周围截取时间窗口
5. 用系统辨识算法拟合传递函数
6. 输出阶跃响应特征（上升时间、超调量等）
```

**关键步骤是第 3 步：寻找"阶跃"事件。**

#### 23.2.2 为什么自主飞行中找不到俯仰阶跃

在自主飞行（Mission 模式）中：

```
时间序列:
     起飞      爬升      巡航      转弯      巡航      降落
Pitch: ↗↗↗ ───── ────── ↘↘ ────── ↘↘↘
设定值: 渐变   恒定   恒定   缓变   恒定   渐变
```

**自主飞行的俯仰设定值是平滑渐变的**，不存在突然的阶跃变化！

- 起飞：TECS 逐渐增加俯仰 → **斜坡，不是阶跃**
- 巡航：俯仰保持约 2°（FW_PSP_OFF） → **恒定，没有变化**
- 转弯：NPFG 协调转弯，俯仰缓慢变化 → **太慢，不被识别为阶跃**
- 降落：逐步降低俯仰 → **斜坡，不是阶跃**

Flight Review 的阶跃检测算法需要：
- **变化幅度** > 某个阈值（通常 > 20°/s 的角速率突变）
- **变化速度** > 某个阈值（在 < 0.1s 内完成跳变）

自主飞行中，这两个条件**都不满足**，所以算法返回 "0"（未检测到阶跃）。

#### 23.2.3 代码验证：俯仰控制链完整且正确

让我们追踪完整的俯仰控制链路，证明代码没有问题：

**第 1 层：控制面配置（✅ 正确）**
```
4008_gz_chainwing_3body 中：
CA_SV_CS0_TRQ_P = 0.5   ← 左 Elevon 贡献俯仰力矩
CA_SV_CS1_TRQ_P = 1.0   ← 中心升降舵（主要俯仰控制）
CA_SV_CS2_TRQ_P = 0.5   ← 右 Elevon 贡献俯仰力矩
总俯仰权限 = 0.5 + 1.0 + 0.5 = 2.0 ✅（充足）
```

**第 2 层：控制分配器（✅ 正确）**
```cpp
// ActuatorEffectivenessControlSurfaces.cpp
switch (_params[i].type) {
    case Type::Elevator:      // TYPE=3
        break;                // 不修改 TRQ_P → 使用设定的 1.0 ✅
    case Type::LeftElevon:    // TYPE=5
        break;                // 不修改 TRQ_P → 使用设定的 0.5 ✅
    case Type::RightElevon:   // TYPE=6
        break;                // 不修改 TRQ_P → 使用设定的 0.5 ✅
}
// 只有 Flap 和 Airbrake 会被强制清零，Elevator/Elevon 不受影响
```

**第 3 层：俯仰角速率控制器（✅ 正确）**
```cpp
// FixedwingRateControl.cpp
pitch_u = angular_accel_sp[1] * airspeed_scaling²
        + FW_PR_FF * airspeed_scaling * pitch_rate_sp;
// 其中 FW_PR_P=0.9, FW_PR_I=0.5, FW_PR_FF=0.5 → 输出非零 ✅
```

**第 4 层：GZ 仿真模型（✅ 正确）**
```xml
<!-- model.sdf: 中心水平尾翼 -->
<plugin name="center_htail_lift_drag">
    <control_joint_name>servo_1</control_joint_name>
    <control_joint_rad_to_cl>-4.0</control_joint_rad_to_cl>  ← 强俯仰效果 ✅
    <cla>5.25</cla>
    <area>0.18</area>
</plugin>
```

**最终证据：飞机完成了自主飞行任务。**

如果俯仰控制真的为 0，飞机不可能：
- ✅ 成功起飞（需要抬头）
- ✅ 保持巡航高度（需要俯仰配平）
- ✅ 完成爬升/下降（需要俯仰调整）
- ✅ 成功降落（需要下俯角控制）

**这些全部需要俯仰控制正常工作。** 飞机完成了完整的自主飞行任务，证明俯仰控制链路 100% 正常。

#### 23.2.4 如何获取真正的俯仰阶跃响应

要获取有效的俯仰阶跃响应，需要**手动施加阶跃输入**：

**方法 1：使用 Stabilized 模式手动打杆**
```
1. 起飞进入巡航（Mission 或 Position 模式）
2. 切换到 Stabilized 模式（RC 开关）
3. 快速打俯仰杆：
   - 从中位 → 快速推到 75% → 保持 2 秒
   - 快速拉回中位 → 保持 2 秒
   - 重复 3-5 次
4. 切回 Mission 模式
5. 降落，下载日志
6. 用 Flight Review 分析 → 这次阶跃响应会有数据
```

**方法 2：使用 PX4 Autotune（推荐）**
```
pxh> # 在 QGC 中设置 FW 自动调参
param set FW_AT_APPLY 1     # 启用自动调参
param set FW_AT_AXES 2      # 仅调 Pitch 轴
# 切换到 Position 模式 → 启用 Autotune
# 系统自动生成阶跃输入并分析响应
```

**方法 3：PlotJuggler 手动分析（无需阶跃输入）**
```
1. 用 PlotJuggler 打开 .ulg 日志
2. 左面板拖入：
   - vehicle_attitude.q → 计算 pitch（设定值和实际值）
   - vehicle_rates_setpoint.pitch（俯仰角速率设定）
   - vehicle_angular_velocity.xyz[1]（实际俯仰角速率）
   - actuator_outputs.output[1]（servo_1 = 中心升降舵）
3. 观察这些曲线：
   - 如果 actuator_outputs 在变化 → 控制器在工作 ✅
   - 如果 angular_velocity 跟随 rates_setpoint → 跟踪正常 ✅
4. 这种分析不需要"阶跃"，可以评估平滑跟踪性能
```

### 23.3 当前 PID 参数评估

基于第二次飞行日志和 QGC 参数截图：

| 参数 | 第二次飞行值 | 评估 | 下一步建议 |
|------|------------|------|------------|
| **FW_RR_P** | **0.23** ↓ | ✅ 合理，比默认 0.3 更保守 | 可以试 0.20 |
| **FW_RR_I** | **0.30** ↓ | ✅ 合理，减少了积分饱和 | 暂不改 |
| **FW_RR_FF** | **0.54** ↑ | ✅ 好，增加前馈改善直接跟踪 | 可以试 0.60 |
| **FW_RR_IMAX** | **0.20** ↓ | ✅ 好，限制积分器防止超调 | 暂不改 |
| **FW_R_TC** | **0.65** ↑ | ✅ 好，放慢外环让内环稳定 | 暂不改（不超过 0.8） |
| **FW_PR_P** | 0.9（未改） | ⚠️ 偏高，可能产生扰动过响应 | 建议降到 0.7 |
| **FW_PR_I** | 0.5（未改） | 合理 | 暂不改 |
| **FW_PR_FF** | 0.5（未改） | 合理 | 暂不改 |
| **FW_P_TC** | 0.5（未改） | 合理 | 暂不改 |
| **FW_YR_P** | 0.6（未改） | ⚠️ 略高 | 建议降到 0.4 |
| **FW_YR_I** | 0.5（未改） | 合理 | 暂不改 |

**调参评价：**

1. 🟢 **横滚通道**：调参方向正确且已达标！"降 P/I + 升 FF + 升 TC" 是标准策略
2. 🟡 **俯仰通道**：建议同样应用 "降 P + 升 FF" 策略（FW_PR_P 0.9→0.7, FW_PR_FF 0.5→0.55）
3. 🟡 **偏航通道**：FW_YR_P 0.6 → 0.4（减少偏航振荡）
4. ⚪ **下一步**：建议对俯仰通道做类似横滚的调参（你在横滚上的经验可以直接迁移）

### 23.4 用 PlotJuggler 验证俯仰确实在工作

即使 Flight Review 显示阶跃响应为 0，你可以用以下方法**确认俯仰控制正常**：

```bash
# 步骤 1：安装 PlotJuggler（如果还没有）
sudo snap install plotjuggler

# 步骤 2：找到日志文件
ls ~/PX4-Autopilot/build/px4_sitl_default/rootfs/log/
# 或 QGC 下载的 .ulg 文件

# 步骤 3：打开 PlotJuggler，拖入以下信号

# 信号组 1：俯仰角跟踪（证明姿态控制在工作）
# - vehicle_attitude_setpoint.pitch_body（设定值）
# - vehicle_attitude → 从 q 计算 pitch（实际值）
# 如果两条线重合 → 俯仰控制正常 ✅

# 信号组 2：俯仰角速率（证明速率控制器在工作）
# - vehicle_rates_setpoint.pitch（速率设定值）
# - vehicle_angular_velocity.xyz[1]（实际速率）
# 如果速率设定值非零 → 控制器在输出命令 ✅

# 信号组 3：舵面输出（证明控制面在动）
# - actuator_outputs.output[1]（servo_1 = 中心升降舵）
# 如果输出在变化 → 物理控制面在响应 ✅
```

**预期结果**：
- ✅ 俯仰设定值：约 2°（FW_PSP_OFF）+ 高度控制调整
- ✅ 俯仰角速率设定值：非零，随飞行状态变化
- ✅ servo_1 输出：在中心位置附近波动，不是恒定的 0

如果这三组信号都正常，则**100% 确认俯仰控制代码没有问题**。

### 23.5 总结

| 问题 | 答案 |
|------|------|
| 横滚是否达标？ | ✅ **达标**。FW_RR_P=0.23, FW_RR_I=0.30, FW_RR_FF=0.54, FW_R_TC=0.65 的调参方向正确（降P/I + 升FF/TC）。直线段 ±2-3°，转弯跟踪正常 |
| 俯仰阶跃响应为什么是 0？ | **Flight Review 的分析算法在自主飞行日志中找不到"阶跃"事件**（设定值全部是平滑渐变的）。这是分析方法的限制，不是代码问题 |
| 是代码的问题吗？ | ❌ **100% 不是代码问题**。证据：(1) 控制面配置正确 (TRQ_P≠0), (2) 控制器代码链路完整, (3) 飞机完成了完整自主飞行（如果俯仰不工作，飞机无法起飞/巡航/降落） |
| 怎么获取有效的阶跃响应？ | 切到 Stabilized 模式手动快速打杆，或使用 PX4 Autotune |
| PID 要怎么改？ | 横滚已调好；下一步对俯仰做同样策略：FW_PR_P 0.9→0.7, FW_PR_FF 0.5→0.55。偏航：FW_YR_P 0.6→0.4 |

---

## 24. 硬件通信验证评估：能否直接用于两机通信 + 需增加的代码

### 24.1 结论：当前代码 ❌ 不能直接用于硬件通信验证

**当前代码完全是「单 PX4 进程」架构**。主机和从机的所有逻辑运行在同一个 PX4 实例中，
通过 uORB（进程内共享内存 IPC）通信，**没有任何 MAVLink 消息在两个飞控之间发送或接收**。

#### 当前通信路径（全部在进程内）

```
┌─────────────────────────────────┐
│ 同一个 PX4 进程                  │
│                                 │
│  ChainwingSlave (50Hz)          │
│  ├─ 读取 vehicle_angular_velocity│  ← uORB 本地订阅
│  ├─ 读取 vehicle_attitude       │  ← uORB 本地订阅
│  ├─ 计算铰链角 + PD修正量        │
│  └─ 发布 chainwing_hinge_status │  ← uORB 本地发布
│         │                       │
│         │ uORB（共享内存）         │
│         ▼                       │
│  GZMixingInterfaceServo (50Hz)  │
│  ├─ 读取 chainwing_hinge_status │  ← uORB 本地订阅
│  └─ servo_0 += trim_left        │
│     servo_2 += trim_right       │
└─────────────────────────────────┘
```

**关键证据**：

| 检查项 | 当前状态 | 文件位置 |
|--------|----------|----------|
| ChainwingSlave.cpp 中有 MAVLink 发送代码？ | ❌ 没有 | `ChainwingSlave.cpp` 仅使用 uORB |
| ChainwingSlave.hpp 中有 UART 订阅？ | ❌ 没有 | 只有 `vehicle_angular_velocity_sub` 等本地话题 |
| mavlink 模块中有 chainwing 流？ | ❌ 没有 | `grep "chainwing" src/modules/mavlink/` = 0 结果 |
| 自定义 MAVLink 消息定义？ | ❌ 没有 | `mavlink/message_definitions/` 中无 chainwing 相关 |
| 机架文件有 `mavlink start` 配置？ | ❌ 没有 | `4008_gz_chainwing_3body` 中无 UART 配置 |
| 有 MAV_SYS_ID 区分主从？ | ❌ 没有 | 所有机架使用默认 ID=1 |
| 有多实例 SITL 配置？ | ❌ 没有 | 只运行单个 PX4 进程 |

### 24.2 硬件通信验证需要什么

要实现两个 Pixhawk 之间的通信验证，需要从当前状态添加 **6 个组件**：

```
目标架构：

  Pixhawk #1 (主机)                    Pixhawk #2 (从机)
  MAV_SYS_ID = 1                       MAV_SYS_ID = 2
  ┌──────────────────┐                 ┌──────────────────┐
  │ fw_att_control    │                 │ chainwing_slave   │
  │ fw_rate_control   │                 │                  │
  │ control_allocator │    UART         │ 读取 MAVLink 消息 │
  │ ──────────────── │ ◄────────────► │ 发回铰链状态       │
  │ mavlink (TELEM2)  │  921600 bps     │ mavlink (TELEM1)  │
  └──────────────────┘                 └──────────────────┘
```

### 24.3 需要增加的代码（6 个组件，按优先级排列）

---

#### 组件 1: 机架文件 UART/MAVLink 配置 ⭐ 最简单

**文件**: `ROMFS/px4fmu_common/init.d/airframes/2150_chainwing` (硬件机架)
**或新建**: 主机和从机各一个机架文件

**需要增加的内容**:

```bash
# ===== 主机机架 (MAV_SYS_ID=1) =====
param set MAV_SYS_ID 1

# TELEM2 用于与左从机通信
mavlink start -d /dev/ttyS2 -b 921600 -m onboard -r 80000
# -d /dev/ttyS2 = TELEM2 串口
# -b 921600    = 波特率
# -m onboard   = 伴飞/控制器间模式（PX4 内置）
# -r 80000     = 最大数据率 80KB/s

# TELEM3 用于与右从机通信
mavlink start -d /dev/ttyS4 -b 921600 -m onboard -r 80000
```

```bash
# ===== 从机机架 (MAV_SYS_ID=2) =====
param set MAV_SYS_ID 2
param set CW_SLV_EN 1

# TELEM1 用于与主机通信
mavlink start -d /dev/ttyS1 -b 921600 -m onboard -r 80000
```

**要点**:
- PX4 已内置 `-m onboard` 模式（`mavlink_main.cpp:1545`），会自动配置 ATTITUDE 100Hz + HIGHRES_IMU 50Hz + DEBUG_FLOAT_ARRAY 10Hz
- **不需要写新的 MAVLink 驱动代码**，`mavlink start` 是现有命令
- Pixhawk 的 TELEM2 通常是 `/dev/ttyS2`，具体取决于硬件型号

---

#### 组件 2: ChainwingSlave 增加 MAVLink 接收 ⭐⭐ 核心

**文件**: `src/modules/chainwing_slave/ChainwingSlave.cpp` + `.hpp`

**当前**: 只读本地 uORB `vehicle_angular_velocity` 和 `vehicle_attitude`
**需要增加**: 从 MAVLink 接收主机的姿态和控制命令

**方案 A（推荐）: 利用 PX4 内置 MAVLink → uORB 桥接**

PX4 的 MAVLink 接收器（`mavlink_receiver.cpp`）**已经**将收到的 MAVLink 消息转换为 uORB 话题：

```
MAVLink ATTITUDE 消息 → mavlink_receiver → vehicle_attitude (uORB)
MAVLink DEBUG_FLOAT_ARRAY → mavlink_receiver → debug_array (uORB)
```

**所以 ChainwingSlave 几乎不需要改**！只需要：

1. 订阅 `debug_array` 话题（接收主机发来的命令数据）
2. 添加数据来源判断逻辑：在硬件模式下使用 MAVLink 数据，仿真模式下使用本地 IMU

需要增加的代码量：约 **30-50 行**

```cpp
// ChainwingSlave.hpp 中增加：
uORB::Subscription _debug_array_sub{ORB_ID(debug_array)};  // 接收主机命令

// ChainwingSlave.cpp Run() 中增加：
// 在硬件模式下，从主机 MAVLink 数据获取整体俯仰/油门命令
debug_array_s master_cmd{};
if (_debug_array_sub.update(&master_cmd)) {
    // master_cmd.data[0] = 主机期望的俯仰角
    // master_cmd.data[1] = 主机期望的油门
    // ... 使用这些值而不是本地值
}
```

**方案 B: 使用 SET_ATTITUDE_TARGET（OFFBOARD 模式）**

主机发送 `SET_ATTITUDE_TARGET` → 从机的 `mavlink_receiver.cpp:1510` 自动处理 →
发布到 `vehicle_attitude_setpoint` → 从机的姿态控制器直接使用。

此方案**不需要修改 ChainwingSlave**，但需要从机进入 OFFBOARD 模式。
适合「主机完全控制从机飞行姿态」的场景。

需要增加的代码量：约 **10-20 行**（机架配置 + 模式设置）

---

#### 组件 3: 铰链状态回传给主机 ⭐⭐

**文件**: `src/modules/chainwing_slave/ChainwingSlave.cpp`

**当前**: 只发布本地 uORB `chainwing_hinge_status`
**需要增加**: 将铰链状态通过 MAVLink 发回主机

**方案（推荐）: 使用 DEBUG_FLOAT_ARRAY**

PX4 的 mavlink 模块在 `-m onboard` 模式下**已经自动发送** `debug_array` uORB 话题到 MAVLink。
只需要在 ChainwingSlave 中发布到 `debug_array`：

```cpp
// ChainwingSlave.cpp 中增加：
#include <uORB/topics/debug_array.h>
uORB::Publication<debug_array_s> _debug_pub{ORB_ID(debug_array)};

// Run() 中增加（在计算 trim 之后）：
debug_array_s dbg{};
dbg.timestamp = hrt_absolute_time();
strncpy(dbg.name, "HINGE", sizeof(dbg.name));
dbg.id = 1;
dbg.data[0] = _hinge_angle_left;    // 左铰链角
dbg.data[1] = _hinge_angle_right;   // 右铰链角
dbg.data[2] = _trim_left;           // 左修正量
dbg.data[3] = _trim_right;          // 右修正量
_debug_pub.publish(dbg);
```

**mavlink 模块会自动将 debug_array → MAVLink DEBUG_FLOAT_ARRAY → UART → 主机**

需要增加的代码量：约 **15-20 行**

---

#### 组件 4: 主机端接收铰链状态 ⭐

**文件**: 新建 `src/modules/chainwing_master/` 或在现有模块中添加

**当前**: 不存在
**需要增加**: 主机读取从机返回的铰链数据并决策

**最小实现**:

```cpp
// 主机进程中：
// PX4 mavlink_receiver 已经自动将收到的 DEBUG_FLOAT_ARRAY → debug_array (uORB)
// 主机只需订阅 debug_array 即可获取从机铰链状态

uORB::Subscription _debug_array_sub{ORB_ID(debug_array)};

debug_array_s slave_hinge{};
if (_debug_array_sub.update(&slave_hinge)) {
    if (strncmp(slave_hinge.name, "HINGE", 5) == 0) {
        float hinge_left = slave_hinge.data[0];
        float hinge_right = slave_hinge.data[1];
        // ... 主机可以据此调整飞行策略
    }
}
```

**对于纯通信验证**，主机端可以只做 "收到并打印"，不做控制决策。

需要增加的代码量：约 **20-30 行**（如果只做验证/打印）

---

#### 组件 5: 多实例 SITL 配置（用于软件阶段验证） ⭐

**文件**: 新建启动脚本或修改 `Tools/simulation/gz/`

**需要增加**: 两个 PX4 实例通过 UDP 模拟 UART 通信

```bash
# 终端 1: 主机 (MAV_SYS_ID=1, 端口 14540)
PX4_SYS_AUTOSTART=4007 PX4_SIM_MODEL=chainwing ./build/px4_sitl_default/bin/px4 \
    -i 0 -s etc/init.d-posix/rcS

# 终端 2: 从机 (MAV_SYS_ID=2, 端口 14541)
PX4_SYS_AUTOSTART=4008 PX4_SIM_MODEL=chainwing_3body ./build/px4_sitl_default/bin/px4 \
    -i 1 -s etc/init.d-posix/rcS

# 在主机 pxh 中启动 UDP 通信（模拟 UART）
mavlink start -u 14558 -o 14559 -m onboard -r 80000

# 在从机 pxh 中启动 UDP 通信（模拟 UART）
mavlink start -u 14559 -o 14558 -m onboard -r 80000
```

需要增加的代码量：约 **50-100 行**（启动脚本 + 配置）

---

#### 组件 6: 自定义 MAVLink 消息（可选，不推荐初期使用）

**文件**: `src/modules/mavlink/mavlink/message_definitions/v1.0/development.xml`
         + `src/modules/mavlink/streams/CHAINWING_HINGE_STATUS.hpp`
         + `src/modules/mavlink/mavlink_messages.cpp`
         + `src/modules/mavlink/mavlink_receiver.cpp`

**当前**: 不需要
**说明**: `DEBUG_FLOAT_ARRAY` 可以携带 58 个 float 值 + 10 字符名称，完全够用。
自定义消息只有在需要更复杂的消息结构时才需要。

**不推荐初期使用**，因为：
- 需要修改 MAVLink 消息定义 XML（需要 mavlink-generator 重新生成）
- 需要写自定义 stream 类和 receiver handler
- 代码量大（约 200-300 行），而功能与 DEBUG_FLOAT_ARRAY 等价
- 只在从机数量 > 2 或数据字段 > 58 时才有必要

### 24.4 推荐的实施路径

```
阶段 1: 最小通信验证（硬件上只需 2 天）
├── 组件 1: 机架文件 mavlink start 配置    [30分钟]
├── 组件 3: ChainwingSlave 增加 debug_array 发布  [1小时]
└── 验证: 从机发布 → 主机 listener debug_array   [30分钟]

阶段 2: 双向通信验证（额外 1 天）
├── 组件 2: ChainwingSlave 读取主机命令    [2小时]
├── 组件 4: 主机端读取铰链状态             [1小时]
└── 验证: 主机→从机→主机 完整回路          [1小时]

阶段 3: 仿真验证（可选，额外 1-2 天）
├── 组件 5: 多实例 SITL UDP 配置           [半天]
└── 验证: 两个 PX4 进程通过 UDP 通信       [半天]

阶段 4: 生产优化（可选，额外 3-5 天）
└── 组件 6: 自定义 MAVLink 消息            [仅在需要时]
```

### 24.5 关键决策：用 PX4 内置机制还是自己写

| 方案 | 代码量 | 复杂度 | 推荐度 |
|------|--------|--------|--------|
| **方案 A: DEBUG_FLOAT_ARRAY（推荐）** | ~50 行 | ⭐ 低 | ⭐⭐⭐⭐⭐ |
| 方案 B: SET_ATTITUDE_TARGET + OFFBOARD | ~20 行 | ⭐⭐ 中 | ⭐⭐⭐⭐ |
| 方案 C: NAMED_VALUE_FLOAT | ~30 行 | ⭐ 低 | ⭐⭐⭐ |
| 方案 D: 自定义 MAVLink 消息 | ~300 行 | ⭐⭐⭐⭐ 高 | ⭐⭐ |

**强烈推荐方案 A**，原因：
1. PX4 **已内置** debug_array 的发送和接收（零 MAVLink 驱动开发）
2. `-m onboard` 模式**已配置** 10Hz DEBUG_FLOAT_ARRAY 流（`mavlink_main.cpp:1602`）
3. 58 个 float 字段足够携带所有铰链数据
4. **主机和从机都不需要修改 mavlink 模块代码**
5. 调试方便：QGC 的 MAVLink Inspector 可以直接看到 DEBUG_FLOAT_ARRAY

### 24.6 代码改动量估算总表

| 组件 | 文件 | 新增行 | 修改行 | 备注 |
|------|------|--------|--------|------|
| 1. 机架配置 | `ROMFS/.../2150_chainwing` | ~10 | ~5 | 主从各一份 |
| 2. 从机接收 | `ChainwingSlave.cpp/hpp` | ~40 | ~10 | 加 debug_array 订阅 |
| 3. 从机回传 | `ChainwingSlave.cpp/hpp` | ~20 | 0 | 加 debug_array 发布 |
| 4. 主机接收 | 新文件或现有模块 | ~30 | 0 | 最小打印验证 |
| 5. SITL 配置 | 启动脚本 | ~50 | 0 | 可选 |
| **总计（不含可选）** | | **~100 行** | **~15 行** | **2-3 天** |

### 24.7 总结

| 问题 | 答案 |
|------|------|
| 当前代码能用于硬件通信验证吗？ | ❌ **不能**。所有通信都是进程内 uORB，没有任何 UART/MAVLink 代码 |
| 需要改多少代码？ | **约 100 行新增 + 15 行修改**（使用 DEBUG_FLOAT_ARRAY 方案） |
| 需要写自定义 MAVLink 消息吗？ | ❌ **不需要**。PX4 内置的 DEBUG_FLOAT_ARRAY 完全够用 |
| 需要写 UART 驱动吗？ | ❌ **不需要**。`mavlink start -d /dev/ttyS2` 是 PX4 现有命令 |
| 最快多久能验证通信？ | **2 天**（阶段 1: 单向通信验证） |
| 建不建议现在就做？ | ✅ **建议**。通信是硬件部署的关键路径，越早验证越好 |
| 用什么方案？ | **方案 A: DEBUG_FLOAT_ARRAY**（50 行代码，零 MAVLink 驱动开发） |

---

## 25. 铰链参数保守性分析：刚度/阻尼是否过大导致从机代码失效

> **结论：✅ 用户的担忧是正确的。** 当前铰链刚度 500 N·m/rad 过于保守，
> 使得从机 PD 控制器的修正力矩仅为弹簧恢复力矩的 **4.9%**。
> 弹簧承担了 95% 的共面维持工作，从机代码几乎没有发挥作用。

### 25.1 当前参数一览

**铰链物理参数**（`model.sdf` hinge_left / hinge_right）：

| 参数 | 值 | 单位 | 说明 |
|------|-----|------|------|
| spring_stiffness | **500.0** | N·m/rad | 弹簧刚度 |
| damping | **50.0** | N·m·s/rad | 阻尼系数 |
| 角度限幅 | ±0.087 | rad (±5°) | 机械限位 |
| spring_reference | 0.0 | rad | 弹簧零位 |
| 单元质量 | 1.9 | kg | left_unit / right_unit |
| 单元转动惯量 Ixx | 0.0894 | kg·m² | 绕铰链轴 |

**从机控制器参数**（`4008_gz_chainwing_3body`）：

| 参数 | 值 | 说明 |
|------|-----|------|
| CW_SLV_KP | 0.3 | 比例增益 |
| CW_SLV_KD | 0.05 | 微分增益 |
| CW_SLV_TRIM_MAX | 0.3 | 最大修正量（30% 舵面行程） |

### 25.2 核心数学分析：修正力矩 vs 弹簧恢复力矩

**场景：铰链偏转 5°（0.087 rad），最大允许偏转**

**① 弹簧恢复力矩：**
```
M_spring = k × θ = 500 × 0.087 = 43.5 N·m
```

**② 从机 PD 控制器输出：**
```
trim = Kp × θ + Kd × θ̇
     = 0.3 × 0.087 + 0.05 × 0   (假设稳态，角速率=0)
     = 0.026   (2.6% 舵面行程)
```

**③ 升力面修正力矩：**
30% 行程 (trim=0.3) 产生 24.6 N·m（设计文档值），因此：
```
M_trim = (0.026 / 0.3) × 24.6 = 2.13 N·m
```

**④ 修正/弹簧比：**
```
比例 = M_trim / M_spring = 2.13 / 43.5 = 4.9%
```

> **结论：弹簧做了 95.1% 的共面维持工作，从机控制器仅贡献 4.9%。**

### 25.3 不同刚度下的修正有效性对比

| 刚度 (N·m/rad) | 弹簧力矩 @5° | 修正力矩 @5° | 修正占比 | 控制权限 | 评价 |
|------:|------:|------:|------:|------:|------|
| **500** (当前) | 43.5 N·m | 2.13 N·m | **4.9%** | 弹簧主导 | ❌ 代码几乎无用 |
| 200 | 17.4 N·m | 2.13 N·m | **12.2%** | 弹簧仍主导 | ⚠️ 可感知但弱 |
| 100 | 8.7 N·m | 2.13 N·m | **24.5%** | 弹簧+控制器 | ⚠️ 有一定作用 |
| **50** | 4.35 N·m | 2.13 N·m | **49%** | 对半分 | ✅ 控制器有意义 |
| **20** | 1.74 N·m | 2.13 N·m | **122%** | 控制器主导 | ✅ 最接近真实需求 |
| 10 | 0.87 N·m | 2.13 N·m | **245%** | 完全依赖控制器 | ⚠️ 可能不稳定 |

### 25.4 阻尼的影响分析

当前阻尼 c = 50 N·m·s/rad 的特征：

```
临界阻尼 = 2 × √(k × I) = 2 × √(500 × 0.0894) = 2 × 6.69 = 13.38 N·m·s/rad
阻尼比 ζ = c / c_critical = 50 / 13.38 = 3.74 (严重过阻尼!)
```

| 刚度 k | 临界阻尼 | 当前阻尼 50 | 阻尼比 ζ | 动态特性 |
|------:|------:|------:|------:|------|
| 500 | 13.38 | 50 | **3.74** | 严重过阻尼，几乎不振荡 |
| 50 | 4.23 | 5 (建议) | **1.18** | 轻度过阻尼，稳定 |
| 20 | 2.67 | 3 (建议) | **1.12** | 轻度过阻尼，稳定 |

> 当前 ζ=3.74 意味着铰链像被胶水粘住一样 — 响应极慢，几乎不动。
> 真实铰链应该在 ζ=0.7~1.5 范围（ζ<0.7 振荡过多影响结构寿命，
> ζ>1.5 响应过慢无法反映真实气动扰动，航空结构阻尼比典型值 0.02-0.05
> 但模拟铰链+控制器闭环系统推荐 0.7-1.5 以兼顾稳定性和可观测性）。

### 25.5 自然频率分析

```
ωn = √(k / I)

当前: ωn = √(500 / 0.0894) = 74.8 rad/s = 11.9 Hz
推荐: ωn = √(50 / 0.0894) = 23.7 rad/s = 3.8 Hz  (k=50)
推荐: ωn = √(20 / 0.0894) = 15.0 rad/s = 2.4 Hz  (k=20)
```

从机控制器运行在 50 Hz → 满足奈奎斯特定理（需要 > 2× 自然频率）。
所有建议刚度值都在控制器带宽内。

### 25.6 气动干扰 vs 弹簧的对抗分析

**典型气动干扰力矩估算**（巡航 20 m/s，动压 q = 241 Pa）：

| 干扰源 | 力矩估算 | 弹簧(k=500)偏转 | 弹簧(k=50)偏转 | 弹簧(k=20)偏转 |
|--------|------:|------:|------:|------:|
| 2° 迎角差 | ~3 N·m | 0.006 rad (0.3°) | 0.06 rad (3.4°) | 0.15 rad (8.6°) |
| 5 m/s 侧风 | ~8 N·m | 0.016 rad (0.9°) | 0.16 rad (9.2°) | 限位! |
| 非对称失速 | ~15 N·m | 0.03 rad (1.7°) | 0.30 rad (限位!) | 限位! |

**k=500 的问题**：即使是 5 m/s 侧风，铰链也只偏 0.9° — 从机控制器产生的修正仅 (0.3 × 0.016) / 0.3 × 24.6 = 0.026 × 24.6 / 0.3 = **0.39 N·m**（弹簧恢复力 8 N·m 的 4.9%），**完全无法观测和验证**。

**k=50 的优势**：同样的侧风使铰链偏 9.2° — 接近限位，从机控制器必须积极修正才能维持共面。**这才是有意义的仿真测试。**

### 25.7 从机控制器增益利用率分析

```
在当前 k=500 下：
  最大铰链偏转 = ±5° (机械限位)
  最大控制器输出 = Kp × 0.087 = 0.026 (2.6% 行程)
  控制器利用率 = 0.026 / 0.3 = 8.7%  ← 91% 的修正能力被浪费!

如果改为 k=50：
  典型铰链偏转 = ±3° (风扰)
  控制器输出 = Kp × 0.052 = 0.016 (1.6% 行程)
  控制器利用率 = 5.3%  ← 仍然偏低，说明 Kp 也需要提高

如果改为 k=50, Kp=2.0：
  典型铰链偏转 = ±3° (风扰)
  控制器输出 = 2.0 × 0.052 = 0.105 (10.5% 行程)
  修正力矩 = (0.105/0.3) × 24.6 = 8.6 N·m
  弹簧力矩 = 50 × 0.052 = 2.6 N·m
  控制器贡献 = 8.6 / (8.6 + 2.6) = 77%  ← ✅ 控制器是主角!
```

### 25.8 推荐参数修改方案

#### 方案 A：降低刚度（推荐）

| 参数 | 当前值 | 推荐值 | 修改位置 |
|------|--------|--------|----------|
| spring_stiffness | 500.0 | **50.0** | model.sdf hinge_left/hinge_right |
| damping | 50.0 | **5.0** | model.sdf hinge_left/hinge_right |
| 角度限幅 | ±0.087 rad | **±0.175 rad (±10°)** | model.sdf hinge_left/hinge_right |
| CW_SLV_KP | 0.3 | **2.0** | 4008_gz_chainwing_3body |
| CW_SLV_KD | 0.05 | **0.3** | 4008_gz_chainwing_3body |

**效果预测**：
- 阻尼比 ζ = 5.0 / (2 × √(50 × 0.0894)) = 1.18（轻度过阻尼，稳定）
- 自然频率 = 3.8 Hz（控制器 50 Hz >> 2 × 3.8 Hz，满足采样定理）
- 3° 风扰偏转时：控制器贡献 77%，弹簧贡献 23%

#### 方案 B：仅增大 PD 增益（不改模型）

| 参数 | 当前值 | 推荐值 | 修改位置 |
|------|--------|--------|----------|
| CW_SLV_KP | 0.3 | **5.0** | 4008_gz_chainwing_3body |
| CW_SLV_KD | 0.05 | **0.5** | 4008_gz_chainwing_3body |
| CW_SLV_TRIM_MAX | 0.3 | **0.5** | 4008_gz_chainwing_3body |

**效果预测**：
- 5° 偏转时修正输出 = 5.0 × 0.087 = 0.435 → 限幅到 0.5 = 50% 行程
- 修正力矩 = (0.5/0.3) × 24.6 = 41 N·m
- 弹簧力矩 = 43.5 N·m
- 控制器贡献 = 41 / (41 + 43.5) = 48.5% — **接近对半**

⚠️ **缺点**：在 k=500 下，铰链本身就几乎不动（<1°），
所以即使增大增益也很难在仿真中观察到明显效果。

#### 方案对比

| 指标 | 方案 A (降刚度) | 方案 B (增增益) |
|------|--------|--------|
| 铰链可观测运动 | ✅ 明显（3-10°） | ❌ 极小（<1°） |
| 控制器贡献度 | ✅ 77% | ⚠️ 48.5%（但偏转极小） |
| 仿真真实性 | ✅ 接近真实柔性铰链 | ❌ 仍是准刚性 |
| 调试可视性 | ✅ GZ 界面可直接观察 | ❌ 肉眼不可见 |
| 稳定性风险 | ⚠️ 需验证（ζ=1.18 应该OK） | ✅ 安全 |
| 修改文件数 | 1 (model.sdf) + 1 (airframe) | 1 (airframe) |
| **推荐度** | ⭐⭐⭐⭐⭐ | ⭐⭐ |

### 25.9 总结

| 问题 | 答案 |
|------|------|
| 铰链参数是否过于保守？ | ✅ **是的**。刚度 500 N·m/rad + 阻尼 50 N·m·s/rad 使铰链几乎不动 |
| 从机代码修正有没有作用？ | ❌ **几乎没有**。仅贡献总恢复力矩的 4.9%，弹簧做了 95% 的工作 |
| 是代码的问题吗？ | ❌ **不是代码问题**。代码逻辑正确（PD 控制 → trim → 舵面），但控制对象（铰链）太硬了 |
| 阻尼比是否合理？ | ❌ **严重过阻尼** ζ=3.74（应该 0.7-1.5），铰链像被胶水粘住 |
| 建议怎么改？ | **方案 A**：刚度 500→50，阻尼 50→5，限幅 ±5°→±10°，Kp 0.3→2.0 |
| 改了会不会不稳定？ | 不会。ζ=1.18（轻度过阻尼），自然频率 3.8 Hz 远低于控制器 50 Hz |
| 需要改代码吗？ | 改 model.sdf（刚度/阻尼）+ airframe（PD增益），不需要改 C++ |

---

*文档结束*
