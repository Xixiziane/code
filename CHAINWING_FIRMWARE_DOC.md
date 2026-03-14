# 链翼无人机 (Chain-Wing UAV) 飞控固件技术文档

> **项目名称**：Chain-Wing UAV — 基于 PX4 的三体串联固定翼飞控系统  
> **固件基线**：PX4 Autopilot v1.15+  
> **仿真环境**：Gazebo Harmonic + QGroundControl  
> **文档版本**：2.0（2026-03-14）

---

## 目录

1. [项目概述](#1-项目概述)
2. [系统架构](#2-系统架构)
3. [飞行器构型与物理参数](#3-飞行器构型与物理参数)
4. [控制系统设计](#4-控制系统设计)
5. [Gazebo 仿真模型详解](#5-gazebo-仿真模型详解)
6. [PX4 固件修改清单](#6-px4-固件修改清单)
7. [参数配置手册](#7-参数配置手册)
8. [MATLAB/Simulink 开环验证](#8-matlabsimulink-开环验证)
9. [飞行模式说明](#9-飞行模式说明)
10. [问题诊断与解决历程](#10-问题诊断与解决历程)
11. [使用指南](#11-使用指南)
12. [后续开发方向](#12-后续开发方向)

**附录**

- [A. 文件清单](#附录-a-文件清单)
- [B. 参数速查表](#附录-b-参数速查表)

---

## 1. 项目概述

### 1.1 设计目标

开发一种**三体串联固定翼无人机 (Chain-Wing UAV)** 的飞控固件。该飞行器由三个相同的固定翼单元在翼尖处连接而成，形成一个 3.6 米翼展的大展弦比飞翼。

### 1.2 技术创新

本项目在标准 PX4 固定翼飞控基础上实现了三个核心创新：

| # | 创新点 | 技术实现 | 目的 |
|---|--------|----------|------|
| 1 | **反向升降副翼配置** | 左右升降副翼的横滚效能符号反转 | 三体结构的横滚稳定性 |
| 2 | **差动推力偏航控制** | 三台电机差速产生偏航力矩 | 无方向舵的偏航控制 |
| 3 | **航向保持控制器** | 基于航向误差的偏航速率修正 | 增稳模式下的航向稳定 |

### 1.3 项目状态

- ✅ MATLAB/Simulink 开环动力学验证
- ✅ Gazebo 仿真模型搭建（968行 SDF）
- ✅ PX4 固件修改与参数配置
- ✅ 自主盘旋飞行验证
- ✅ RTL（返航）功能验证
- 🔄 增稳模式调试中（地面打转问题排查中）

---

## 2. 系统架构

### 2.1 总体架构图

```
┌─────────────────────────────────────────────────────────────────┐
│                        PX4 Autopilot                            │
│                                                                 │
│  ┌──────────────┐    ┌──────────────────┐    ┌───────────────┐ │
│  │  Navigator    │───▶│ FW Attitude Ctrl │───▶│ Control       │ │
│  │  (任务/导航)  │    │ (姿态控制)       │    │ Allocator     │ │
│  └──────────────┘    │                  │    │ (控制分配)    │ │
│         │            │  ┌────────────┐  │    │               │ │
│         ▼            │  │ Yaw Ctrl   │  │    │  Motor 0 ◄──┐│ │
│  ┌──────────────┐    │  │ +Heading   │  │    │  Motor 1 ◄──┤│ │
│  │  Commander   │    │  │  Hold      │  │    │  Motor 2 ◄──┤│ │
│  │  (指挥官)    │    │  └────────────┘  │    │  Servo 0 ◄──┤│ │
│  └──────────────┘    └──────────────────┘    │  Servo 1 ◄──┤│ │
│                                              │  Servo 2 ◄──┘│ │
│                                              └───────────────┘ │
└─────────────────────────────────┬───────────────────────────────┘
                                  │ MAVLink / uORB
                                  ▼
┌─────────────────────────────────────────────────────────────────┐
│                     Gazebo Harmonic                              │
│                                                                 │
│  ┌──────────┐  ┌───────────┐  ┌──────────────┐  ┌───────────┐ │
│  │ GZ Bridge│  │ Physics   │  │ LiftDrag ×8  │  │ Propeller │ │
│  │ (桥接器) │  │ Engine    │  │ (气动力插件)  │  │ ×3        │ │
│  └──────────┘  └───────────┘  └──────────────┘  └───────────┘ │
└─────────────────────────────────────────────────────────────────┘
```

### 2.2 数据流

```
传感器数据 ──▶ EKF2 ──▶ 姿态估计 ──▶ 姿态控制器 ──▶ 速率控制器 ──▶ 控制分配 ──▶ 电机/舵面
  (IMU,GPS,            (扩展卡尔曼     (ECL_*_Controller)  (FW Rate    (CA)        (GZ Bridge)
   Baro,Mag,            滤波器)                            Control)
   Airspeed)
```

---

## 3. 飞行器构型与物理参数

### 3.1 几何参数

```
           ◄──────────── 3.60 m 翼展 ───────────────▶
           ┌─────────────────────────────────────────┐
           │    左单元     │   中心单元    │   右单元   │
           │   1.20 m     │   1.20 m     │  1.20 m   │
           │  ┌───────┐   │  ┌───────┐   │ ┌───────┐ │
      ◄────┤  │ Motor │   │  │ Motor │   │ │ Motor │ ├────▶
           │  │  #0   │   │  │  #1   │   │ │  #2   │ │
           │  └───────┘   │  └───────┘   │ └───────┘ │
           │              │              │            │
           │  ┌───────┐   │  ┌───────┐   │ ┌───────┐ │
           │  │Elevon │   │  │Elevat.│   │ │Elevon │ │
           │  │ CS0   │   │  │ CS1   │   │ │ CS2   │ │
           │  └───────┘   │  └───────┘   │ └───────┘ │
           └──────▲───────┴──────────────┴────▲──────┘
                  │         翼尖连接           │
            Y=-1.2m       Y=0.0m         Y=+1.2m
```

### 3.2 质量与惯量参数

| 参数 | 值 | 来源 |
|------|-----|------|
| 总质量 | 5.70 kg | MATLAB 模型 |
| 翼展 | 3.60 m (3 × 1.20 m) | 几何设计 |
| 弦长 | 0.30 m | 几何设计 |
| 翼面积 | 1.08 m² (3 × 0.36 m²) | 计算 |
| Ixx (横滚) | 5.740 kg·m² | MATLAB 计算 |
| Iyy (俯仰) | 0.432 kg·m² | MATLAB 计算 |
| Izz (偏航) | 5.958 kg·m² | MATLAB 计算 |
| 展弦比 | 12.0 | b²/S = 3.6²/1.08 |

> **注意**：Ixx 和 Izz 很大（≈6 kg·m²）是因为质量分布在 3.6m 翼展上。这使得横滚和偏航响应较慢。

### 3.3 气动参数

| 参数 | 符号 | 值 | 说明 |
|------|------|-----|------|
| 零升迎角 | α₀ | -0.05 rad | MATLAB 标定 |
| 升力线斜率 | CLα | 5.25 /rad | 有限翼修正 |
| 阻力系数 | CDA | 0.65 | 含诱导阻力 |
| 失速迎角 | α_stall | 0.227 rad (13°) | MATLAB 标定 |
| 零升阻力 | Cd0 | 0.01 | 估算 |
| 诱导阻力因子 | K_ind | 0.0313 | 1/(π·e·AR) |

### 3.4 动力系统参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 电机数量 | 3 | 左/中/右各 1 台 |
| 单台最大推力 | 15 N | motorConstant = 1.5e-05 |
| 总最大推力 | 45 N | 3 × 15 N |
| 巡航油门 | 60% | FW_THR_TRIM = 0.60 |
| 电机间距 | 1.2 m | 差动推力力臂 |
| Motor 0 旋向 | CCW | 左单元 |
| Motor 1 旋向 | CW | 中心单元（反扭矩平衡） |
| Motor 2 旋向 | CCW | 右单元 |
| momentConstant | 0.001 | 螺旋桨反扭矩系数 |

### 3.5 飞行性能参数

| 参数 | 值 |
|------|-----|
| 最小空速 | 15 m/s |
| 巡航空速 | 20 m/s |
| 最大空速 | 30 m/s |
| 失速速度 | 8 m/s |
| 最大爬升率 | 8 m/s |
| 最大下降率 | 3.0 m/s |
| 最小下降率 | 2.5 m/s |

---

## 4. 控制系统设计

### 4.1 核心控制难题

链翼无人机与标准固定翼的关键区别：

| 特性 | 标准固定翼 | 链翼无人机 |
|------|-----------|-----------|
| 偏航控制 | 方向舵 | **差动推力**（无方向舵） |
| 横滚控制 | 副翼同向 | **副翼反向**（翼尖连接） |
| 偏航权威性 | 随空速变化 | **与空速无关**（电机推力） |
| 惯量 | 正常 | **横滚/偏航惯量大**（6 kg·m²） |

### 4.2 创新一：反向升降副翼配置

#### 4.2.1 原理

在三体串联结构中，翼尖处的连接改变了升降副翼的横滚力矩方向：

```
标准副翼：左副翼上偏 → 左翼升力↓ → 左翼下沉 → 向左滚转
链翼副翼：左副翼上偏 → 左单元升力↓ → 但通过翼尖连接 → 整体向右滚转（反向！）
```

#### 4.2.2 实现

在机架配置文件 `4007_gz_chainwing` 中设置反向横滚效能：

```bash
# CS0: 左单元升降副翼 — 横滚效能为 +0.5（反向）
param set-default CA_SV_CS0_TRQ_R 0.5

# CS2: 右单元升降副翼 — 横滚效能为 -0.5（反向）  
param set-default CA_SV_CS2_TRQ_R -0.5
```

对比标准 PX4 rc_cessna 的副翼配置：
- rc_cessna 左副翼：`TRQ_R = -0.5`（标准方向）
- rc_cessna 右副翼：`TRQ_R = +0.5`（标准方向）
- **链翼**：符号与 rc_cessna **相同**，但物理效果相反（因为 GZ model.sdf 中 `control_joint_rad_to_cl = -0.3`）

> **注意**：`CA_SV_CS*_TYPE` 的值（左升降副翼=5、右升降副翼=6）在设置了自定义 TRQ_R/TRQ_P/TRQ_Y 值后**不影响**效能符号。自定义值直接覆盖类型默认值。（参见 `ActuatorEffectivenessControlSurfaces.cpp:157`）

### 4.3 创新二：差动推力偏航控制

#### 4.3.1 原理

```
正偏航力矩（向右转）：
  Motor 0 (左, Y=-1.2m)：增加推力 → 产生正偏航力矩
  Motor 2 (右, Y=+1.2m)：减少推力 → 产生正偏航力矩

偏航力矩 = T_left × 1.2 - T_right × 1.2
```

#### 4.3.2 控制分配

PX4 的 `ActuatorEffectivenessFixedWing` 自动计算差动推力效能：

```
Motor 0: yaw_effectiveness = +1.2 × ct  (Y=-1.2, 轴向 +X)
Motor 1: yaw_effectiveness =  0.0       (Y= 0.0, 无偏航贡献)
Motor 2: yaw_effectiveness = -1.2 × ct  (Y=+1.2, 轴向 +X)
```

其中 `ct` 是推力系数。电机的螺旋桨反扭矩 (`enablePropellerTorque`) 已禁用，避免额外偏航耦合。

#### 4.3.3 与方向舵的区别

| 特性 | 方向舵 | 差动推力 |
|------|--------|----------|
| 低速效率 | 低（依赖空速） | **高**（与空速无关） |
| 高速效率 | 高 | 正常 |
| 地面控制 | 无效 | **有效**（这是优势也是问题） |
| 响应延迟 | 快（舵面） | 稍慢（电机转速变化） |

### 4.4 创新三：航向保持控制器

#### 4.4.1 动机

标准 PX4 固定翼在增稳模式下不主动控制偏航——它仅使用协调转弯公式：

```
yaw_rate = tan(roll) × cos(pitch) × g / airspeed
```

这对有方向舵的飞机足够了（方向舵效率随空速变化，自然阻尼了低速偏航）。但链翼的差动推力在低速时仍有完整权威性，导致地面/低速时的偏航不稳定。

#### 4.4.2 实现

在 `ecl_yaw_controller.cpp` 中添加航向保持逻辑：

```cpp
// 当 FW_YAW_STAB_SC > 0 时激活航向保持
if (_heading_hold_gain > FLT_EPSILON &&
    PX4_ISFINITE(ctl_data.yaw_setpoint) && PX4_ISFINITE(ctl_data.yaw)) {
    
    // 计算航向误差
    const float heading_error = wrap_pi(ctl_data.yaw_setpoint - ctl_data.yaw);

    // 空速依赖增益缩放：低速时降低增益，防止振荡
    const float airspeed_ratio = constrain(airspeed / trim_airspeed, 0.1, 1.0);
    const float scaled_gain = heading_hold_gain × airspeed_ratio²;

    // 添加航向修正到偏航速率设定值
    _body_rate_setpoint += constrain(heading_error × scaled_gain, -max_rate, max_rate);
}
```

#### 4.4.3 增稳模式航向锁定

在 `FixedwingAttitudeControl.cpp` 中实现航向设定值管理：

```cpp
// 进入增稳模式时锁定当前航向
if (!_heading_setpoint_initialized) {
    _heading_setpoint = yaw_body;  // 锁定当前航向
    _heading_setpoint_initialized = true;
}

// 偏航摇杆更新航向设定值（增量式）
if (fabsf(yaw_stick) > 0.05f) {
    _heading_setpoint += yaw_stick × FW_Y_RMAX × dt;
}
```

#### 4.4.4 参数

| 参数 | 当前值 | 范围 | 说明 |
|------|--------|------|------|
| FW_YAW_STAB_SC | 1.0 | 0.0~2.0 | 航向保持增益（0=禁用） |

#### 4.4.5 控制框图

```
              ┌──────────────────┐
 heading_sp ──┤                  │
              │  Heading Hold    ├──▶ yaw_rate_correction
 heading    ──┤  (比例控制器)    │        │
              │  gain×(V/V₀)²   │        │
              └──────────────────┘        │
                                          ▼
 roll ──────▶ Coord Turn ──▶ yaw_rate_base + yaw_rate_correction ──▶ body_rate_sp
              tan(φ)g/V                                                   │
                                                                          ▼
                                                              Rate Controller
                                                              (FW_YR_P/I/FF)
                                                                          │
                                                                          ▼
                                                              Control Allocator
                                                              → 差动推力
```

---

## 5. Gazebo 仿真模型详解

### 5.1 模型文件

- **飞行器模型路径**：`Tools/simulation/gz/models/chainwing/model.sdf`（968行）
- **世界文件路径**：`Tools/simulation/gz/worlds/flat_terrain.sdf`
- **格式**：SDF 1.9（Gazebo Harmonic 格式）

### 5.1b 超大平地仿真世界（flat_terrain）

为解决固定翼起降测试中地面范围不足的问题，创建了 `flat_terrain.sdf` 世界文件：

| 特性 | 默认世界 (default.sdf) | 超大平地 (flat_terrain.sdf) |
|------|----------------------|---------------------------|
| 地面尺寸（碰撞） | 1×1 m | **2000×2000 m** |
| 地面尺寸（视觉） | 100×100 m | **2000×2000 m** |
| 地面摩擦力 | 默认 | mu=100, mu2=50 |
| 跑道标记 | 无 | **200m×2m 深色中心线** |
| 物理步长 | 0.004s (250Hz) | 0.004s (250Hz) |

机架配置中使用 `PX4_GZ_WORLD=flat_terrain` 自动加载该世界。

> **注意**：Gazebo 的 `<plane>` 几何体在碰撞检测中实际表现为无限平面，但视觉渲染尺寸决定了可见范围。将视觉尺寸设为 2000m 确保飞机在盘旋/着陆时始终能看到地面。

### 5.2 模型结构

```
chainwing/
├── model.config          # 模型元数据
└── model.sdf             # 模型定义（968行）
    ├── base_link          # 主机体（含所有视觉和惯量）
    │   ├── 机身 ×3        # 左/中/右机身
    │   ├── 机翼 ×3        # 左/中/右翼段
    │   ├── 垂尾 ×3        # 左/中/右垂直尾翼
    │   ├── 连接标记 ×2    # 翼尖连接可视化
    │   └── 碰撞体         # 简化碰撞几何
    ├── 升降副翼 ×3        # 可活动舵面（revolute joints）
    ├── LiftDrag ×8        # 气动力插件
    │   ├── 翼段 ×3        # 左/中/右（含舵面控制）
    │   ├── 中心翼 ×1      # 中心升降舵控制
    │   ├── 垂尾 ×3        # 被动偏航阻尼
    │   └── 右翼 ×1        # 右翼段+右升降副翼
    ├── JointController ×3 # 舵面位置控制器
    └── Propeller ×3       # 螺旋桨推力模型
```

### 5.3 LiftDrag 插件配置

#### 5.3.1 主翼段（3个，带舵面控制）

| 参数 | 值 | 说明 |
|------|-----|------|
| a0 | -0.05 rad | 零升迎角 |
| cla | 5.25 /rad | 升力线斜率 |
| cda | 0.65 | 阻力系数 |
| alpha_stall | 0.227 rad | 失速迎角 |
| area | 0.36 m² | 翼段面积 |
| control_joint_rad_to_cl | -0.3 | **舵面偏转 → 升力变化**（负号：正偏转减小升力） |

#### 5.3.2 中心升降舵

| 参数 | 值 | 说明 |
|------|-----|------|
| control_joint_rad_to_cl | -4.0 | 升降舵效能（比副翼大 13 倍） |
| 其他参数 | 同主翼段 | — |

#### 5.3.3 垂直尾翼（3个，被动阻尼）

| 参数 | 值 | 说明 |
|------|-----|------|
| upward | 0, 1, 0 | **Y轴朝上**→ 侧向气动力 |
| area | 0.02 m² | 垂尾面积（小） |
| cla | 4.75 /rad | 侧力斜率 |
| control_joint | 无 | 被动（无方向舵） |

### 5.4 螺旋桨插件

| Motor | 位置 | 旋向 | 说明 |
|-------|------|------|------|
| Motor 0 | Y = -1.2m (左) | CCW | maxRotVelocity = 1000 |
| Motor 1 | Y = 0.0m (中) | CW | 反向旋转（扭矩平衡） |
| Motor 2 | Y = +1.2m (右) | CCW | 与 Motor 0 相同 |

所有电机参数：`motorConstant = 1.5e-05`，`momentConstant = 0.001`

### 5.5 舵面关节

| 关节 | 控制面 | 轴向 | 范围 | 阻尼 |
|------|--------|------|------|------|
| servo_0 | 左升降副翼 | Y (0,1,0) | ±0.53 rad (±30°) | 1.0 |
| servo_1 | 中心升降舵 | Y (0,1,0) | ±0.53 rad (±30°) | 1.0 |
| servo_2 | 右升降副翼 | Y (0,1,0) | ±0.53 rad (±30°) | 1.0 |

---

## 6. PX4 固件修改清单

### 6.1 修改文件总览

| 文件 | 修改类型 | 行数 | 说明 |
|------|----------|------|------|
| `ecl_yaw_controller.cpp` | **核心修改** | +24行 | 航向保持控制器 |
| `ecl_yaw_controller.h` | **核心修改** | +5行 | 新增成员变量和接口 |
| `FixedwingAttitudeControl.cpp` | **核心修改** | +20行 | 航向设定值管理 |
| `FixedwingAttitudeControl.hpp` | **核心修改** | +2行 | 新增成员变量 |
| `4007_gz_chainwing` | **新建文件** | 219行 | 机架配置 |
| `model.sdf` | **新建文件** | 968行 | Gazebo 仿真模型 |
| `model.config` | **新建文件** | 12行 | 模型元数据 |

### 6.2 ecl_yaw_controller.h 修改详解

```cpp
// 新增公开接口
void set_heading_hold_gain(float gain) { _heading_hold_gain = gain; }
void set_trim_airspeed(float airspeed) { _trim_airspeed = airspeed; }

// 新增私有成员
float _heading_hold_gain{0.f};   // 航向保持增益（由 FW_YAW_STAB_SC 设置）
float _trim_airspeed{15.f};       // 巡航空速（用于增益缩放）
```

### 6.3 ecl_yaw_controller.cpp 修改详解

在 `control_attitude()` 函数末尾（原始代码 return 之前）添加航向保持逻辑：

```cpp
/* Heading hold for chain-wing yaw stabilization:
 * Add a yaw rate correction proportional to heading error.
 * Scale gain by (airspeed / trim_airspeed)^2 to prevent over-correction
 * at low speeds (landing/approach) where differential thrust authority
 * and vertical tail effectiveness are reduced.
 */
if (_heading_hold_gain > FLT_EPSILON &&
    PX4_ISFINITE(ctl_data.yaw_setpoint) && PX4_ISFINITE(ctl_data.yaw)) {
    const float heading_error = wrap_pi(ctl_data.yaw_setpoint - ctl_data.yaw);

    const float airspeed_ratio = math::constrain(
        ctl_data.airspeed_constrained / math::max(_trim_airspeed, 1.f),
        0.1f, 1.0f);
    const float scaled_gain = _heading_hold_gain * airspeed_ratio * airspeed_ratio;

    const float heading_rate_correction = heading_error * scaled_gain;
    _body_rate_setpoint += math::constrain(heading_rate_correction, -_max_rate, _max_rate);
    _body_rate_setpoint = math::constrain(_body_rate_setpoint, -_max_rate, _max_rate);
}
```

**关键设计决策**：
- `airspeed_ratio` 范围 [0.1, 1.0]：最小增益为 0.01×gain（不是零，保留基础修正）
- 使用 `airspeed_ratio²`：二次衰减，巡航时全增益，低速时快速降低
- 最终偏航速率受 `_max_rate`（FW_Y_RMAX）双重限制

### 6.4 FixedwingAttitudeControl.cpp 修改详解

#### 6.4.1 参数传递（`parameters_update` 函数）

```cpp
_yaw_ctrl.set_heading_hold_gain(_param_fw_yaw_stab_sc.get());
_yaw_ctrl.set_trim_airspeed(_param_fw_airspd_trim.get());
```

#### 6.4.2 增稳模式航向锁定（`vehicle_manual_poll` 函数）

```cpp
if (_param_fw_yaw_stab_sc.get() > FLT_EPSILON) {
    // 链翼航向保持：锁定进入增稳模式时的航向
    if (!_heading_setpoint_initialized) {
        _heading_setpoint = yaw_body;
        _heading_setpoint_initialized = true;
    }
    _att_sp.yaw_body = _heading_setpoint;
} else {
    _att_sp.yaw_body = yaw_body;  // 标准PX4行为
}
```

#### 6.4.3 偏航摇杆航向更新（主控制循环）

```cpp
if (_param_fw_yaw_stab_sc.get() > FLT_EPSILON) {
    // 链翼模式：偏航摇杆增量更新航向设定值
    const float yaw_stick = _manual_control_setpoint.yaw;
    if (fabsf(yaw_stick) > 0.05f) {
        _heading_setpoint += yaw_stick * radians(_param_fw_y_rmax.get()) * dt;
        _heading_setpoint = wrap_pi(_heading_setpoint);
    }
}
```

#### 6.4.4 航向设定值重置（切出增稳模式时）

```cpp
_heading_setpoint_initialized = false;  // 切模式时重置
```

### 6.5 FixedwingAttitudeControl.hpp 修改详解

```cpp
// 新增成员变量
float _heading_setpoint{0.f};
bool _heading_setpoint_initialized{false};
```

### 6.6 向后兼容性

所有修改通过 `FW_YAW_STAB_SC` 参数控制：

| FW_YAW_STAB_SC | 行为 |
|-----------------|------|
| 0.0（默认） | 完全等同于标准 PX4（无任何链翼增强） |
| > 0.0 | 启用链翼航向保持控制器 |

标准 PX4 固定翼用户不受任何影响。

---

## 7. 参数配置手册

### 7.1 控制分配参数

#### 7.1.1 电机配置

| 参数 | 值 | 说明 |
|------|-----|------|
| CA_AIRFRAME | 1 | 固定翼机型 |
| CA_ROTOR_COUNT | 3 | 三台电机 |
| CA_ROTOR0_PY | -1.2 | 左电机 Y 坐标 |
| CA_ROTOR1_PY | 0.0 | 中心电机 Y 坐标 |
| CA_ROTOR2_PY | 1.2 | 右电机 Y 坐标 |

#### 7.1.2 舵面配置

| 参数 | 值 | 说明 |
|------|-----|------|
| CA_SV_CS_COUNT | 3 | 三个舵面 |
| CA_SV_CS0_TYPE | 6 | 右升降副翼（标签） |
| CA_SV_CS0_TRQ_R | 0.5 | 横滚效能（反向） |
| CA_SV_CS0_TRQ_P | 0.5 | 俯仰效能 |
| CA_SV_CS1_TYPE | 3 | 升降舵 |
| CA_SV_CS1_TRQ_P | 1.0 | 纯俯仰效能 |
| CA_SV_CS2_TYPE | 5 | 左升降副翼（标签） |
| CA_SV_CS2_TRQ_R | -0.5 | 横滚效能（反向） |
| CA_SV_CS2_TRQ_P | 0.5 | 俯仰效能 |

### 7.2 空速参数

| 参数 | 值 | 单位 | 说明 |
|------|-----|------|------|
| FW_AIRSPD_MIN | 15 | m/s | 最小空速 |
| FW_AIRSPD_TRIM | 20 | m/s | 巡航空速 |
| FW_AIRSPD_MAX | 30 | m/s | 最大空速 |
| FW_AIRSPD_STALL | 8 | m/s | 失速速度 |

### 7.3 姿态控制参数

| 参数 | 值 | 说明 |
|------|-----|------|
| FW_R_TC | 0.5 | 横滚时间常数 |
| FW_P_TC | 0.5 | 俯仰时间常数 |
| FW_R_RMAX | 30.0 | 最大横滚速率 (°/s) |
| FW_P_RMAX_POS | 25.0 | 最大抬头速率 (°/s) |
| FW_P_RMAX_NEG | 25.0 | 最大低头速率 (°/s) |
| FW_Y_RMAX | 15.0 | 最大偏航速率 (°/s) |
| FW_R_LIM | 35 | 最大横滚角 (°) |
| FW_P_LIM_MAX | 15 | 最大俯仰角 (°) |
| FW_P_LIM_MIN | -15 | 最小俯仰角 (°) |
| FW_PSP_OFF | 2.0 | 俯仰角偏移 (°) |

### 7.4 速率控制器参数

| 参数 | 值 | 说明 |
|------|-----|------|
| FW_PR_P | 0.9 | 俯仰速率 P 增益 |
| FW_PR_FF | 0.5 | 俯仰速率前馈 |
| FW_PR_I | 0.5 | 俯仰速率积分 |
| FW_RR_P | 0.3 | 横滚速率 P 增益 |
| FW_RR_FF | 0.5 | 横滚速率前馈 |
| FW_RR_I | 0.5 | 横滚速率积分 |
| FW_YR_P | 0.6 | 偏航速率 P 增益 |
| FW_YR_FF | 0.5 | 偏航速率前馈 |
| FW_YR_I | 0.5 | 偏航速率积分 |

### 7.5 油门参数

| 参数 | 值 | 说明 |
|------|-----|------|
| FW_THR_MAX | 1.0 | 最大油门 |
| FW_THR_MIN | 0.05 | 最小油门 |
| FW_THR_TRIM | 0.60 | 巡航油门 |
| TRIM_PITCH | -0.15 | 俯仰舵面配平 |

### 7.6 导航参数

| 参数 | 值 | 说明 |
|------|-----|------|
| MIS_TAKEOFF_ALT | 15 | 起飞目标高度 (m) |
| NAV_ACC_RAD | 20 | 航点接受半径 (m) |
| NAV_LOITER_RAD | 50 | 盘旋半径 (m) |
| NPFG_PERIOD | 12 | NPFG 导航周期 (s) |
| NAV_DLL_ACT | 0 | 数据链丢失动作（0=无） |

### 7.7 RTL 参数

| 参数 | 值 | 说明 |
|------|-----|------|
| RTL_RETURN_ALT | 30 | 返航高度 (m) |
| RTL_DESCEND_ALT | 15 | 下降高度 (m) |
| RTL_LAND_DELAY | 0 | 降落延迟 (s)（0=立即降落） |

### 7.8 降落参数

| 参数 | 值 | 说明 |
|------|-----|------|
| FW_LND_ANG | 8 | 进场角度 (°) |
| FW_LND_AIRSPD | 16 | 进场空速 (m/s) |
| FW_LND_FL_PMIN | 3 | 拉平最小俯仰 (°) |
| FW_LND_FL_PMAX | 10 | 拉平最大俯仰 (°) |
| FW_LND_FLALT | 5 | 拉平高度 (m) |
| **FW_LND_USETER** | **0** | **禁用地形估计**（无测距仪，使用航点高度着陆） |
| **FW_LND_ABORT** | **0** | **禁用着陆中止**（无地形传感器时必须禁用） |

> ⚠️ **重要**：`FW_LND_USETER=0` 是防止 "No terrain measurement result" 着陆中止错误的关键参数。详见[第10.1节 问题#11](#101-已解决的问题)。

### 7.9 SITL 仿真参数

| 参数 | 值 | 说明 |
|------|-----|------|
| SIM_GZ_EN | 1 | 启用 Gazebo 仿真 |
| SENS_EN_GPSSIM | 1 | 启用 GPS 仿真 |
| SENS_EN_MAGSIM | 1 | 启用磁力计仿真 |
| SENS_EN_ARSPDSIM | 1 | 启用空速仿真 |
| SIM_BAT_DRAIN | 3600 | 电池耗尽时间 (s) |
| COM_LOW_BAT_ACT | 0 | 电池低电量动作（0=仅警告） |
| CBRK_SUPPLY_CHK | 894281 | 电源检查断路器 |
| COM_RC_IN_MODE | 3 | RC/摇杆输入模式 |
| COM_PREARM_MODE | 2 | 预解锁检查模式 |
| FD_ESCS_EN | 0 | ESC 故障检测（禁用） |
| CP_DIST | -1 | 碰撞预防（禁用） |
| SYS_DM_BACKEND | 1 | 数据管理后端（RAM） |
| RWTO_TKOFF | 1 | 滑跑起飞模式 |
| FW_W_EN | 1 | 前轮控制启用 |

### 7.10 EKF2 参数

| 参数 | 值 | 说明 |
|------|-----|------|
| EKF2_ACC_NOISE | 0.5 | 加速度计噪声 |
| EKF2_GYR_NOISE | 0.02 | 陀螺仪噪声 |
| EKF2_ACC_B_NOISE | 0.005 | 加速度计偏差噪声 |

---

## 8. MATLAB/Simulink 开环验证

### 8.1 验证流程

```
MATLAB 开环模型                    PX4 + Gazebo 闭环仿真
┌──────────────┐                  ┌──────────────────┐
│ 气动力系数    │─── 参数传递 ───▶│ model.sdf LiftDrag│
│ CLA, CDA,    │                  │ a0, cla, cda,    │
│ α_stall      │                  │ alpha_stall      │
├──────────────┤                  ├──────────────────┤
│ 惯量张量      │─── 参数传递 ───▶│ model.sdf inertia│
│ Ixx,Iyy,Izz │                  │ ixx, iyy, izz    │
├──────────────┤                  ├──────────────────┤
│ 电机推力      │─── 参数传递 ───▶│ Propeller plugin │
│ T_max=15N    │                  │ motorConstant    │
├──────────────┤                  ├──────────────────┤
│ 配平状态      │─── 参数传递 ───▶│ 4007_gz_chainwing│
│ V0=20, α=4.7°│                  │ FW_AIRSPD_TRIM   │
│ T_trim=60%   │                  │ FW_THR_TRIM      │
└──────────────┘                  └──────────────────┘
```

### 8.2 验证的参数对应关系

| MATLAB 参数 | 值 | 对应 GZ/PX4 参数 |
|------------|-----|-------------------|
| V₀ (巡航速度) | 20 m/s | FW_AIRSPD_TRIM = 20 |
| α_trim (配平迎角) | 4.7° | FW_PSP_OFF = 2.0（GZ配平更低） |
| T_trim (配平推力) | 60% | FW_THR_TRIM = 0.60 |
| b (单元间距) | 1.2 m | CA_ROTOR0_PY = -1.2 |
| T_max (最大推力) | 15 N | motorConstant = 1.5e-05 |
| S_wing (翼面积) | 0.36 m²/段 | LiftDrag area = 0.36 |
| CL_α (升力线斜率) | 5.25 /rad | LiftDrag cla = 5.25 |

---

## 9. 飞行模式说明

### 9.1 支持的飞行模式

本固件**同时支持手飞增稳和自主飞行**，共 15 种飞行模式：

#### 手动模式（2种）

| 模式 | 说明 | 链翼特殊行为 |
|------|------|-------------|
| Manual | 直接控制舵面 | 无增强 |
| Acro | 角速率控制 | 无增强 |

#### 增稳模式（3种）

| 模式 | 说明 | 链翼特殊行为 |
|------|------|-------------|
| **Stabilized** | 姿态稳定 | ✅ 航向保持（FW_YAW_STAB_SC） |
| Altitude | 高度保持 | ✅ 航向保持 |
| Position | 位置保持 | ✅ 航向保持 |

#### 自主模式（10种）

| 模式 | 说明 | 链翼特殊行为 |
|------|------|-------------|
| **Takeoff** | 自动起飞 | 差动推力保持方向 |
| **Land** | 自动降落 | 8° 进场角 |
| **Mission** | 任务飞行 | 航点导航 |
| **Loiter** | 盘旋 | 50m 盘旋半径 |
| **RTL** | 返航 | 30m→15m→降落 |
| Orbit | 绕圈 | 标准行为 |
| Hold | 悬停/盘旋 | 标准行为 |
| Offboard | 外部控制 | 标准行为 |
| Follow Target | 跟踪目标 | 标准行为 |
| VTOL Transition | VTOL 过渡 | 不适用 |

### 9.2 模式切换注意事项

- **自动 → 增稳**：航向在切换瞬间锁定。确保飞行稳定后再切换。
- **增稳模式**：需要 QGC 虚拟摇杆或物理遥控器发送手动控制数据。
- **COM_RC_IN_MODE = 3**：接受 RC 或摇杆（QGC 虚拟摇杆）输入。

---

## 10. 问题诊断与解决历程

### 10.1 已解决的问题

| # | 问题 | 原因 | 解决方案 | 提交 |
|---|------|------|---------|------|
| 1 | 解锁后立即锁定 | GZ bridge 延迟设置 esc_armed_flags | FD_ESCS_EN=0 | 3fdee64 |
| 2 | QGC 参数缺失警告 | CP_DIST 未设置 | CP_DIST=-1 | 6fcd133 |
| 3 | 起飞地面打转 | 垂尾缺失+电机扭矩不平衡 | 添加垂尾+CW/CCW平衡 | a17dec8 |
| 4 | EKF2 姿态警告 | 气动参数与 EKF 预期不一致 | 更新 EKF2 噪声参数 | 0b55fba |
| 5 | 无 QGC 时无法起飞 | NAV_DLL_ACT 触发故障保护 | NAV_DLL_ACT=0 | 3c6816b |
| 6 | 起飞高度过高/RTL 不降落 | MIS_TAKEOFF_ALT=30, RTL_LAND_DELAY=-1 | 降低高度,LAND_DELAY=0 | 20598bf |
| 7 | 降落时偏航振荡 | 低速时航向增益过高 | 空速依赖增益缩放 | b102fe5 |
| 8 | "Switching to STAB not available" | 无手动控制输入 | COM_RC_IN_MODE=3 | 51bbee0 |
| 9 | 电池60秒耗尽 | SIM_BAT_DRAIN 默认值 | SIM_BAT_DRAIN=3600 | 6d09703 |
| 10 | 电池故障保护中断飞行 | COM_LOW_BAT_ACT 触发 Hold/RTL | COM_LOW_BAT_ACT=0 | 本次提交 |
| 11 | **着陆中止：No terrain measurement** | 无测距仪+FW_LND_USETER=1+FW_LND_ABORT=3 | FW_LND_USETER=0, FW_LND_ABORT=0 | 本次提交 |

#### 问题 #11 详细分析：着陆中止 "No terrain measurement result"

**错误信息**：
```
WARN  [navigator] Landing aborted: terrain measurement not found
```

**触发条件**：
1. `FW_LND_USETER=1`（默认值，使用地形估计触发拉平）
2. `FW_LND_ABORT=3`（默认值，bit 0=1 → 地形未找到时中止）
3. 链翼模型**没有测距仪/激光雷达传感器**
4. GZ 桥接器**不订阅 lidar/LaserScan 话题**
5. PX4 SITL 中**不存在 sensor_distance_sim 模块**

**代码路径** (`FixedwingPositionControl.cpp:getLandingTerrainAltitudeEstimate()`):
```
FW_LND_USETER > 0 → 检查 dist_bottom_valid
  → false（无测距仪，EKF2 无 dist_bottom）
    → 从未有过有效测量
      → 超时（TERRAIN_ALT_FIRST_MEASUREMENT_TIMEOUT = 5s）
        → FW_LND_ABORT bit 0 = 1
          → updateLandingAbortStatus(TERRAIN_NOT_FOUND)
            → "Landing aborted: terrain measurement not found"
```

**解决方案**：
```bash
param set-default FW_LND_USETER 0   # 禁用地形估计，使用航点高度
param set-default FW_LND_ABORT  0   # 禁用所有着陆中止条件
```

**效果**：着陆改为使用航点海拔高度而非地形测量值，在平坦地形上完全可靠。

### 10.2 已知问题（调试中）

| # | 问题 | 分析 | 状态 |
|---|------|------|------|
| 1 | 增稳模式地面打转 | 协调转弯公式在低空速时产生大偏航速率指令；差动推力在地面有完整权威性 | 🔄 排查中 |
| 2 | "Arming denied: high throttle" | QGC虚拟摇杆弹回中心,油门>-0.8 | 🔄 需要 SITL 条件编译解决 |

---

## 11. 使用指南

### 11.1 环境要求

- Ubuntu 22.04
- PX4 Autopilot v1.15+
- Gazebo Harmonic (gz-sim)
- QGroundControl 4.x
- Python 3.10+

### 11.2 构建固件

```bash
cd ~/PX4-Autopilot

# 清理旧参数缓存（重要！修改参数后必须执行）
rm -f build/px4_sitl_default/rootfs/parameters*.bson

# 构建并启动仿真
make px4_sitl gz_chainwing
```

### 11.3 连接 QGroundControl

1. 启动 QGC
2. QGC 自动连接 PX4（UDP 端口 14550）
3. 等待 GPS 锁定（Home position set）

### 11.4 自主飞行（推荐的首次测试）

```bash
# PX4 Shell 中执行：
commander takeoff    # 自动起飞到 15m
# 观察飞机盘旋，确认稳定
commander rtl        # 返航降落
```

### 11.5 增稳模式飞行（需要手动控制输入）

1. **启用 QGC 虚拟摇杆**：
   - QGC → 应用设置 → Virtual Joystick → 启用
2. **切换模式**：
   - QGC 模式选择 → Stabilized
3. **使用虚拟摇杆**控制飞机

### 11.6 关键 PX4 Shell 命令

```bash
# 检查传感器状态
listener sensor_accel_fifo -n 1
listener sensor_gyro_fifo -n 1
listener airspeed -n 1

# 检查 EKF2 状态
ekf2 status

# 检查控制分配
control_allocator status

# 检查参数值
param show FW_YAW_STAB_SC
param show SIM_BAT_DRAIN

# 查看当前姿态
listener vehicle_attitude -n 1

# 查看控制输出
listener actuator_outputs -n 1
```

### 11.7 参数修改后的重启流程

```bash
# 1. 停止当前仿真 (Ctrl+C)
# 2. 清理参数缓存
rm -f build/px4_sitl_default/rootfs/parameters*.bson
# 3. 重新构建并启动
make px4_sitl gz_chainwing
```

---

## 12. 后续开发方向

### 12.1 短期目标

| 任务 | 优先级 | 说明 |
|------|--------|------|
| 解决增稳模式地面打转 | 高 | 需要在低空速时抑制协调转弯指令 |
| 解决高油门解锁拒绝 | 高 | SITL 条件编译绕过油门检查 |
| PID 自动调参 | 中 | 使用 PX4 内置 autotune |

### 12.2 中期目标

| 任务 | 说明 |
|------|------|
| 任务航线飞行 | 多航点自主飞行测试 |
| 风扰动测试 | Gazebo 风场插件验证抗风能力 |
| 硬件在环 (HIL) | 连接实际飞控硬件 |

### 12.3 长期目标

| 任务 | 说明 |
|------|------|
| 实飞测试 | 实际飞行验证 |
| 编队飞行 | 多架链翼协同 |
| 自适应控制 | 在线参数辨识 |

---

## 附录 A. 文件清单

### A.1 新建文件

| 文件路径 | 行数 | 说明 |
|----------|------|------|
| `ROMFS/px4fmu_common/init.d-posix/airframes/4007_gz_chainwing` | 241 | 机架配置 |
| `Tools/simulation/gz/models/chainwing/model.sdf` | 968 | Gazebo 仿真模型 |
| `Tools/simulation/gz/models/chainwing/model.config` | 12 | 模型元数据 |
| `Tools/simulation/gz/worlds/flat_terrain.sdf` | 177 | 超大平地仿真世界 (2000×2000m) |
| `CHAINWING_FIRMWARE_DOC.md` | 本文件 | 固件技术文档 |
| `CHAINWING_CONTROL_FLOW.md` | ~1600 | 控制流程详解 |
| `CHAINWING_TUNING_GUIDE.md` | ~800 | 调参指南 |

### A.2 修改文件

| 文件路径 | 修改量 | 说明 |
|----------|--------|------|
| `src/modules/fw_att_control/ecl_yaw_controller.cpp` | +24行 | 航向保持控制器 |
| `src/modules/fw_att_control/ecl_yaw_controller.h` | +5行 | 新接口和成员变量 |
| `src/modules/fw_att_control/FixedwingAttitudeControl.cpp` | +20行 | 航向设定值管理 |
| `src/modules/fw_att_control/FixedwingAttitudeControl.hpp` | +2行 | 新成员变量 |

---

## 附录 B. 参数速查表

### B.1 链翼专用参数

| 参数 | 值 | 类别 | 说明 |
|------|-----|------|------|
| FW_YAW_STAB_SC | 1.0 | 控制 | 航向保持增益（0=禁用） |
| CA_SV_CS0_TRQ_R | 0.5 | 分配 | 左升降副翼横滚效能（反向） |
| CA_SV_CS2_TRQ_R | -0.5 | 分配 | 右升降副翼横滚效能（反向） |
| CA_ROTOR0_PY | -1.2 | 分配 | 左电机位置 |
| CA_ROTOR2_PY | 1.2 | 分配 | 右电机位置 |

### B.2 SITL 仿真专用参数

| 参数 | 值 | 说明 |
|------|-----|------|
| SIM_BAT_DRAIN | 3600 | 电池 1 小时耗尽 |
| COM_LOW_BAT_ACT | 0 | 低电量仅警告 |
| CBRK_SUPPLY_CHK | 894281 | 绕过电源检查 |
| FD_ESCS_EN | 0 | 禁用 ESC 故障检测 |
| COM_RC_IN_MODE | 3 | 接受 RC 或摇杆 |
| NAV_DLL_ACT | 0 | 数据链丢失无动作 |
| SYS_DM_BACKEND | 1 | RAM 模式 |
| CP_DIST | -1 | 禁用碰撞预防 |
| **FW_LND_USETER** | **0** | **禁用地形估计（无测距仪）** |
| **FW_LND_ABORT** | **0** | **禁用着陆中止条件** |

### B.3 飞行性能参数

| 参数 | 值 | 说明 |
|------|-----|------|
| FW_AIRSPD_TRIM | 20 | 巡航空速 (m/s) |
| FW_THR_TRIM | 0.60 | 巡航油门 |
| MIS_TAKEOFF_ALT | 15 | 起飞高度 (m) |
| NAV_LOITER_RAD | 50 | 盘旋半径 (m) |
| RTL_RETURN_ALT | 30 | 返航高度 (m) |
| FW_LND_ANG | 8 | 进场角度 (°) |

---

> **文档维护**：如有参数修改，请同步更新本文档。使用 `param show <PARAM>` 验证实际运行值。
