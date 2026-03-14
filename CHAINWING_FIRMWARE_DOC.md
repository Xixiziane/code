# 链翼无人机飞控固件技术文档

> **Chain-Wing UAV Flight Control Firmware — Technical Documentation**
>
> 版本: 1.0 | 日期: 2026-03-14 | 基于: PX4 Autopilot v1.15+

---

## 目录

- [1. 项目概述](#1-项目概述)
- [2. 系统架构](#2-系统架构)
- [3. 飞行器构型与物理参数](#3-飞行器构型与物理参数)
- [4. 控制系统设计（核心创新）](#4-控制系统设计核心创新)
- [5. Gazebo 仿真模型](#5-gazebo-仿真模型)
- [6. PX4 固件修改清单](#6-px4-固件修改清单)
- [7. 参数配置手册](#7-参数配置手册)
- [8. MATLAB/Simulink 开环验证](#8-matlabsimulink-开环验证)
- [9. 飞行模式说明](#9-飞行模式说明)
- [10. 问题诊断与解决历程](#10-问题诊断与解决历程)
- [11. 使用指南](#11-使用指南)
- [12. 后续开发方向](#12-后续开发方向)
- [附录 A: 文件清单](#附录-a-文件清单)
- [附录 B: 参数速查表](#附录-b-参数速查表)

---

## 1. 项目概述

### 1.1 项目背景

本项目开发了一套**链翼无人机（Chain-Wing UAV）**的飞控固件，基于开源飞控系统 PX4 Autopilot。链翼无人机是一种创新构型——由 **三架小型固定翼飞机在翼尖处铰接连接**，形成一个大展弦比飞行平台。

### 1.2 设计目标

| 目标 | 说明 |
|------|------|
| **稳定飞行** | 实现三单元链翼构型的稳定飞行控制 |
| **差动推力偏航** | 利用三个电机的推力差实现偏航控制（无方向舵） |
| **反向升降副翼** | 通过反向的升降副翼配置实现横滚稳定 |
| **航向保持** | 自研航向保持算法，补偿链翼固有的弱偏航稳定性 |
| **仿真验证** | 在 Gazebo 仿真环境中完整验证飞行性能 |
| **向后兼容** | 所有修改可通过参数开关禁用，不影响标准 PX4 功能 |

### 1.3 技术创新点

1. **航向保持控制器**：在 PX4 偏航控制器中新增基于航向误差的比例反馈，并引入空速依赖的增益缩放
2. **差动推力偏航控制**：三电机布局实现无方向舵的偏航控制
3. **反向升降副翼配置**：左单元使用右类型升降副翼、右单元使用左类型升降副翼，形成自恢复横滚力矩
4. **MATLAB 开环验证**：通过多体动力学仿真验证气动参数后移植到 Gazebo 模型

---

## 2. 系统架构

### 2.1 整体架构图

```
┌──────────────────────────────────────────────────────────┐
│                  QGC 地面站 (UDP 14550)                   │
│         手动控制 / 任务规划 / 参数调节 / 遥测              │
└─────────────────────────┬────────────────────────────────┘
                          │ MAVLink
┌─────────────────────────┴────────────────────────────────┐
│                     PX4 飞控固件                          │
│                                                          │
│  ┌──────────┐  ┌──────────────┐  ┌───────────────────┐  │
│  │ Commander │  │  Navigator    │  │ EKF2 状态估计     │  │
│  │ 模式管理  │  │  航线/RTL/   │  │  IMU+GPS+磁力计   │  │
│  │ 安全检查  │  │  Loiter管理  │  │  +气压+空速融合   │  │
│  └─────┬────┘  └──────┬───────┘  └─────────┬─────────┘  │
│        │              │                    │             │
│  ┌─────┴──────────────┴────────────────────┴──────────┐  │
│  │          FixedwingAttitudeControl (修改)             │  │
│  │                                                     │  │
│  │  ┌─────────────┐ ┌──────────────┐ ┌──────────────┐ │  │
│  │  │ Roll Ctrl   │ │ Pitch Ctrl   │ │ Yaw Ctrl ★   │ │  │
│  │  │ 标准PX4     │ │ 标准PX4      │ │ +航向保持    │ │  │
│  │  │ 升降副翼    │ │ 升降舵       │ │ +空速缩放    │ │  │
│  │  └──────┬──────┘ └──────┬───────┘ └──────┬────────┘ │  │
│  └─────────┼───────────────┼────────────────┼──────────┘  │
│            │               │                │             │
│  ┌─────────┴───────────────┴────────────────┴──────────┐  │
│  │              控制分配器 (Control Allocator)           │  │
│  │                                                     │  │
│  │  横滚力矩 → 差动升降副翼 (CS0 + CS2)                │  │
│  │  俯仰力矩 → 全部升降面 (CS0 + CS1 + CS2)            │  │
│  │  偏航力矩 → 差动推力 (Motor0 vs Motor2)              │  │
│  │  推力     → 全部电机 (Motor0 + Motor1 + Motor2)      │  │
│  └──────────────────────┬──────────────────────────────┘  │
│                         │                                 │
└─────────────────────────┼─────────────────────────────────┘
                          │ uORB / GZ Bridge
┌─────────────────────────┴─────────────────────────────────┐
│                    Gazebo 仿真环境                          │
│                                                            │
│  ┌────────┐   ┌────────┐   ┌────────┐                     │
│  │ Motor0 │   │ Motor1 │   │ Motor2 │   ← 差动推力        │
│  │ (CCW)  │   │ (CW)   │   │ (CCW)  │                     │
│  └───┬────┘   └───┬────┘   └───┬────┘                     │
│      │            │            │                           │
│  ╔═══╧════════════╧════════════╧═══╗                       │
│  ║   左单元      中间单元     右单元  ║  ← 链翼本体        │
│  ║  Y=-1.2m      Y=0m       Y=+1.2m ║                     │
│  ║  [升降副翼]  [升降舵]   [升降副翼] ║                     │
│  ╚═════════════════════════════════╝                       │
│                                                            │
│  空气动力学: LiftDrag 插件 × 8 (3翼段+2水平尾翼+3垂直尾翼) │
│  传感器: IMU + 气压计 (GZ) + GPS/磁力计/空速 (PX4模拟)      │
└────────────────────────────────────────────────────────────┘
```

### 2.2 软件层次

| 层次 | 组件 | 修改状态 |
|------|------|----------|
| 应用层 | Commander, Navigator, Logger | 未修改（标准 PX4） |
| 控制层 | FixedwingAttitudeControl | **已修改**：航向保持逻辑 |
| 控制器 | ecl_yaw_controller | **已修改**：航向保持算法 + 空速缩放 |
| 控制器 | ecl_roll_controller, ecl_pitch_controller | 未修改 |
| 分配层 | ControlAllocator | 未修改（通过参数配置） |
| 估计层 | EKF2, airspeed_selector | 未修改 |
| 驱动层 | GZ Bridge (gz_bridge) | 未修改 |
| 仿真层 | Gazebo model.sdf | **新增**：链翼气动模型 |
| 配置层 | 4007_gz_chainwing / 2150_chainwing | **新增**：机架配置文件 |

---

## 3. 飞行器构型与物理参数

### 3.1 构型示意图

```
俯视图 (Top View):
                      前方 →
                        
        Motor0 (CCW)    Motor1 (CW)    Motor2 (CCW)
           ↓               ↓               ↓
    ┌──────●──────┐ ┌──────●──────┐ ┌──────●──────┐
    │    左单元    │⊙│   中间单元   │⊙│    右单元    │
    │  Y=-1.2m    │铰│   Y=0.0m    │铰│  Y=+1.2m    │
    │  1.2m翼展   │接│  1.2m翼展   │接│  1.2m翼展   │
    │             │点│             │点│             │
    │  [右升降副翼]│  │  [升降舵]   │  │ [左升降副翼] │
    └──────┤──────┘  └──────┤──────┘  └──────┤──────┘
           │               │               │
         垂尾             垂尾             垂尾
         (被动)           (被动)           (被动)

           ← ─ ─ ─  3.6m 总翼展  ─ ─ ─ →


侧视图 (Side View):
                    
    Motor           尾翼
      ↓   机翼       ↓
      ●───━━━━━━━━───┤
      │   机身       │垂尾
      └──────────────┘
     ●              ●
    前轮            后轮
```

### 3.2 物理参数（源自 MATLAB 开环验证）

#### 单体参数

| 参数 | 符号 | 值 | 单位 | 来源 |
|------|------|-----|------|------|
| 单机质量 | m | 1.9 | kg | ThreeBodyDynamics.m |
| 单机翼展 | b | 1.2 | m | ThreeBodyDynamics.m |
| 弦长 | c | 0.3 | m | ThreeBodyAerodynamics.m |
| 翼面积 | S | 0.36 | m² | ThreeBodyAerodynamics.m |
| 横滚惯量 | Ixx | 0.0894 | kg·m² | ThreeBodyDynamics.m |
| 俯仰惯量 | Iyy | 0.144 | kg·m² | ThreeBodyDynamics.m |
| 偏航惯量 | Izz | 0.162 | kg·m² | ThreeBodyDynamics.m |

#### 组合体参数（通过平行轴定理计算）

| 参数 | 符号 | 值 | 单位 | 计算方法 |
|------|------|-----|------|----------|
| 总质量 | M | 5.7 | kg | 3 × 1.9 |
| 总翼展 | B | 3.6 | m | 3 × 1.2 |
| 总横滚惯量 | Ixx | 5.740 | kg·m² | Σ(Ixx_i + m_i × y_i²) |
| 总俯仰惯量 | Iyy | 0.432 | kg·m² | 3 × Iyy_单体 |
| 总偏航惯量 | Izz | 5.958 | kg·m² | Σ(Izz_i + m_i × y_i²) |

> **惯量计算详解**：
> - Ixx = 3×0.0894 + 1.9×(-1.2)² + 1.9×0² + 1.9×1.2² = 0.2682 + 2.736 + 0 + 2.736 = 5.740
> - Izz = 3×0.162 + 1.9×1.44 + 0 + 1.9×1.44 = 0.486 + 2.736 + 2.736 = 5.958
> - Iyy = 3×0.144 = 0.432（Y 轴对称，无平行轴偏移）
>
> **三角不等式验证**：|Ixx - Iyy| = 5.308 < Izz = 5.958 ✓

### 3.3 气动参数

| 参数 | 符号 | 值 | 单位 | 说明 |
|------|------|-----|------|------|
| 升力线斜率 | CLα | 5.25 | /rad | 有限翼展修正后的 2π |
| 零升攻角 | α₀ | -0.05 | rad | 约 -2.9° |
| 失速攻角 | α_stall | 0.227 | rad | 约 13° |
| 零升阻力系数 | CD0 | 0.01 | — | |
| 诱导阻力因子 | K | 0.0313 | — | |
| 俯仰力矩系数 | Cm0 | 0.135 | — | |
| 俯仰力矩斜率 | Cmα | -1.5 | /rad | 静稳定性 |
| 升降舵效率 | CmδE | -1.13 | /rad | |
| 俯仰阻尼 | Cmq | -50.8 | — | |
| 偏航阻尼 | Cnr | -0.411 | — | |
| 横滚阻尼 | Clp | -0.414 | — | |

### 3.4 动力参数

| 参数 | 值 | 单位 | 说明 |
|------|-----|------|------|
| 电机数量 | 3 | — | 左/中/右各一个 |
| 最大推力/电机 | 15.0 | N | motorConstant=1.5e-5, maxRPM=1000 |
| 总最大推力 | 45.0 | N | 推重比 ≈ 0.81 |
| 巡航速度 | 20 | m/s | MATLAB V0 |
| 巡航油门 | 0.60 | — | MATLAB 配平值 |
| 失速速度 | 8 | m/s | |
| 最低安全速度 | 15 | m/s | |
| 最大速度 | 30 | m/s | |

### 3.5 电机旋转方向

```
Motor0 (左)    Motor1 (中)    Motor2 (右)
   CCW            CW            CCW

反扭矩分析:
  2 × CCW + 1 × CW = 净 1 单位 CCW 反扭矩
  （相比 3 × CCW 减少 66% 净反扭矩）
```

---

## 4. 控制系统设计（核心创新）

### 4.1 控制通道分配

链翼无人机有三个控制通道，每个通道使用不同的执行器：

| 控制通道 | 执行器 | 机制 |
|----------|--------|------|
| **横滚** | 左/右升降副翼 (CS0, CS2) | 差动偏转产生横滚力矩 |
| **俯仰** | 全部升降面 (CS0, CS1, CS2) | 同向偏转产生俯仰力矩 |
| **偏航** | 左/右电机 (Motor0, Motor2) | 差动推力产生偏航力矩 |

### 4.2 升降副翼配置

#### 配置说明

链翼构型的升降副翼沿用与 rc_cessna 副翼相同的标准配置：
- 左单元 → 使用 **Left Elevon** (Type 5)，TRQ_R = -0.5
- 右单元 → 使用 **Right Elevon** (Type 6)，TRQ_R = +0.5
- 中间单元 → 使用 **Elevator** (Type 3)

在 GZ LiftDrag 插件中，`control_joint_rad_to_cl = -0.3` 意味着正舵偏角降低升力系数。
因此横滚效率符号必须与 rc_cessna 一致：左侧负、右侧正。

```
横滚右指令时:
  CS0 (左, Left Elevon):  TRQ_R = -0.5  → servo_0 减小 → 左翼 Cl↑ → 左翼升力↑
  CS2 (右, Right Elevon): TRQ_R = +0.5  → servo_2 增大 → 右翼 Cl↓ → 右翼升力↓
  → 左翼上、右翼下 = 向右横滚 ✓

俯仰下指令时:
  CS0: TRQ_P = +0.5  → 贡献俯仰
  CS1: TRQ_P = +1.0  → 主俯仰控制
  CS2: TRQ_P = +0.5  → 贡献俯仰
  → 三面协同产生俯仰力矩 ✓
```

#### 配置代码

```bash
# 4007_gz_chainwing (第 46-68 行)
param set-default CA_SV_CS0_TYPE 5       # Left unit: Left Elevon
param set-default CA_SV_CS0_TRQ_R -0.5   # 横滚贡献: -0.5 (同 rc_cessna 左副翼)
param set-default CA_SV_CS0_TRQ_P 0.5    # 俯仰贡献: +0.5

param set-default CA_SV_CS1_TYPE 3       # Center unit: Elevator
param set-default CA_SV_CS1_TRQ_P 1.0    # 纯俯仰控制

param set-default CA_SV_CS2_TYPE 6       # Right unit: Right Elevon
param set-default CA_SV_CS2_TRQ_R 0.5    # 横滚贡献: +0.5 (同 rc_cessna 右副翼)
param set-default CA_SV_CS2_TRQ_P 0.5    # 俯仰贡献: +0.5
```

### 4.3 差动推力偏航控制（创新点 2）

#### 问题

链翼无人机**没有方向舵**，无法通过传统的气动舵面控制偏航。

#### 解决方案

利用三个电机在不同横向位置 (Y = -1.2m, 0, +1.2m) 的推力差产生偏航力矩：

```
偏航力矩 = Σ (Thrust_i × Y_i)

例: 右偏航 (鼻子向右):
  Motor0 (Y=-1.2): 推力 ↑  → 产生使鼻子向右的力矩 ✓
  Motor2 (Y=+1.2): 推力 ↓  → 减少使鼻子向左的力矩 ✓
  净效果: 左侧推力 > 右侧推力 → 鼻子向右转
```

PX4 控制分配器自动根据 `CA_ROTOR*_PY` 位置计算差动推力：

```bash
param set-default CA_ROTOR0_PY -1.2    # 左电机位置
param set-default CA_ROTOR1_PY 0.0     # 中间电机位置
param set-default CA_ROTOR2_PY 1.2     # 右电机位置
```

### 4.4 航向保持控制器（创新点 3 — 核心算法）

#### 问题

链翼无人机的偏航稳定性较弱：
1. **大惯量**：Izz = 5.958 kg·m²，偏航响应慢
2. **无方向舵**：只有差动推力提供偏航力矩，响应带宽低
3. **弱被动阻尼**：小垂直尾翼面积 (3 × 0.02 m²) 提供的气动阻尼有限

标准 PX4 的偏航控制器仅实现**协调转弯**（消除侧滑），不主动保持航向。

#### 解决方案

在 `ecl_yaw_controller.cpp` 中新增**航向保持反馈**：

```
新偏航速率 = 协调转弯偏航速率 + 航向保持修正

航向保持修正 = heading_error × scaled_gain

其中:
  heading_error = wrap_pi(yaw_setpoint - yaw_actual)
  scaled_gain   = FW_YAW_STAB_SC × (airspeed / trim_airspeed)²
```

#### 空速依赖增益缩放

这是防止低速打转的关键设计：

```
                 增益 (相对巡航)
    100% ─────────────●  (20 m/s 巡航)
                     /
     56% ──────────●    (15 m/s 接近)
                  /
     25% ──────●        (10 m/s 降落)
              /
     16% ────●          (8 m/s 失速)
            /
      0% ──●
            0    5   10   15   20   25   30
                      空速 (m/s)
```

**物理依据**：低速飞行时，差动推力和垂直尾翼的偏航控制效率都下降，如果增益不随空速衰减，会导致过修正 → 正反馈 → 打转。

#### 代码实现

**文件**: `src/modules/fw_att_control/ecl_yaw_controller.cpp` (第 101-121 行)

```cpp
/* 航向保持 — 链翼偏航增稳 */
if (_heading_hold_gain > FLT_EPSILON &&
    PX4_ISFINITE(ctl_data.yaw_setpoint) && PX4_ISFINITE(ctl_data.yaw)) {

    // 计算航向误差
    const float heading_error = wrap_pi(ctl_data.yaw_setpoint - ctl_data.yaw);

    // 空速依赖增益缩放: (V / V_trim)²
    const float airspeed_ratio = math::constrain(
        ctl_data.airspeed_constrained / math::max(_trim_airspeed, 1.f),
        0.1f, 1.0f);
    const float scaled_gain = _heading_hold_gain * airspeed_ratio * airspeed_ratio;

    // P 控制: 航向误差 × 缩放增益 → 偏航速率修正
    const float heading_rate_correction = heading_error * scaled_gain;
    _body_rate_setpoint += math::constrain(heading_rate_correction, -_max_rate, _max_rate);
    _body_rate_setpoint = math::constrain(_body_rate_setpoint, -_max_rate, _max_rate);
}
```

**接口定义**: `src/modules/fw_att_control/ecl_yaw_controller.h` (第 70-75 行)

```cpp
// 链翼特有的 setter 方法
void set_heading_hold_gain(float gain) { _heading_hold_gain = gain; }
void set_trim_airspeed(float airspeed) { _trim_airspeed = airspeed; }

private:
    float _heading_hold_gain{0.f};   // FW_YAW_STAB_SC 参数
    float _trim_airspeed{15.f};      // FW_AIRSPD_TRIM 参数
```

**向后兼容性**：当 `FW_YAW_STAB_SC = 0` 时，`_heading_hold_gain` 为零，条件 `> FLT_EPSILON` 不满足，航向保持完全禁用，控制器退化为标准 PX4 行为。

### 4.5 增稳模式航向锁定

在增稳模式 (Stabilized) 下，飞行员通过摇杆控制横滚和俯仰，但偏航需要自动保持。

**文件**: `src/modules/fw_att_control/FixedwingAttitudeControl.cpp`

```cpp
// 参数传递 (第 86-87 行)
_yaw_ctrl.set_heading_hold_gain(_param_fw_yaw_stab_sc.get());
_yaw_ctrl.set_trim_airspeed(_param_fw_airspd_trim.get());

// 航向设定值管理 (第 109-120 行)
if (_param_fw_yaw_stab_sc.get() > FLT_EPSILON) {
    // 首次进入: 锁定当前航向
    if (!_heading_setpoint_initialized) {
        _heading_setpoint = yaw_body;
        _heading_setpoint_initialized = true;
    }
    _att_sp.yaw_body = _heading_setpoint;
} else {
    _att_sp.yaw_body = yaw_body; // 标准 PX4: 不控制偏航
}

// 偏航摇杆更新航向 (第 391-398 行)
if (_param_fw_yaw_stab_sc.get() > FLT_EPSILON) {
    const float yaw_stick = _manual_control_setpoint.yaw;
    if (fabsf(yaw_stick) > 0.05f) {
        // 摇杆积分式更新航向设定值
        _heading_setpoint += yaw_stick * radians(_param_fw_y_rmax.get()) * dt;
        _heading_setpoint = wrap_pi(_heading_setpoint);
    }
}
```

**工作流程**：
1. 进入增稳模式 → 锁定当前航向 (`_heading_setpoint = yaw_body`)
2. 飞行员不操作偏航摇杆 → 航向保持不变
3. 飞行员推偏航摇杆 → 以最大偏航速率更新航向
4. 松开摇杆 → 锁定新航向
5. 着陆时重置 (`_heading_setpoint_initialized = false`)

### 4.6 控制数据流总览

```
                    传感器输入
                       │
    ┌──────────────────┼──────────────────┐
    │                  │                  │
    ▼                  ▼                  ▼
┌────────┐       ┌──────────┐       ┌──────────┐
│ 横滚   │       │ 俯仰     │       │ 偏航     │
│ 控制器 │       │ 控制器   │       │ 控制器 ★ │
│        │       │          │       │          │
│ 输入:  │       │ 输入:    │       │ 输入:    │
│ roll_sp│       │ pitch_sp │       │ yaw_sp   │
│ roll   │       │ pitch    │       │ yaw      │
│        │       │          │       │ airspeed │
│ 输出:  │       │ 输出:    │       │          │
│ roll_  │       │ pitch_   │       │ 输出:    │
│ rate_sp│       │ rate_sp  │       │ yaw_     │
└───┬────┘       └────┬─────┘       │ rate_sp  │
    │                 │             │          │
    │                 │             │ 算法:    │
    │                 │             │ coord_   │
    │                 │             │ turn +   │
    │                 │             │ heading_ │
    │                 │             │ hold ★   │
    │                 │             └────┬─────┘
    │                 │                  │
    ▼                 ▼                  ▼
┌─────────────────────────────────────────────┐
│              速率控制器 (Rate Control)        │
│  roll_rate → torque_roll                     │
│  pitch_rate → torque_pitch                   │
│  yaw_rate → torque_yaw                       │
└──────────────────┬──────────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────────┐
│              控制分配器 (Allocator)           │
│                                             │
│  torque_roll  → CS0(+0.5) + CS2(-0.5)       │
│  torque_pitch → CS0(+0.5) + CS1(+1.0)       │
│                 + CS2(+0.5)                  │
│  torque_yaw   → Motor0(Y=-1.2)              │
│                 + Motor2(Y=+1.2)             │
│  thrust       → Motor0 + Motor1 + Motor2    │
└──────────────────┬──────────────────────────┘
                   │
                   ▼
              执行器输出
         (3电机 + 3舵面)
```

---

## 5. Gazebo 仿真模型

### 5.1 模型文件结构

```
Tools/simulation/gz/models/chainwing/
├── model.config          # 模型元数据 (21 行)
└── model.sdf             # 完整 SDF 模型 (968 行)
```

### 5.2 model.sdf 结构概览

| 段落 | 行数 | 内容 |
|------|------|------|
| 头部注释 | 1-15 | 参数来源说明（MATLAB数据） |
| base_link 惯性 | 16-30 | 质量 5.7kg, 惯量矩阵 |
| 碰撞几何 | 31-80 | 机身碰撞箱, 机翼碰撞箱, 起落架 |
| 视觉模型 | 81-350 | 3个机身, 3段机翼, 3个水平尾翼, 3个垂直尾翼 |
| 传感器 | 351-430 | IMU (250Hz), 气压计 (50Hz) |
| 舵面关节 | 431-500 | servo_0, servo_1, servo_2 旋转关节 |
| 电机关节 | 501-570 | rotor_left/center/right 旋转关节 |
| 升力-阻力插件 | 571-770 | 左翼, 中翼, 右翼, 左尾, 中尾, 右尾 |
| 垂直尾翼插件 | 771-830 | 3个被动偏航阻尼面 |
| 电机插件 | 831-910 | 3个电机模型（含转动惯量和阻尼） |
| 舵面控制器 | 911-940 | 3个位置PID控制器 |
| 里程计插件 | 941-968 | GZ-PX4 通信桥接 |

### 5.3 气动插件配置

每个翼段和尾翼都有独立的 LiftDrag 插件：

| 插件 | 位置 (x, y, z) | 面积 (m²) | 控制面 | 说明 |
|------|----------------|-----------|--------|------|
| 左翼段 | (-0.07, -1.20, 0.03) | 0.36 | servo_0 | 左升降副翼控制 |
| 中翼段 | (-0.07, 0, 0.03) | 0.36 | — | 被动（无控制面） |
| 右翼段 | (-0.07, 1.20, 0.03) | 0.36 | servo_2 | 右升降副翼控制 |
| 左水平尾 | (-0.55, -1.20, 0.08) | 0.18 | servo_1 | 升降舵共享 |
| 中水平尾 | (-0.55, 0, 0.08) | 0.18 | servo_1 | 升降舵控制 |
| 右水平尾 | (-0.55, 1.20, 0.08) | 0.18 | servo_1 | 升降舵共享 |
| 左垂直尾 | (-0.55, -1.20, 0.09) | 0.02 | — | 被动偏航阻尼 |
| 中垂直尾 | (-0.55, 0, 0.09) | 0.02 | — | 被动偏航阻尼 |
| 右垂直尾 | (-0.55, 1.20, 0.09) | 0.02 | — | 被动偏航阻尼 |

### 5.4 电机模型参数

| 参数 | Motor0 (左) | Motor1 (中) | Motor2 (右) |
|------|-------------|-------------|-------------|
| 旋转方向 | CCW | **CW** | CCW |
| motorConstant | 1.5e-05 | 1.5e-05 | 1.5e-05 |
| momentConstant | 0.06 | 0.06 | 0.06 |
| maxRotVelocity | 1000 rad/s | 1000 rad/s | 1000 rad/s |
| rotorDragCoefficient | 8.06e-05 | 8.06e-05 | 8.06e-05 |
| timeConstantUp | 0.0125 s | 0.0125 s | 0.0125 s |
| timeConstantDown | 0.025 s | 0.025 s | 0.025 s |

### 5.5 传感器配置

| 传感器 | 来源 | 更新率 | 说明 |
|--------|------|--------|------|
| IMU (加速度计+陀螺仪) | GZ `imu_sensor` | 250 Hz | 在 base_link 中 |
| 气压计 | GZ `air_pressure_sensor` | 50 Hz | 高斯噪声 0.01 Pa |
| GPS | PX4 `sensor_gps_sim` | 模拟 | SENS_EN_GPSSIM=1 |
| 磁力计 | PX4 `sensor_mag_sim` | 模拟 | SENS_EN_MAGSIM=1 |
| 空速管 | PX4 `sensor_airspeed_sim` | 模拟 | SENS_EN_ARSPDSIM=1 |

---

## 6. PX4 固件修改清单

### 6.1 修改文件总览

| # | 文件 | 修改类型 | 修改量 | 说明 |
|---|------|----------|--------|------|
| 1 | `src/modules/fw_att_control/ecl_yaw_controller.cpp` | **修改** | +21 行 | 航向保持算法 |
| 2 | `src/modules/fw_att_control/ecl_yaw_controller.h` | **修改** | +4 行 | setter 方法和成员变量 |
| 3 | `src/modules/fw_att_control/FixedwingAttitudeControl.cpp` | **修改** | +15 行 | 航向设定值管理 |
| 4 | `src/modules/fw_att_control/FixedwingAttitudeControl.hpp` | **修改** | +2 行 | 成员变量声明 |
| 5 | `ROMFS/px4fmu_common/init.d-posix/airframes/4007_gz_chainwing` | **新增** | 219 行 | SITL 机架配置 |
| 6 | `ROMFS/px4fmu_common/init.d/airframes/2150_chainwing` | **新增** | 86 行 | 硬件机架配置 |
| 7 | `Tools/simulation/gz/models/chainwing/model.sdf` | **新增** | 968 行 | GZ 气动模型 |
| 8 | `Tools/simulation/gz/models/chainwing/model.config` | **新增** | 21 行 | 模型元数据 |

### 6.2 代码修改详解

#### 6.2.1 ecl_yaw_controller.cpp — 航向保持算法

**位置**: 第 101-121 行（`control_attitude` 函数末尾）

**修改内容**:
- 在标准协调转弯计算之后，新增航向误差比例反馈
- 增益按 `(V/V_trim)²` 缩放，防止低速过修正
- 使用 `wrap_pi()` 正确处理 ±π 边界
- 结果受 `_max_rate` 限制（防止饱和）

**代码行数**: +21 行（不含注释）

#### 6.2.2 ecl_yaw_controller.h — 接口扩展

**位置**: 第 70-75 行

**修改内容**:
- 新增 `set_heading_hold_gain(float)` 方法
- 新增 `set_trim_airspeed(float)` 方法
- 新增 `_heading_hold_gain` 和 `_trim_airspeed` 私有成员

**代码行数**: +4 行

#### 6.2.3 FixedwingAttitudeControl.cpp — 航向设定值管理

**修改位置 1**: 第 86-87 行（`parameters_update()`）
- 将 `FW_YAW_STAB_SC` 和 `FW_AIRSPD_TRIM` 参数传递给偏航控制器

**修改位置 2**: 第 109-120 行（`vehicle_manual_poll()`）
- 增稳模式下：首次进入时锁定当前航向，后续通过偏航摇杆更新

**修改位置 3**: 第 313 行（着陆检测）
- 着陆时重置 `_heading_setpoint_initialized = false`

**修改位置 4**: 第 391-398 行（偏航摇杆处理）
- 摇杆积分式更新航向设定值

**代码行数**: +15 行

#### 6.2.4 FixedwingAttitudeControl.hpp — 成员变量

**位置**: 第 160-165 行

**修改内容**:
- 新增 `_heading_setpoint` (float)
- 新增 `_heading_setpoint_initialized` (bool)

**代码行数**: +2 行

### 6.3 未修改的关键模块

以下模块**未做任何修改**，完全使用标准 PX4 代码：

| 模块 | 说明 |
|------|------|
| `fw_rate_control` | 速率控制器（PID） |
| `control_allocator` | 控制分配（通过参数 CA_* 配置） |
| `ekf2` | 状态估计（IMU/GPS/磁力计/气压/空速融合） |
| `navigator` | 航线/RTL/Loiter 导航 |
| `commander` | 模式管理/安全检查 |
| `airspeed_selector` | 空速选择/验证 |
| `gz_bridge` | Gazebo 通信桥接 |

---

## 7. 参数配置手册

### 7.1 SITL 配置 (4007_gz_chainwing)

#### 7.1.1 控制分配参数

| 参数 | 值 | 说明 |
|------|-----|------|
| CA_AIRFRAME | 1 | 通用机架类型 |
| CA_ROTOR_COUNT | 3 | 三电机 |
| CA_ROTOR0_PY | -1.2 | 左电机 Y 位置 (m) |
| CA_ROTOR1_PY | 0.0 | 中电机 Y 位置 (m) |
| CA_ROTOR2_PY | 1.2 | 右电机 Y 位置 (m) |
| CA_SV_CS_COUNT | 3 | 三舵面 |
| CA_SV_CS0_TYPE | 5 (Left Elevon) | 左单元升降副翼 |
| CA_SV_CS0_TRQ_R | -0.5 | 横滚效率 |
| CA_SV_CS0_TRQ_P | 0.5 | 俯仰效率 |
| CA_SV_CS1_TYPE | 3 (Elevator) | 中间升降舵 |
| CA_SV_CS1_TRQ_P | 1.0 | 俯仰效率 |
| CA_SV_CS2_TYPE | 6 (Right Elevon) | 右单元升降副翼 |
| CA_SV_CS2_TRQ_R | 0.5 | 横滚效率 |
| CA_SV_CS2_TRQ_P | 0.5 | 俯仰效率 |

#### 7.1.2 空速参数

| 参数 | 值 | 单位 | 说明 |
|------|-----|------|------|
| FW_AIRSPD_STALL | 8 | m/s | 失速速度 |
| FW_AIRSPD_MIN | 15 | m/s | 最低安全飞行速度 |
| FW_AIRSPD_TRIM | 20 | m/s | 巡航速度（MATLAB V0） |
| FW_AIRSPD_MAX | 30 | m/s | 最大速度 |

#### 7.1.3 姿态控制参数

| 参数 | 值 | 单位 | 说明 |
|------|-----|------|------|
| FW_R_TC | 0.5 | s | 横滚时间常数 |
| FW_P_TC | 0.5 | s | 俯仰时间常数 |
| FW_R_RMAX | 30.0 | °/s | 最大横滚速率 |
| FW_P_RMAX_POS | 25.0 | °/s | 最大正俯仰速率 |
| FW_P_RMAX_NEG | 25.0 | °/s | 最大负俯仰速率 |
| FW_Y_RMAX | 15.0 | °/s | 最大偏航速率 |
| FW_R_LIM | 35 | ° | 最大横滚角（防失速） |
| FW_P_LIM_MAX | 15 | ° | 最大正俯仰角 |
| FW_P_LIM_MIN | -15 | ° | 最大负俯仰角 |

#### 7.1.4 速率控制器增益

| 参数 | 值 | 说明 |
|------|-----|------|
| FW_PR_P | 0.9 | 俯仰速率 P 增益 |
| FW_PR_I | 0.5 | 俯仰速率 I 增益 |
| FW_PR_FF | 0.5 | 俯仰速率前馈 |
| FW_RR_P | 0.3 | 横滚速率 P 增益 |
| FW_RR_I | 0.5 | 横滚速率 I 增益 |
| FW_RR_FF | 0.5 | 横滚速率前馈 |
| FW_YR_P | 0.6 | 偏航速率 P 增益 |
| FW_YR_I | 0.5 | 偏航速率 I 增益 |
| FW_YR_FF | 0.5 | 偏航速率前馈 |

#### 7.1.5 航向保持与油门

| 参数 | 值 | 说明 |
|------|-----|------|
| FW_YAW_STAB_SC | 1.0 | 航向保持增益（0=禁用） |
| FW_PSP_OFF | 2.0° | 俯仰偏置（水平飞行攻角） |
| TRIM_PITCH | -0.15 | 升降舵配平 |
| FW_THR_TRIM | 0.60 | 巡航油门（MATLAB 配平值） |
| FW_THR_MIN | 0.05 | 最小油门 |
| FW_THR_MAX | 1.0 | 最大油门 |

#### 7.1.6 导航参数

| 参数 | 值 | 说明 |
|------|-----|------|
| MIS_TAKEOFF_ALT | 15 m | 起飞目标高度 |
| NAV_ACC_RAD | 20 m | 航点接受半径 |
| NAV_LOITER_RAD | 50 m | 盘旋半径 |
| NAV_DLL_ACT | 0 | 数据链路丢失动作（禁用） |
| NPFG_PERIOD | 12 | NPFG 导航控制周期 |

#### 7.1.7 RTL 参数

| 参数 | 值 | 说明 |
|------|-----|------|
| RTL_RETURN_ALT | 30 m | 返航高度 |
| RTL_DESCEND_ALT | 15 m | 返航后下降高度 |
| RTL_LAND_DELAY | 0 s | 到达后立即降落 |

#### 7.1.8 降落参数

| 参数 | 值 | 说明 |
|------|-----|------|
| FW_LND_ANG | 8° | 降落下滑角 |
| FW_LND_AIRSPD | 16 m/s | 降落接近速度 |
| FW_LND_FLALT | 5 m | 拉平开始高度 |
| FW_LND_FL_PMIN | 3° | 拉平最小俯仰 |
| FW_LND_FL_PMAX | 10° | 拉平最大俯仰 |

#### 7.1.9 SITL 专用参数

| 参数 | 值 | 说明 |
|------|-----|------|
| FD_ESCS_EN | 0 | 禁用 ESC 解锁检测 |
| COM_RC_IN_MODE | 3 | 接受 RC 或 QGC 虚拟摇杆 |
| SIM_BAT_DRAIN | 3600 | 电池耗尽时间 (秒) |
| CBRK_SUPPLY_CHK | 894281 | 绕过电源检查 |
| CP_DIST | -1 | 禁用碰撞预防 |
| SYS_DM_BACKEND | 1 | RAM 模式（重启清除任务） |

### 7.2 硬件配置 (2150_chainwing)

硬件配置文件 `2150_chainwing` 包含物理部署所需的核心参数：
- 电机/舵面配置与 SITL 完全一致
- 不包含 GZ 特定参数 (SIM_GZ_*)
- 不包含 SITL 安全绕过参数 (FD_ESCS_EN, CBRK_SUPPLY_CHK 等)
- `FW_YAW_STAB_SC = 2.0`（硬件版本使用更高增益，因为真实环境有更多扰动）

---

## 8. MATLAB/Simulink 开环验证

### 8.1 验证目的

在将气动参数应用到 Gazebo 仿真模型之前，先通过 MATLAB 多体动力学仿真验证参数的正确性。

### 8.2 文件结构

```
动力学开环验证/
├── ThreeBodyDynamics.m        # 多体约束动力学求解器 (~390 行)
├── ThreeBodyAerodynamics.m    # 气动力/力矩计算 (~95 行)
├── Run_Validation.m           # 验证测试脚本 (~49 行)
├── codebasedsimu.slx          # Simulink 模型 (113 KB)
├── codebasedsimu.slxc         # 编译缓存 (5 KB)
└── slprj/                     # Simulink 项目元数据
```

### 8.3 ThreeBodyDynamics.m — 多体动力学

实现了三体链翼系统的 Newton-Euler 约束动力学方程：

**状态向量** (36维):
```
x = [r₁, θ₁, v₁, ω₁,   (位置, 欧拉角, 速度, 角速度 — 单元1)
     r₂, θ₂, v₂, ω₂,   (单元2)
     r₃, θ₃, v₃, ω₃]   (单元3)
```

**约束条件**:
- 两个铰接点约束 (Y = ±0.6m)
- Baumgarte 稳定化 (α = β = 20.0)

**物理参数**:
```matlab
m = 1.9;                          % 单机质量 (kg)
J = diag([0.0894, 0.144, 0.162]); % 惯量矩阵 (kg·m²)
b = 1.2;                          % 翼展 (m)
```

### 8.4 ThreeBodyAerodynamics.m — 气动力计算

为每个单元计算独立的气动力和力矩：

```matlab
rho = 1.225;        % 空气密度 (kg/m³)
span = 1.2;         % 翼展 (m)
chord = 0.3;        % 弦长 (m)
S_wing = 0.36;      % 翼面积 (m²)

a0 = 2*pi;          % 升力线斜率 (2π)
alpha_0 = -0.05;    % 零升攻角 (rad)
Cd0 = 0.01;         % 零升阻力系数
K_ind = 0.0313;     % 诱导阻力因子

Cmq = -50.8;        % 俯仰阻尼导数
Cnr = -0.411;       % 偏航阻尼导数
Clp = -0.414;       % 横滚阻尼导数
CmDe = -1.13;       % 升降舵控制导数
```

### 8.5 参数传递链路

```
MATLAB 参数                  GZ model.sdf              PX4 参数
─────────────            ──────────────           ──────────────
m = 1.9 kg/unit    →     mass = 5.7 kg
b = 1.2 m/unit     →     wing size = 3.6×0.3 m
S = 0.36 m²        →     area = 0.36/section
a0 = 2π → 5.25    →     cla = 5.25
alpha_0 = -0.05   →     a0 = -0.05
V0 = 20 m/s        →                          →  FW_AIRSPD_TRIM = 20
T_max = 15 N       →     motorConstant=1.5e-5
throttle_trim = 0.6 →                          →  FW_THR_TRIM = 0.60
Ixx = 5.740        →     ixx = 5.740
Iyy = 0.432        →     iyy = 0.432
Izz = 5.958        →     izz = 5.958
```

---

## 9. 飞行模式说明

### 9.1 支持的飞行模式

本固件支持 PX4 的全部固定翼飞行模式，共 **15 种**：

#### 手动模式（需要遥控器/虚拟摇杆）

| 模式 | 说明 | 链翼增强 |
|------|------|----------|
| **Manual** | 完全手动，直通舵面 | 无 |
| **Acro** | 特技模式，控制角速率 | 无 |

#### 增稳模式（需要遥控器/虚拟摇杆）

| 模式 | 说明 | 链翼增强 |
|------|------|----------|
| **Stabilized** ★ | 自动保持水平，手动油门 | **航向保持** |
| **Altitude** | 自动保持高度+水平 | **航向保持** |
| **Position** | 自动保持位置+高度 | **航向保持** |

#### 自主模式（无需遥控器）

| 模式 | 说明 | 链翼增强 |
|------|------|----------|
| **Takeoff** | 自动起飞到目标高度 | **航向保持** |
| **Mission** | 执行预设航线 | **航向保持** |
| **Loiter** | 在当前位置盘旋 | **航向保持** |
| **RTL** | 自动返航降落 | **航向保持** |
| **Land** | 自动降落 | **航向保持** |
| **Orbit** | 绕指定点飞行 | **航向保持** |
| **Hold** | 保持当前位置盘旋 | **航向保持** |

> ★ 增稳模式需要先在 QGC 中启用虚拟摇杆

### 9.2 航向保持的开关

```
FW_YAW_STAB_SC = 0    → 标准 PX4 行为（所有模式无链翼增强）
FW_YAW_STAB_SC > 0    → 启用链翼航向保持（所有含偏航控制的模式）
FW_YAW_STAB_SC = 1.0  → 推荐 SITL 值
FW_YAW_STAB_SC = 2.0  → 推荐硬件值（真实环境扰动更大）
```

---

## 10. 问题诊断与解决历程

### 10.1 已解决的问题时间线

| 序号 | 问题 | 根因 | 解决方案 | 修改文件 |
|------|------|------|----------|----------|
| 1 | 无法解锁：无GCS连接 | NAV_DLL_ACT=2 | 设为 0 | 4007_gz_chainwing |
| 2 | 解锁后立即断电：ESC故障 | FD_ESCS_EN=1 + 固定翼电机0速 | 设为 0 | 4007_gz_chainwing |
| 3 | 起飞后飞机打转 | 无垂直尾翼空气动力学 | 添加3个垂直尾翼LiftDrag插件 | model.sdf |
| 4 | 降落时飞机打转 | 低速下航向保持增益过大 | 空速依赖增益缩放 | ecl_yaw_controller.cpp |
| 5 | QGC 缺少CP_DIST参数 | 固定翼不需要碰撞预防 | CP_DIST=-1 | 4007_gz_chainwing |
| 6 | 电池60秒耗尽 | SIM_BAT_DRAIN 默认60 | 设为 3600 | 4007_gz_chainwing |
| 7 | 无法切换增稳模式 | COM_RC_IN_MODE=0 | 设为 3 | 4007_gz_chainwing |
| 8 | 增稳模式切换时坠落 | FW_YAW_STAB_SC过大 + FW_R_LIM过大 | SC→1.0, R_LIM→35° | 4007_gz_chainwing |
| 9 | RTL 爬升到100m盘旋不降落 | RTL_RETURN_ALT=100, LAND_DELAY=-1 | 调整RTL参数 | 4007_gz_chainwing |
| 10 | 旧任务阻止启动 | 持久存储保留旧任务 | SYS_DM_BACKEND=1 | 4007_gz_chainwing |

### 10.2 重要发现

#### 发现 1: 垂直尾翼必须有空气动力学插件

链翼模型最初有垂直尾翼的视觉模型，但**没有对应的 LiftDrag 插件**。这意味着垂直尾翼是纯装饰品，不产生任何侧向气动力。飞机在偏航方向上完全没有被动阻尼，导致任何微小的偏航扰动都会发展成持续打转。

**修复**: 添加 3 个垂直尾翼 LiftDrag 插件，`<upward>0 1 0</upward>`，面积 0.02 m²。

#### 发现 2: 空速依赖增益缩放是关键

最初的航向保持增益在所有速度下恒定。巡航时工作良好，但在降落接近时（速度降低到 15→10→8 m/s），恒定增益导致过修正：

```
低速 → 差动推力偏航效率↓ + 垂直尾翼阻尼↓
恒定增益 → 过修正 → 横滚耦合 → 正反馈 → 打转
```

**修复**: 增益按 `(V/V_trim)²` 缩放，低速时自动衰减。

#### 发现 3: 电机反扭矩平衡很重要

最初三个电机全部 CCW 旋转，产生 3 个单位的净反扭矩。改中间电机为 CW 后，净反扭矩减少到 1 个单位（减少 66%），显著改善偏航稳定性。

---

## 11. 使用指南

### 11.1 环境要求

| 组件 | 版本 | 用途 |
|------|------|------|
| Ubuntu | 22.04+ (或 WSL2) | 操作系统 |
| PX4 Autopilot | v1.15+ | 飞控固件 |
| Gazebo (gz-sim) | Harmonic | 物理仿真 |
| QGroundControl | 4.x | 地面站 |

### 11.2 构建与启动

```bash
# 1. 进入 PX4 目录
cd ~/PX4-Autopilot

# 2. 首次运行或参数更新后：清理缓存
make clean
rm -f build/px4_sitl_default/rootfs/parameters*.bson
rm -f build/px4_sitl_default/rootfs/dataman

# 3. 构建并启动仿真
make px4_sitl gz_chainwing

# 4. (可选) 无头模式 (WSL 渲染问题时)
HEADLESS=1 make px4_sitl gz_chainwing

# 5. (可选) WSL 渲染引擎切换
export GZ_SIM_RENDER_ENGINE_GUI=ogre
make px4_sitl gz_chainwing
```

### 11.3 连接 QGC

| 环境 | 连接方式 |
|------|----------|
| 原生 Ubuntu | QGC 自动连接 (UDP 14550) |
| WSL2 | QGC (Windows) → 添加通信链路 → UDP → 端口 14550 |

### 11.4 基本飞行操作

```bash
# PX4 终端命令:

# 起飞 (自动模式)
commander takeoff

# 切换到盘旋
commander mode loiter

# 返航降落
commander rtl

# 紧急停止
commander disarm

# 切换增稳模式 (需先在QGC启用虚拟摇杆)
commander mode stabilized

# 查看状态
commander status
ekf2 status
listener airspeed
listener vehicle_attitude
```

### 11.5 实时调参

在 QGC 参数页面 (Vehicle Setup → Parameters) 中可以实时修改大部分参数：

1. **搜索参数名** (如 `FW_YAW_STAB_SC`)
2. **修改值** 并点击保存
3. **观察效果** (部分参数需要重启)

**推荐调参顺序**: 俯仰 → 横滚 → 偏航 → 航向保持 → 油门 → 导航

---

## 12. 后续开发方向

| 方向 | 优先级 | 说明 |
|------|--------|------|
| PID 精调 | 高 | 基于飞行日志进行俯仰/横滚/偏航 PID 细调 |
| 铰接动力学 | 高 | 在 Gazebo 中添加铰接约束，模拟翼尖柔性 |
| 风场测试 | 中 | 添加侧风/阵风条件下的稳定性验证 |
| 单电机失效 | 中 | 设计单电机故障时的降级飞行策略 |
| 硬件部署 | 中 | 在 Pixhawk 硬件上验证 |
| 自适应增益 | 低 | 替换固定 FW_YAW_STAB_SC 为在线自适应调度 |
| 编队飞行 | 低 | 多架链翼无人机的协同控制 |

---

## 附录 A: 文件清单

### 新增文件

| 文件路径 | 行数 | 类型 | 说明 |
|----------|------|------|------|
| `ROMFS/.../4007_gz_chainwing` | 219 | Shell | SITL 机架配置 |
| `ROMFS/.../2150_chainwing` | 86 | Shell | 硬件机架配置 |
| `Tools/.../chainwing/model.sdf` | 968 | XML | Gazebo 气动模型 |
| `Tools/.../chainwing/model.config` | 21 | XML | 模型元数据 |
| `动力学开环验证/ThreeBodyDynamics.m` | ~390 | MATLAB | 多体动力学 |
| `动力学开环验证/ThreeBodyAerodynamics.m` | ~95 | MATLAB | 气动力计算 |
| `动力学开环验证/Run_Validation.m` | ~49 | MATLAB | 验证脚本 |
| `动力学开环验证/codebasedsimu.slx` | — | Simulink | 仿真模型 |
| `CHAINWING_CONTROL_FLOW.md` | 1333 | Markdown | 控制流程文档 |
| `CHAINWING_TUNING_GUIDE.md` | 894 | Markdown | 调参指南 |
| `CHAINWING_FIRMWARE_DOC.md` | — | Markdown | 本文档 |

### 修改文件

| 文件路径 | 改动行数 | 说明 |
|----------|----------|------|
| `src/.../ecl_yaw_controller.cpp` | +21 | 航向保持算法 |
| `src/.../ecl_yaw_controller.h` | +4 | 接口扩展 |
| `src/.../FixedwingAttitudeControl.cpp` | +15 | 航向设定值管理 |
| `src/.../FixedwingAttitudeControl.hpp` | +2 | 成员变量 |

---

## 附录 B: 参数速查表

### 链翼核心参数

| 参数 | 默认值 | 链翼值 | 用途 |
|------|--------|--------|------|
| FW_YAW_STAB_SC | 0 | **1.0** | 航向保持增益 (关键!) |
| CA_ROTOR_COUNT | 1 | **3** | 电机数量 |
| CA_ROTOR0_PY | 0 | **-1.2** | 左电机位置 |
| CA_ROTOR2_PY | 0 | **1.2** | 右电机位置 |
| CA_SV_CS0_TYPE | 1 | **6** | 反向升降副翼 (左) |
| CA_SV_CS2_TYPE | 2 | **5** | 反向升降副翼 (右) |

### 安全参数 (SITL 专用)

| 参数 | PX4 默认 | 链翼值 | 原因 |
|------|----------|--------|------|
| FD_ESCS_EN | 1 | **0** | GZ 电机反馈延迟 |
| COM_RC_IN_MODE | 0 | **3** | QGC 虚拟摇杆 |
| SIM_BAT_DRAIN | 60 | **3600** | 测试时间不足 |
| NAV_DLL_ACT | 2 | **0** | 无 GCS 也能解锁 |
| RTL_LAND_DELAY | -1 | **0** | 默认永不降落 |

---

> **文档结束**
>
> 如有问题或需要更多细节，请参考：
> - `CHAINWING_CONTROL_FLOW.md` — 控制架构详细数据流
> - `CHAINWING_TUNING_GUIDE.md` — 逐步调参操作手册
> - PX4 官方文档: https://docs.px4.io/
