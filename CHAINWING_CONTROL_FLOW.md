# Chain-Wing 三体无人机 完整控制流程文档

> **版本**: v1.0  
> **日期**: 2026-03-23  
> **适用**: PX4 v1.14 + Gazebo Harmonic + chainwing_3body 模型  
> **读者**: 需要理解整个项目控制链路的工程师 / 评审人员

---

## 目录

- [1. 项目总体架构](#1-项目总体架构)
- [2. 系统控制层级](#2-系统控制层级)
- [3. 第一层：任务规划与导航](#3-第一层任务规划与导航)
- [4. 第二层：位置控制与TECS](#4-第二层位置控制与tecs)
- [5. 第三层：姿态控制](#5-第三层姿态控制)
- [6. 第四层：角速率控制](#6-第四层角速率控制)
- [7. 第五层：控制分配](#7-第五层控制分配)
- [8. 第六层：从机铰链修正控制（自定义）](#8-第六层从机铰链修正控制自定义)
- [9. 第七层：Gazebo仿真接口](#9-第七层gazebo仿真接口)
- [10. 第八层：Gazebo物理仿真](#10-第八层gazebo物理仿真)
- [11. 完整数据流追踪](#11-完整数据流追踪)
- [12. 系统时序与频率分析](#12-系统时序与频率分析)
- [13. 数值计算示例](#13-数值计算示例)
- [14. 铰链修正专题：阵风干扰完整响应](#14-铰链修正专题阵风干扰完整响应)
- [15. 参数速查表](#15-参数速查表)
- [16. 源文件清单](#16-源文件清单)

---

## 1. 项目总体架构

### 1.1 物理结构

Chain-Wing 三体无人机由三个独立的固定翼机体通过翼尖铰链连接：

```
              ← 4.8m 总翼展 →
   ┌─────────────┬─────────────┬─────────────┐
   │  左从机      ⊕  中央主机    ⊕  右从机      │
   │ (left_unit) │ (base_link) │(right_unit) │
   │  1.9 kg     │  1.9 kg     │  1.9 kg     │
   │  翼展 1.2m  │  翼展 1.2m  │  翼展 1.2m  │
   │  Servo_0    │  Servo_1    │  Servo_2    │
   │  Motor_0    │  Motor_1    │  Motor_2    │
   └─────────────┴─────────────┴─────────────┘
         ⊕ = 翼尖铰链 (revolute joint, X轴旋转)
             允许相对滚转 ±15°
```

### 1.2 控制架构分层

```
┌─────────────────────────────────────────────────────────┐
│ 第1层 ── 任务规划 (navigator)                            │
│          目标航点 → 期望位置/高度                         │
├─────────────────────────────────────────────────────────┤
│ 第2层 ── 位置控制 (fw_pos_control + TECS)               │
│          位置误差 → 期望姿态 + 油门                      │
├─────────────────────────────────────────────────────────┤
│ 第3层 ── 姿态控制 (fw_att_control)                      │
│          姿态误差 → 期望角速率                           │
├─────────────────────────────────────────────────────────┤
│ 第4层 ── 角速率控制 (fw_rate_control)                    │
│          角速率误差 → 力矩指令 [-1, 1]                   │
├─────────────────────────────────────────────────────────┤
│ 第5层 ── 控制分配 (control_allocator)                    │
│          力矩指令 → 各舵面行程 [0, 1000]                 │
├─────────────────────────────────────────────────────────┤
│ 第6层 ── 铰链修正 (chainwing_slave) ★ 自定义模块         │
│          铰链偏角 → PD修正 → 叠加到从机elevon舵面        │
├─────────────────────────────────────────────────────────┤
│ 第7层 ── 仿真接口 (gz_bridge / GZMixingInterfaceServo)  │
│          PX4舵面指令 → Gazebo关节目标位置                │
├─────────────────────────────────────────────────────────┤
│ 第8层 ── 物理仿真 (Gazebo Harmonic)                     │
│          关节控制器 + 气动力 + 铰链动力学 + IMU          │
└─────────────────────────────────────────────────────────┘
```

---

## 2. 系统控制层级

### 2.1 完整控制回路框图

```
                         ┌──────────┐
                         │ 任务航点  │
                         └────┬─────┘
                              │ position_setpoint_triplet
                              ▼
                    ┌──────────────────┐
                    │ fw_pos_control   │ 50 Hz
                    │ (TECS + L1/NPFG) │
                    └────────┬─────────┘
                             │ vehicle_attitude_setpoint
                             │ (φ_sp, θ_sp, ψ_sp, thrust)
                             ▼
                    ┌──────────────────┐
                    │ fw_att_control   │ ~50 Hz (event-driven)
                    │ (3轴姿态PID)     │
                    └────────┬─────────┘
                             │ vehicle_rates_setpoint
                             │ (p_sp, q_sp, r_sp)
                             ▼
                    ┌──────────────────┐
                    │ fw_rate_control  │ ~250 Hz (IMU-driven)
                    │ (3轴角速率PID)   │
                    └──┬─────────┬─────┘
                       │         │
        vehicle_torque_setpoint  vehicle_thrust_setpoint
        (τ_roll, τ_pitch, τ_yaw) (T_x)
                       │         │
                       ▼         ▼
                    ┌──────────────────┐
                    │ control_allocator│ ~50 Hz
                    │ (效率矩阵求逆)   │
                    └────────┬─────────┘
                             │ actuator_servos [0..2]
                             │ actuator_motors [0..2]
                             ▼
          ┌──────────────────┼──────────────────┐
          │                  │                  │
   ┌──────▼──────┐   ┌──────▼──────┐   ┌──────▼──────┐
   │ Servo_0     │   │ Servo_1     │   │ Servo_2     │
   │(左从机Elevon)│   │(中央升降舵) │   │(右从机Elevon)│
   └──────┬──────┘   └──────┬──────┘   └──────┬──────┘
          │                  │                  │
          │ +trim_left       │ (无修正)          │ +trim_right
          │                  │                  │
   ┌──────▼──────────────────▼──────────────────▼──────┐
   │           GZMixingInterfaceServo                   │
   │    (叠加铰链修正 → 发送到 Gazebo)                   │
   └──────┬──────────────────┬──────────────────┬──────┘
          │                  │                  │
          ▼                  ▼                  ▼
   ┌─────────────┐   ┌─────────────┐   ┌─────────────┐
   │ GZ 舵面控制  │   │ GZ 舵面控制  │   │ GZ 舵面控制  │
   │ (P=10.0)    │   │ (P=10.0)    │   │ (P=10.0)    │
   └──────┬──────┘   └──────┬──────┘   └──────┬──────┘
          │                  │                  │
          ▼                  ▼                  ▼
   ┌─────────────┐   ┌─────────────┐   ┌─────────────┐
   │ LiftDrag    │   │ LiftDrag    │   │ LiftDrag    │
   │ 气动力插件   │   │ 气动力插件   │   │ 气动力插件   │
   └──────┬──────┘   └──────┬──────┘   └──────┬──────┘
          │                  │                  │
          ▼                  ▼                  ▼
   ┌──────────────────────────────────────────────────┐
   │                Gazebo 物理引擎                     │
   │   (刚体动力学 + 铰链弹簧/阻尼 + 碰撞检测)         │
   └──────────────┬───────────────────────────────────┘
                  │
           IMU / GPS / 气压计 / 空速管 (传感器插件)
                  │
                  ▼
   ┌──────────────────────────┐
   │ EKF2 状态估计            │ ──→ vehicle_attitude
   │ (互补滤波 + 卡尔曼)      │ ──→ vehicle_angular_velocity
   └──────────────────────────┘ ──→ vehicle_local_position
                  │
                  └──→ (反馈到各控制器)
```

### 2.2 铰链修正控制回路（并行运行）

```
                        ┌──────────────────┐
                        │  chainwing_slave │ 50 Hz
                        │  (PD 铰链控制器)  │
                        └────┬─────────────┘
                             │
                ┌────────────┼────────────┐
                │            │            │
       vehicle_angular   vehicle    chainwing_hinge_status
       _velocity_sub    _attitude   _pub (发布)
         (读IMU角速率)   (读姿态)    ↓
                │            │     ┌──────────────────┐
                └────────────┘     │ GZMixingInterface │
                                   │ Servo (读取trim)  │
                                   └──────────────────┘
```

---

## 3. 第一层：任务规划与导航

### 3.1 模块信息

| 属性 | 值 |
|------|-----|
| 模块名 | `navigator` |
| 源文件 | `src/modules/navigator/` |
| 运行频率 | 事件驱动（位置更新时触发） |
| 输入话题 | `vehicle_local_position`, `vehicle_global_position`, `home_position`, `mission` |
| 输出话题 | `position_setpoint_triplet` |

### 3.2 功能说明

Navigator 是最上层的任务管理器，负责：
- 解析任务航点（Mission items）
- 生成当前/前一个/下一个位置设定点三元组
- 管理飞行模式（Mission / Loiter / RTL / Land / Takeoff）
- 计算航路点之间的过渡逻辑

### 3.3 关键参数

| 参数 | 值 | 说明 |
|------|-----|------|
| `MIS_TAKEOFF_ALT` | 15 m | 起飞目标高度 |
| `NAV_ACC_RAD` | 20 m | 航点接受半径 |
| `NAV_LOITER_RAD` | 50 m | 盘旋半径 |
| `NPFG_PERIOD` | 12 s | NPFG 横向导航周期 |

### 3.4 输出格式

```
position_setpoint_triplet_s:
├── previous:  前一个航点 (lat, lon, alt)
├── current:   当前目标航点 (lat, lon, alt, type, acceptance_radius)
└── next:      下一个航点 (lat, lon, alt)
```

---

## 4. 第二层：位置控制与TECS

### 4.1 模块信息

| 属性 | 值 |
|------|-----|
| 模块名 | `fw_pos_control` (FixedwingPositionControl) |
| 源文件 | `src/modules/fw_pos_control/FixedwingPositionControl.cpp` |
| 运行频率 | ~50 Hz（由 `vehicle_local_position` 更新驱动） |
| 输入话题 | `vehicle_local_position`, `vehicle_attitude`, `position_setpoint_triplet`, `airspeed_validated` |
| 输出话题 | `vehicle_attitude_setpoint` |

### 4.2 横向控制 (NPFG)

**横向导航**将航线跟踪误差转换为期望横滚角：

```
航线偏差 (cross-track error)
    │
    ▼
NPFG 算法 (Nonlinear Path Following Guidance)
    │
    ▼
期望横滚角 φ_sp = f(横向偏差, 地速, 风速)
    │
    限幅: |φ_sp| ≤ FW_R_LIM = 35°
```

### 4.3 纵向控制 (TECS)

**总能量控制系统** (Total Energy Control System) 同时管理速度和高度：

```
                    ┌─────────────┐
   期望高度 h_sp ──→│             │──→ θ_sp (俯仰设定点)
                    │   TECS      │
   期望空速 V_sp ──→│   能量分配   │──→ δ_T (油门设定点)
                    │             │
   实际高度 h    ──→│ 能量守恒:   │
   实际空速 V    ──→│ E = mgh     │
                    │   + ½mV²    │
                    └─────────────┘
```

**TECS 核心原理**：
- **总能量** E_total = 势能(mgh) + 动能(½mV²)
- **能量分配** = 势能 / 总能量
- **油门** 控制总能量的增减
- **俯仰角** 控制能量在势能和动能之间的分配

### 4.4 TECS 关键参数

| 参数 | 值 | 说明 |
|------|-----|------|
| `FW_AIRSPD_TRIM` | 20 m/s | 巡航空速 |
| `FW_AIRSPD_MIN` | 15 m/s | 最小空速 |
| `FW_AIRSPD_MAX` | 30 m/s | 最大空速 |
| `FW_AIRSPD_STALL` | 8 m/s | 失速速度 |
| `FW_T_CLMB_MAX` | 8 m/s | 最大爬升率 |
| `FW_T_SINK_MAX` | 3.0 m/s | 最大下降率 |
| `FW_T_SINK_MIN` | 2.5 m/s | 最小下降率 |
| `FW_THR_TRIM` | 0.60 | 巡航油门 |
| `FW_THR_MIN` | 0.05 | 最小油门 |
| `FW_THR_MAX` | 1.0 | 最大油门 |

### 4.5 输出

```
vehicle_attitude_setpoint_s:
├── roll_body:       φ_sp (来自 NPFG 横向导航)
├── pitch_body:      θ_sp (来自 TECS) + FW_PSP_OFF (2°偏置)
├── yaw_body:        ψ_sp (航向跟踪)
└── thrust_body[0]:  δ_T  (来自 TECS 油门, [0, 1])
```

---

## 5. 第三层：姿态控制

### 5.1 模块信息

| 属性 | 值 |
|------|-----|
| 模块名 | `fw_att_control` (FixedwingAttitudeControl) |
| 源文件 | `src/modules/fw_att_control/FixedwingAttitudeControl.cpp` |
| 运行频率 | 事件驱动（`vehicle_attitude` 更新触发），回退 50 Hz |
| 输入话题 | `vehicle_attitude`, `vehicle_attitude_setpoint`, `airspeed_validated` |
| 输出话题 | `vehicle_rates_setpoint` |

### 5.2 三轴姿态控制律

姿态控制器将姿态误差转换为期望角速率：

```
                    姿态误差                    角速率设定点
                ┌───────────────┐          ┌───────────────┐
                │               │          │               │
   φ_sp - φ ──→│  横滚控制器    │──→ p_sp  │ 限幅:         │
                │  1/FW_R_TC    │          │ |p| ≤ 30°/s   │
                │               │          │ (FW_R_RMAX)   │
                └───────────────┘          └───────────────┘

   θ_sp - θ ──→│  俯仰控制器    │──→ q_sp  │ |q| ≤ 25°/s   │
                │  1/FW_P_TC    │          │ (FW_P_RMAX)   │

   ψ_sp - ψ ──→│  偏航控制器    │──→ r_sp  │ |r| ≤ 15°/s   │
                │  协调转弯      │          │ (FW_Y_RMAX)   │
```

**横滚通道控制律**：
```
p_sp = (φ_sp - φ) / FW_R_TC
p_sp = constrain(p_sp, -FW_R_RMAX, +FW_R_RMAX)
```

**俯仰通道控制律**：
```
q_sp = (θ_sp - θ) / FW_P_TC
q_sp = constrain(q_sp, -FW_P_RMAX_NEG, +FW_P_RMAX_POS)
```

**偏航通道**：协调转弯 + 航向保持，包含 `FW_YAW_STAB_SC = 1.0` 稳定增强。

### 5.3 关键参数

| 参数 | 值 | 说明 |
|------|-----|------|
| `FW_R_TC` | 0.5 s | 横滚时间常数（越小响应越快） |
| `FW_P_TC` | 0.5 s | 俯仰时间常数 |
| `FW_R_RMAX` | 30 °/s | 最大横滚速率 |
| `FW_P_RMAX_POS` | 25 °/s | 最大抬头速率 |
| `FW_P_RMAX_NEG` | 25 °/s | 最大低头速率 |
| `FW_Y_RMAX` | 15 °/s | 最大偏航速率 |
| `FW_PSP_OFF` | 2° | 俯仰设定点偏置（补偿升力需要的迎角） |
| `FW_YAW_STAB_SC` | 1.0 | 偏航稳定增强比例 |

### 5.4 输出

```
vehicle_rates_setpoint_s:
├── roll:  p_sp (rad/s)   ← 期望横滚角速率
├── pitch: q_sp (rad/s)   ← 期望俯仰角速率
├── yaw:   r_sp (rad/s)   ← 期望偏航角速率
└── thrust_body[0]: δ_T   ← 直传油门（来自位置控制器）
```

---

## 6. 第四层：角速率控制

### 6.1 模块信息

| 属性 | 值 |
|------|-----|
| 模块名 | `fw_rate_control` (FixedwingRateControl) |
| 源文件 | `src/modules/fw_rate_control/FixedwingRateControl.cpp` |
| 运行频率 | 事件驱动（IMU 角速度更新触发），~250 Hz |
| 输入话题 | `vehicle_angular_velocity`, `vehicle_rates_setpoint`, `airspeed_validated` |
| 输出话题 | `vehicle_torque_setpoint`, `vehicle_thrust_setpoint` |

### 6.2 控制律详解

角速率控制器是**最内层**的控制回路，也是运行最快的：

```
                    ┌─────────────────────────────────────┐
                    │        角速率 PID 控制器              │
                    │                                     │
   p_sp, p    ──→   │  e = p_sp - p                       │
                    │  roll_acc = Kp*e + Ki*∫e + Kd*de    │
                    │  roll_ff  = FF * p_sp               │
                    │                                     │
                    │  roll_u = (roll_acc * S²) + roll_ff │
                    │  τ_roll = clamp(roll_u + TRIM, -1,1)│
                    │                                     │
   q_sp, q    ──→   │  (同理 pitch 通道)                   │
                    │  τ_pitch = clamp(pitch_u + TRIM,-1,1)│
                    │                                     │
   r_sp, r    ──→   │  (同理 yaw 通道)                    │
                    │  τ_yaw = clamp(yaw_u + TRIM, -1, 1)│
                    └─────────────────────────────────────┘
```

其中 `S` 是空速缩放因子 (airspeed scaling)：
```
S = V_trim / V_indicated
```
当空速等于巡航速度时 S=1.0。低速时 S>1 增大控制量，高速时 S<1 减小控制量。

### 6.3 横滚通道完整公式

```python
# 角速率误差
e_p = p_sp - p                         # rad/s

# PID 控制器
roll_acc_sp = FW_RR_P * e_p            # P 项
            + FW_RR_I * ∫e_p dt        # I 项（带 anti-windup）
            + FW_RR_D * d(e_p)/dt      # D 项

# 前馈
roll_ff = FW_RR_FF * S * p_sp          # 前馈项

# 总控制量
roll_u = roll_acc_sp * S² + roll_ff    # 空速缩放

# 加上配平并限幅
τ_roll = clamp(roll_u + TRIM_ROLL, -1.0, +1.0)
```

### 6.4 俯仰通道完整公式

```python
e_q = q_sp - q

pitch_acc_sp = FW_PR_P * e_q + FW_PR_I * ∫e_q dt + FW_PR_D * d(e_q)/dt
pitch_ff = FW_PR_FF * S * q_sp
pitch_u = pitch_acc_sp * S² + pitch_ff

τ_pitch = clamp(pitch_u + TRIM_PITCH, -1.0, +1.0)
# 注：TRIM_PITCH = -0.15（来自机架文件）
```

### 6.5 偏航通道完整公式

```python
e_r = r_sp - r

yaw_acc_sp = FW_YR_P * e_r + FW_YR_I * ∫e_r dt + FW_YR_D * d(e_r)/dt
yaw_ff = FW_YR_FF * S * r_sp
yaw_u = yaw_acc_sp * S² + yaw_ff

# 横滚-偏航耦合前馈（补偿不利偏航）
τ_yaw = clamp(yaw_u + FW_RLL_TO_YAW_FF * τ_roll + TRIM_YAW, -1.0, +1.0)
```

### 6.6 关键参数

| 参数 | 值 | 说明 |
|------|-----|------|
| **横滚** | | |
| `FW_RR_P` | 0.3 | 横滚角速率比例增益 |
| `FW_RR_I` | 0.5 | 横滚角速率积分增益 |
| `FW_RR_FF` | 0.5 | 横滚角速率前馈 |
| **俯仰** | | |
| `FW_PR_P` | 0.9 | 俯仰角速率比例增益 |
| `FW_PR_I` | 0.5 | 俯仰角速率积分增益 |
| `FW_PR_FF` | 0.5 | 俯仰角速率前馈 |
| **偏航** | | |
| `FW_YR_P` | 0.6 | 偏航角速率比例增益 |
| `FW_YR_I` | 0.5 | 偏航角速率积分增益 |
| `FW_YR_FF` | 0.5 | 偏航角速率前馈 |
| **配平** | | |
| `TRIM_PITCH` | -0.15 | 俯仰配平（补偿安装偏差/重心） |

### 6.7 输出

```
vehicle_torque_setpoint_s:
├── xyz[0]: τ_roll   [-1, +1]  ← 横滚力矩指令
├── xyz[1]: τ_pitch  [-1, +1]  ← 俯仰力矩指令
└── xyz[2]: τ_yaw    [-1, +1]  ← 偏航力矩指令

vehicle_thrust_setpoint_s:
└── xyz[0]: δ_T      [0, 1]    ← 油门指令（直传）
```

---

## 7. 第五层：控制分配

### 7.1 模块信息

| 属性 | 值 |
|------|-----|
| 模块名 | `control_allocator` (ControlAllocator) |
| 源文件 | `src/modules/control_allocator/ControlAllocator.cpp` |
| 运行频率 | 事件驱动（力矩/推力设定点更新触发） |
| 输入话题 | `vehicle_torque_setpoint`, `vehicle_thrust_setpoint` |
| 输出话题 | `actuator_servos`, `actuator_motors` |

### 7.2 效率矩阵

控制分配器将 3 轴力矩指令映射到 3 个舵面：

```
┌         ┐   ┌                              ┐   ┌          ┐
│ τ_roll  │   │  TRQ_R_0   TRQ_R_1   TRQ_R_2 │   │ Servo_0  │
│ τ_pitch │ = │  TRQ_P_0   TRQ_P_1   TRQ_P_2 │ × │ Servo_1  │
│ τ_yaw   │   │  TRQ_Y_0   TRQ_Y_1   TRQ_Y_2 │   │ Servo_2  │
└         ┘   └                              ┘   └          ┘
```

**本项目的效率矩阵（3×3）**：

```
         Servo_0          Servo_1        Servo_2
         (左Elevon)       (中央升降舵)    (右Elevon)
         CS_TYPE=6        CS_TYPE=3      CS_TYPE=5
         (RightElevon)    (Elevator)     (LeftElevon)
         
Roll:      +0.5             0.0           -0.5
Pitch:     +0.5             +1.0          +0.5
Yaw:        0.0              0.0           0.0
```

### 7.3 求逆计算

给定力矩指令 `[τ_roll, τ_pitch, τ_yaw]`，求解舵面行程：

```python
# 伪逆求解（实际使用 CA 内部迭代求解器）
# 简化为显式公式：

Servo_0 = +0.5 × τ_roll + 0.5 × τ_pitch    # 左Elevon: 混合横滚+俯仰
Servo_1 = +1.0 × τ_pitch                     # 中央升降舵: 纯俯仰
Servo_2 = -0.5 × τ_roll + 0.5 × τ_pitch    # 右Elevon: 混合横滚+俯仰
```

**Elevon 混合逻辑**：
- 俯仰指令：两个 elevon 同向偏转（同时上或同时下）
- 横滚指令：两个 elevon 差动偏转（一上一下）
- 组合：两个分量叠加

### 7.4 输出缩放

```python
# 控制分配器输出：[-1, +1] 归一化
# 发布到 actuator_servos 时缩放到 [0, 1000]：
actuator_servos.control[i] = servo_value × 500 + 500

# 例：Servo_1 = +0.6 → 0.6 × 500 + 500 = 800
# 例：Servo_0 = -0.3 → -0.3 × 500 + 500 = 350
```

### 7.5 电机分配

```
Motor_0 (左从机):  actuator_motors[0] = δ_T   (直接映射)
Motor_1 (中央):    actuator_motors[1] = δ_T
Motor_2 (右从机):  actuator_motors[2] = δ_T
```

三台电机同步接收相同油门指令。

---

## 8. 第六层：从机铰链修正控制（自定义）

### 8.1 模块信息

| 属性 | 值 |
|------|-----|
| 模块名 | `chainwing_slave` (ChainwingSlave) |
| 源文件 | `src/modules/chainwing_slave/ChainwingSlave.cpp` |
| 头文件 | `src/modules/chainwing_slave/ChainwingSlave.hpp` |
| 参数文件 | `src/modules/chainwing_slave/chainwing_slave_params.c` |
| 运行频率 | **50 Hz**（`ScheduleOnInterval(20000_us)`） |
| 工作队列 | `lp_default`（低优先级默认队列） |
| 输入话题 | `vehicle_angular_velocity`, `vehicle_attitude`, `debug_array` |
| 输出话题 | `chainwing_hinge_status`, `debug_array` |
| 消息定义 | `msg/ChainwingHingeStatus.msg` |
| 启动方式 | 机架文件自动启动：`chainwing_slave start` |

### 8.2 功能概述

当三个机体通过铰链连接时，气流扰动/质量不对称可能导致从机相对主机产生**绕 X 轴的相对滚转**。`chainwing_slave` 模块负责：

1. **估计**：通过 IMU 积分 + 互补滤波估计铰链相对滚转角
2. **修正**：计算 PD 修正量叠加到从机 elevon 舵面
3. **通信**（可选）：通过 MAVLink 向主机报告状态/接收指令

### 8.3 Run() 主循环流程图

```
Run() @ 50 Hz
    │
    ├─ 1. 参数更新检查
    │     if (_parameter_update_sub.updated()) → updateParams()
    │
    ├─ 2. 使能检查
    │     if (CW_SLV_EN == 0) → return  ← 模块空转，不产生任何输出
    │
    ├─ 3. 时间步长计算
    │     dt = constrain((now - _last_run) × 1e-6, 0.001, 0.1)
    │     首次运行时 _last_run=0 → 初始化并 return
    │
    ├─ 4. 铰链角度估计
    │     updateHingeEstimate(dt)  → 详见 §8.4
    │
    ├─ 5. PD 修正计算
    │     trim_left  = computeTrim(_hinge_angle_left,  _hinge_rate_left)
    │     trim_right = computeTrim(_hinge_angle_right, _hinge_rate_right)
    │                              → 详见 §8.5
    │
    ├─ 6. 发布铰链状态 (uORB)
    │     chainwing_hinge_status_s:
    │     ├── hinge_angle_left   (rad)
    │     ├── hinge_angle_right  (rad)
    │     ├── hinge_rate_left    (rad/s)
    │     ├── hinge_rate_right   (rad/s)
    │     ├── trim_left          (归一化 [-0.3, +0.3])
    │     ├── trim_right         (归一化 [-0.3, +0.3])
    │     └── data_valid         (bool)
    │
    └─ 7. MAVLink 通信（如果 CW_SLV_COMM_EN=1）
          publishDebugArray()      → 发送铰链状态给主机
          processMasterCommands()  → 接收主机指令
```

### 8.4 铰链角度估计算法 (updateHingeEstimate)

```
updateHingeEstimate(dt)
    │
    ├─ 1. 读取 IMU 角速度
    │     angular_vel.xyz[0] = roll_rate (绕X轴)  ← 铰链轴
    │     angular_vel.xyz[1] = pitch_rate (绕Y轴)
    │     angular_vel.xyz[2] = yaw_rate (绕Z轴)
    │
    ├─ 2. 读取姿态四元数
    │     attitude.q → Eulerf → euler.phi() (横滚角)
    │
    ├─ 3. 初始化参考横滚角（首次）
    │     _roll_ref = euler.phi()    ← 记为"零偏角"基准
    │     PX4_INFO("Slave reference roll initialized: %.2f deg")
    │
    ├─ 4. 低通滤波角速率
    │     alpha = dt / (dt + 1/(2π × CW_SLV_LP_FREQ))
    │     _hinge_rate_left  = (1-alpha) × prev + alpha × roll_rate
    │     _hinge_rate_right = (1-alpha) × prev + alpha × (-roll_rate)
    │                                              ↑ 右从机取反（对称）
    │
    ├─ 5. 积分角度（带衰减）
    │     tau_decay = 2.0 s
    │     decay = exp(-dt / tau_decay)    ← 防止积分漂移
    │     _hinge_angle_left  = decay × (prev + rate × dt)
    │     _hinge_angle_right = decay × (prev + rate × dt)
    │
    └─ 6. 互补滤波修正（姿态反馈）
          roll_error = euler.phi() - _roll_ref
          cf_alpha = 0.02    ← 2% 权重给姿态，98% 给积分
          _hinge_angle_left  = 0.98 × integrated + 0.02 × roll_error
          _hinge_angle_right = 0.98 × integrated + 0.02 × (-roll_error)
```

**设计解读**：
- **低通滤波** (10 Hz)：滤除 IMU 高频噪声，保留铰链运动信号
- **指数衰减** (τ=2s)：即使积分有误差，2 秒后误差衰减到 37%
- **互补滤波** (cf=0.02)：用绝对姿态缓慢修正长期漂移
- **左右取反**：左从机绕 X 轴正向旋转 = 铰链正偏角；右从机正好相反

### 8.5 PD 修正控制律 (computeTrim)

```python
δ_trim = Kp × θ_hinge + Kd × θ̇_hinge
δ_trim = clamp(δ_trim, -CW_SLV_TRIM_MAX, +CW_SLV_TRIM_MAX)
```

**物理含义**：
- **Kp × θ**：比例项——铰链偏角越大，修正越大（弹簧效应）
- **Kd × θ̇**：微分项——铰链转动越快，修正越大（阻尼效应）

**数值示例**（铰链偏角 5° = 0.087 rad，角速率 0.1 rad/s）：
```
δ_trim = 1.5 × 0.087 + 0.2 × 0.1
       = 0.131 + 0.02
       = 0.151 (归一化)
       → 15.1% elevon 行程
```

### 8.6 参数表

| 参数 | 默认值 | 范围 | 说明 |
|------|--------|------|------|
| `CW_SLV_EN` | 1 | 0/1 | 模块使能（0=空转，铰链修正不生效） |
| `CW_SLV_KP` | 1.5 | [0, 5] | PD 比例增益 |
| `CW_SLV_KD` | 0.2 | [0, 2] | PD 微分增益 |
| `CW_SLV_TRIM_MAX` | 0.3 | [0, 1] | 最大修正量（归一化，30%行程） |
| `CW_SLV_LP_FREQ` | 10 Hz | [0, 50] | 角速率低通滤波截止频率 |
| `CW_SLV_COMM_EN` | 0 | 0/1 | MAVLink 通信使能 |

### 8.7 MAVLink 通信协议

**从机 → 主机**（DEBUG_FLOAT_ARRAY, id=42, name="CW_HINGE"）：

| 字段 | 含义 | 单位 |
|------|------|------|
| data[0] | hinge_angle_left | rad |
| data[1] | hinge_angle_right | rad |
| data[2] | hinge_rate_left | rad/s |
| data[3] | hinge_rate_right | rad/s |
| data[4] | trim_left | 归一化 |
| data[5] | trim_right | 归一化 |
| data[6] | data_valid | 1.0 或 0.0 |

**主机 → 从机**（DEBUG_FLOAT_ARRAY, id=43, name="CW_CMD"）：

| 字段 | 含义 | 范围 |
|------|------|------|
| data[0] | pitch 指令 | [-1, +1] |
| data[1] | throttle 指令 | [0, +1] |
| data[2] | roll 指令 | [-1, +1] |

**超时保护**：500 ms 未收到主机指令 → `_master_cmd_valid = false`

---

## 9. 第七层：Gazebo仿真接口

### 9.1 模块信息

| 属性 | 值 |
|------|-----|
| 模块名 | GZMixingInterfaceServo（gz_bridge 的一部分） |
| 源文件 | `src/modules/simulation/gz_bridge/GZMixingInterfaceServo.cpp` |
| 运行频率 | ScheduledWorkItem（由 mixing_output 驱动） |
| 输入话题 | `actuator_servos`, `chainwing_hinge_status` |
| 输出 | Gazebo topics: `/model/chainwing_3body/servo_{0,1,2}` |

### 9.2 舵面输出计算

```
对每个舵面 i = 0, 1, 2:

    1. 从控制分配器读取: outputs[i] ∈ [0, 1000]
    
    2. 归一化到 [-1, +1]:
       output = (outputs[i] - 500) / 500.0
    
    3. 叠加铰链修正 (仅对从机舵面):
       if (hinge_valid):
           if (i == 0):  output += trim_left     ← 左从机 Elevon
           if (i == 2):  output += trim_right    ← 右从机 Elevon
           output = clamp(output, -1.0, +1.0)
       
       注意: servo_1 (中央升降舵) 不受铰链修正影响
    
    4. 发布到 Gazebo:
       servo_pub[i].Publish(output)
```

### 9.3 安全保护机制

铰链修正有**双重安全保护**：

```
hinge_valid = _hinge_status_sub.copy(&hinge_status)  ← 第一重: copy() 成功？
           && hinge_status.data_valid                  ← 第二重: 数据有效？
```

失效条件（任一即跳过修正）：
- `chainwing_slave` 模块未运行 → copy() 返回 false
- `CW_SLV_EN=0` → 不发布消息 → copy() 返回 false
- 参考姿态未初始化 → `data_valid = false`

---

## 10. 第八层：Gazebo物理仿真

### 10.1 模型结构

**三个刚体**：

| 链接 | 质量 | 位置 | 惯量 (Ixx, Iyy, Izz) |
|------|------|------|----------------------|
| base_link | 1.9 kg | Y=0 | 0.0894, 0.144, 0.162 kg·m² |
| left_unit | 1.9 kg | Y=-1.2 m | 同上 |
| right_unit | 1.9 kg | Y=+1.2 m | 同上 |

### 10.2 铰链关节

| 属性 | 值 |
|------|-----|
| 类型 | Revolute（旋转关节） |
| 旋转轴 | X（机体前方，即滚转轴） |
| 位置 | Y = ±0.60 m（翼尖） |
| 弹簧刚度 | 200 N·m/rad |
| 阻尼 | 12 N·m·s/rad |
| 限位 | ±0.262 rad（±15°） |
| 参考位置 | 0 rad（共面） |
| ODE求解 | implicit_spring_damper = 1 |

**铰链动力学方程**：
```
I_unit × θ̈ + c × θ̇ + k × θ = M_aero + M_trim
```

其中：
- `I_unit ≈ 0.0894 + 1.9×0.6² = 0.773 kg·m²`（平行轴定理）
- `c = 12 N·m·s/rad`（阻尼系数）
- `k = 200 N·m/rad`（弹簧刚度）
- `M_aero`：气动干扰力矩
- `M_trim`：elevon 修正产生的气动力矩

**自然频率**：ωn = √(k/I) = √(200/0.773) = 16.1 rad/s = **2.56 Hz**

**阻尼比**：ζ = c / (2√(kI)) = 12 / (2√(200×0.773)) = 0.48

### 10.3 舵面关节

| 舵面 | 父链接 | 旋转轴 | 限位 | 阻尼 | 用途 |
|------|--------|--------|------|------|------|
| servo_0 | left_unit | Y | ±30° | 1.0 | 左从机 elevon |
| servo_1 | base_link | Y | ±30° | 1.0 | 中央升降舵 |
| servo_2 | right_unit | Y | ±30° | 1.0 | 右从机 elevon |

每个舵面由 `JointPositionController` 插件驱动，P 增益 = 10.0。

### 10.4 气动力模型 (LiftDrag 插件)

每个翼面和尾翼使用 `LiftDragPlugin` 计算气动力：

**升力公式**：
```
C_L = C_La × (α - α_0) + ΔC_L_servo
ΔC_L_servo = control_joint_rad_to_cl × δ_servo

L = ½ρV² × S × C_L
```

**翼面参数**：

| 翼面 | 面积 | C_La | α_0 | α_stall | rad_to_cl | 所属链接 |
|------|------|------|------|---------|-----------|---------|
| 左翼 | 0.36 m² | 5.25 | -0.05 | 0.227 | -0.3 (servo_0) | left_unit |
| 中翼 | 0.36 m² | 5.25 | -0.05 | 0.227 | 无控制 | base_link |
| 右翼 | 0.36 m² | 5.25 | -0.05 | 0.227 | -0.3 (servo_2) | right_unit |
| 左水平尾 | 0.12 m² | 5.25 | -0.2 | 0.34 | -4.0 (servo_0) | left_unit |
| 中水平尾 | 0.18 m² | 5.25 | -0.2 | 0.34 | -4.0 (servo_1) | base_link |
| 右水平尾 | 0.12 m² | 5.25 | -0.2 | 0.34 | -4.0 (servo_2) | right_unit |

### 10.5 电机模型

| 电机 | 链接 | 旋转方向 | 推力常数 | 最大转速 |
|------|------|---------|---------|---------|
| Motor_0 | left_unit | CCW | 1.5e-05 | 1000 rad/s |
| Motor_1 | base_link | CW | 1.5e-05 | 1000 rad/s |
| Motor_2 | right_unit | CCW | 1.5e-05 | 1000 rad/s |

电机时间常数：加速 0.0125s，减速 0.025s。

### 10.6 传感器

每个链接有独立的 IMU 传感器插件，以 250 Hz 发布数据到 GZ Bridge。

---

## 11. 完整数据流追踪

### 11.1 正向数据流（指令 → 执行）

```
mission_item (航点坐标)
    │
    │ [navigator]
    ▼
position_setpoint_triplet (lat/lon/alt 三元组)
    │
    │ [fw_pos_control]  TECS + NPFG
    ▼
vehicle_attitude_setpoint (φ_sp, θ_sp, ψ_sp, δ_T)
    │
    │ [fw_att_control]  姿态 PID
    ▼
vehicle_rates_setpoint (p_sp, q_sp, r_sp, δ_T)
    │
    │ [fw_rate_control]  角速率 PID + 空速缩放
    ▼
vehicle_torque_setpoint (τ_roll, τ_pitch, τ_yaw)  ← 归一化 [-1, +1]
vehicle_thrust_setpoint (δ_T)                      ← 归一化 [0, 1]
    │
    │ [control_allocator]  效率矩阵求逆
    ▼
actuator_servos (servo_0..2)  ← 缩放到 [0, 1000]
actuator_motors (motor_0..2)  ← 缩放到 [0, 1000]
    │
    │ [GZMixingInterfaceServo]  + 铰链修正
    ▼
/model/chainwing_3body/servo_{0,1,2}  ← Gazebo 话题, [-1, +1]
/model/chainwing_3body/command/motor_speed  ← Gazebo 话题
    │
    │ [Gazebo JointPositionController]  P=10.0
    ▼
舵面实际偏转角 + 电机转速
    │
    │ [Gazebo LiftDragPlugin]  气动力计算
    ▼
刚体上的力和力矩
    │
    │ [Gazebo ODE Physics]  牛顿-欧拉积分
    ▼
刚体新位姿 + 速度 + 角速度
```

### 11.2 反馈数据流（传感器 → 估计器）

```
Gazebo IMU 传感器 (250 Hz)
    │
    │ [gz_bridge]  坐标系转换 (ENU→NED, FLU→FRD)
    ▼
sensor_accel + sensor_gyro (uORB)
    │
    │ [EKF2]  扩展卡尔曼滤波
    ▼
vehicle_attitude (四元数)          ──→ fw_att_control (姿态反馈)
vehicle_angular_velocity (p,q,r)  ──→ fw_rate_control (角速率反馈)
vehicle_local_position (x,y,z)    ──→ fw_pos_control (位置反馈)
                                  ──→ chainwing_slave (铰链估计)
```

### 11.3 铰链修正数据流

```
vehicle_angular_velocity ──→ chainwing_slave ──→ chainwing_hinge_status
vehicle_attitude        ──→     (50 Hz)     ──→     (uORB)
                                                      │
                                                      ▼
                                               GZMixingInterfaceServo
                                                      │
                                    servo_0 += trim_left
                                    servo_2 += trim_right
                                                      │
                                                      ▼
                                               Gazebo servo topics
```

---

## 12. 系统时序与频率分析

### 12.1 各模块运行频率

```
                频率        触发方式            延迟
                ────        ────────            ────
Gazebo Physics  1000 Hz     固定步长 (1ms)       -
IMU 传感器       250 Hz     Gazebo 插件           ~1ms
EKF2            ~250 Hz     IMU 数据到达          ~2ms
fw_rate_control ~250 Hz     angular_velocity 更新  ~1ms
fw_att_control  ~50 Hz      attitude 更新          ~1ms
fw_pos_control  ~50 Hz      local_position 更新    ~1ms
control_allocator ~50 Hz    torque/thrust 更新     ~1ms
chainwing_slave  50 Hz      ScheduleOnInterval     ~1ms
GZ servo output  ~50 Hz     actuator_servos 更新   ~1ms
navigator       ~1-10 Hz    事件驱动              ~10ms
```

### 12.2 控制回路时延分析

```
从传感器到舵面输出的总延迟：

IMU采样 → EKF2 → rate_control → allocator → GZ_servo → JointCtrl → 气动力
  0ms      2ms      1ms           1ms         1ms        ~5ms      ~5ms
  
总延迟 ≈ 15 ms (在 1000 Hz 物理步长下)

相位裕度分析：
- 角速率回路带宽 ~10 rad/s (1.6 Hz)
- 15ms 延迟在 1.6 Hz 处相位滞后 = 360° × 0.015 × 1.6 = 8.6°
- 相位裕度充足（典型要求 > 30°）
```

### 12.3 铰链修正频率匹配

```
铰链自然频率:  ωn = 2.56 Hz  (16.1 rad/s)
PD控制器频率:  50 Hz (采样定理: 需 > 2×ωn = 5.1 Hz ✓)
低通滤波:      10 Hz > ωn = 2.56 Hz ✓
姿态控制带宽:  ~1.5 Hz < ωn = 2.56 Hz ✓ (不耦合)
```

---

## 13. 数值计算示例

### 13.1 场景：巡航平飞转弯 20°

**第一步：位置控制器**
```
输入: 航线要求右转
NPFG 输出: φ_sp = +20° (右倾)
TECS 输出: θ_sp = 4° (补偿转弯升力损失), δ_T = 0.62
```

**第二步：姿态控制器**
```
当前姿态: φ = 0°, θ = 2° (配平状态)

横滚: p_sp = (20° - 0°) / 0.5s = 40°/s → clamp(30°/s) = 0.524 rad/s
俯仰: q_sp = (4° - 2°) / 0.5s = 4°/s = 0.070 rad/s
偏航: r_sp ≈ 0 + 协调转弯补偿
```

**第三步：角速率控制器**（假设启动初始 p=0, q=0）
```
横滚通道 (S=1.0, V=20m/s=V_trim):
  e_p = 0.524 - 0 = 0.524 rad/s
  roll_acc = 0.3 × 0.524 = 0.157
  roll_ff  = 0.5 × 1.0 × 0.524 = 0.262
  roll_u   = 0.157 × 1.0 + 0.262 = 0.419
  τ_roll   = clamp(0.419 + 0, -1, 1) = 0.419

俯仰通道:
  e_q = 0.070 - 0 = 0.070 rad/s
  pitch_acc = 0.9 × 0.070 = 0.063
  pitch_ff  = 0.5 × 1.0 × 0.070 = 0.035
  pitch_u   = 0.063 × 1.0 + 0.035 = 0.098
  τ_pitch   = clamp(0.098 + (-0.15), -1, 1) = -0.052
```

**第四步：控制分配**
```
Servo_0 (左Elevon)  = 0.5 × 0.419 + 0.5 × (-0.052) = 0.184
Servo_1 (中央升降舵) = 1.0 × (-0.052) = -0.052
Servo_2 (右Elevon)  = -0.5 × 0.419 + 0.5 × (-0.052) = -0.236

缩放到 [0, 1000]:
Servo_0 = 0.184 × 500 + 500 = 592
Servo_1 = -0.052 × 500 + 500 = 474
Servo_2 = -0.236 × 500 + 500 = 382
```

**第五步：铰链修正**（假设铰链偏角为 0°）
```
hinge_valid = true, trim_left = 0, trim_right = 0

最终 Gazebo 输出:
Servo_0 = (592 - 500)/500 + 0 = +0.184   ← 左Elevon上偏
Servo_1 = (474 - 500)/500     = -0.052   ← 中央升降舵微下偏
Servo_2 = (382 - 500)/500 + 0 = -0.236   ← 右Elevon下偏

→ 左Elevon上+右Elevon下 = 右横滚力矩 ✓
→ 三面均微下偏 = 微俯仰 (因为 TRIM_PITCH = -0.15) ✓
```

### 13.2 场景：铰链偏角 5° 时的修正量

```
假设: 左从机绕铰链正向偏转 5° (0.087 rad)
      铰链角速率 0.1 rad/s

PD 修正:
  trim_left = 1.5 × 0.087 + 0.2 × 0.1
            = 0.131 + 0.020
            = 0.151 (15.1% elevon 行程)

  trim_right = 1.5 × 0 + 0.2 × 0 = 0 (右侧无偏转)

Servo_0 (左Elevon) 最终输出:
  = CA输出 + 0.151
  → 左Elevon 额外上偏 0.151 × 30° = 4.53°

产生的修正气动力矩:
  左尾翼: ΔC_L = -4.0 × (servo_0_angle)
  → 额外升力变化 → 对铰链产生恢复力矩
```

---

## 14. 铰链修正专题：阵风干扰完整响应

### 14.1 场景描述

在 20 m/s 巡航中，一阵侧风使左从机相对主机产生 +5° 铰链偏角。

### 14.2 时间线

```
T = 0 ms    阵风到达，左从机开始绕铰链旋转
            ├─ 铰链弹簧产生恢复力矩: τ_spring = -200 × θ
            └─ 铰链阻尼产生阻力: τ_damp = -12 × θ̇

T = 1-10 ms Gazebo 物理引擎积分铰链动力学
            ├─ 左从机 IMU 感受到 roll rate 变化
            └─ IMU 数据通过 gz_bridge 发送到 PX4

T = 10-15 ms EKF2 更新 vehicle_angular_velocity
             ├─ roll_rate (angular_vel.xyz[0]) 显著增大
             └─ vehicle_attitude 的 φ 开始变化

T = 20 ms   chainwing_slave.Run() 触发（50 Hz 周期）
            │
            ├─ updateHingeEstimate():
            │   ├─ 低通滤波: rate_filtered = 0.02 × roll_rate + 0.98 × 0
            │   │  (首个周期滤波器尚未建立，响应滞后)
            │   ├─ 积分: angle += rate × dt = filtered_rate × 0.02
            │   └─ 互补滤波: angle = 0.98 × integrated + 0.02 × (φ-φ_ref)
            │
            └─ computeTrim():
                ├─ trim_left = 1.5 × angle + 0.2 × rate
                ├─ (此时角度估计还在爬升中，约 1-2°)
                └─ trim_left ≈ 0.04 (4% 行程)

T = 20-25 ms 控制分配 + GZMixingInterfaceServo
             ├─ Servo_0 原始值 + 0.04 修正
             └─ Servo_0 额外上偏 → 增大左尾翼升力

T = 25-40 ms Gazebo 气动 + 铰链物理
             ├─ 修正气动力开始作用
             ├─ 铰链弹簧也在恢复
             └─ 两者叠加加速恢复

T = 40 ms   chainwing_slave.Run() 第2次触发
            ├─ 角度估计更准确: ~3-4°
            ├─ trim_left ≈ 0.08 (8% 行程)
            └─ 修正更强

T = 60 ms   第3次触发
            ├─ 角度开始回落: ~2°
            └─ trim_left ≈ 0.06

T = 100 ms  第5次触发
            ├─ 角度回到 ~0.5°
            └─ trim_left ≈ 0.02

T = 200 ms  ~第10次触发
            ├─ 角度接近 0°
            └─ trim_left → 0

T = 500 ms  积分衰减 + 互补滤波
            ├─ 残余估计 → 0
            └─ 系统回到平衡
```

### 14.3 恢复过程分析

在弹簧+阻尼+主动修正的共同作用下：

| 恢复机制 | 力矩贡献 | 时间尺度 |
|---------|---------|---------|
| 弹簧 (k=200) | 200 × 0.087 = 17.4 N·m | 自然周期 ~0.4s |
| 阻尼 (c=12) | 12 × ω | 与弹簧同步 |
| PD修正 (Kp=1.5) | 间接通过气动力 ~6.6 N·m | 50 Hz 更新 |
| **总恢复** | 弹簧 62% + 控制器 38% | **~0.3-0.5 秒** |

---

## 15. 参数速查表

### 15.1 姿态控制参数

| 参数 | 值 | 通道 | 说明 |
|------|-----|------|------|
| FW_R_TC | 0.5 | Roll | 横滚时间常数 (s) |
| FW_P_TC | 0.5 | Pitch | 俯仰时间常数 (s) |
| FW_R_RMAX | 30 | Roll | 最大横滚速率 (°/s) |
| FW_P_RMAX_POS | 25 | Pitch | 最大抬头速率 (°/s) |
| FW_P_RMAX_NEG | 25 | Pitch | 最大低头速率 (°/s) |
| FW_Y_RMAX | 15 | Yaw | 最大偏航速率 (°/s) |

### 15.2 角速率控制参数

| 参数 | 值 | 通道 | 说明 |
|------|-----|------|------|
| FW_RR_P | 0.3 | Roll | P 增益 |
| FW_RR_I | 0.5 | Roll | I 增益 |
| FW_RR_FF | 0.5 | Roll | 前馈 |
| FW_PR_P | 0.9 | Pitch | P 增益 |
| FW_PR_I | 0.5 | Pitch | I 增益 |
| FW_PR_FF | 0.5 | Pitch | 前馈 |
| FW_YR_P | 0.6 | Yaw | P 增益 |
| FW_YR_I | 0.5 | Yaw | I 增益 |
| FW_YR_FF | 0.5 | Yaw | 前馈 |

### 15.3 铰链修正参数

| 参数 | 值 | 说明 |
|------|-----|------|
| CW_SLV_EN | 1 | 使能 |
| CW_SLV_KP | 1.5 | 比例增益 |
| CW_SLV_KD | 0.2 | 微分增益 |
| CW_SLV_TRIM_MAX | 0.3 | 最大修正量 |
| CW_SLV_LP_FREQ | 10 | 低通滤波 (Hz) |
| CW_SLV_COMM_EN | 0 | MAVLink使能 |

### 15.4 物理模型参数

| 参数 | 值 | 说明 |
|------|-----|------|
| k_hinge | 200 N·m/rad | 铰链刚度 |
| c_hinge | 12 N·m·s/rad | 铰链阻尼 |
| θ_limit | ±15° | 铰链限位 |
| m_unit | 1.9 kg | 单机质量 |
| I_xx | 0.0894 kg·m² | 滚转惯量 |
| S_wing | 0.36 m² | 翼面积 |

### 15.5 控制分配参数

| 参数 | 值 | 说明 |
|------|-----|------|
| CA_SV_CS_COUNT | 3 | 控制面数 |
| CA_SV_CS0_TYPE | 6 (RightElevon) | 左从机 |
| CA_SV_CS0_TRQ_R | 0.5 | 横滚效率 |
| CA_SV_CS0_TRQ_P | 0.5 | 俯仰效率 |
| CA_SV_CS1_TYPE | 3 (Elevator) | 中央 |
| CA_SV_CS1_TRQ_P | 1.0 | 俯仰效率 |
| CA_SV_CS2_TYPE | 5 (LeftElevon) | 右从机 |
| CA_SV_CS2_TRQ_R | -0.5 | 横滚效率 |
| CA_SV_CS2_TRQ_P | 0.5 | 俯仰效率 |

---

## 16. 源文件清单

### 16.1 自定义代码（本项目新增/修改）

| 文件 | 类型 | 说明 |
|------|------|------|
| `src/modules/chainwing_slave/ChainwingSlave.cpp` | 新增 | 从机控制器主逻辑 |
| `src/modules/chainwing_slave/ChainwingSlave.hpp` | 新增 | 类定义 + 成员变量 |
| `src/modules/chainwing_slave/chainwing_slave_params.c` | 新增 | 6个CW_SLV_*参数定义 |
| `src/modules/chainwing_slave/CMakeLists.txt` | 新增 | 构建配置 |
| `src/modules/chainwing_slave/Kconfig` | 新增 | Kconfig 菜单项 |
| `msg/ChainwingHingeStatus.msg` | 新增 | uORB 消息定义 |
| `src/modules/simulation/gz_bridge/GZMixingInterfaceServo.cpp` | 修改 | 叠加铰链修正逻辑 |
| `src/modules/simulation/gz_bridge/GZMixingInterfaceServo.hpp` | 修改 | 添加 hinge_status 订阅 |
| `src/modules/logger/logged_topics.cpp` | 修改 | 添加 chainwing_hinge_status 日志 |
| `Tools/simulation/gz/models/chainwing_3body/model.sdf` | 新增 | 三体模型 SDF |
| `ROMFS/.../4008_gz_chainwing_3body` | 新增 | 机架配置文件 |
| `boards/px4/sitl/default.px4board` | 修改 | 编译使能 |

### 16.2 PX4 原生模块（未修改，参与控制链路）

| 模块 | 源路径 | 功能 |
|------|--------|------|
| navigator | `src/modules/navigator/` | 任务规划/航点管理 |
| fw_pos_control | `src/modules/fw_pos_control/` | 位置控制 + TECS |
| fw_att_control | `src/modules/fw_att_control/` | 姿态控制 |
| fw_rate_control | `src/modules/fw_rate_control/` | 角速率控制 |
| control_allocator | `src/modules/control_allocator/` | 控制分配 |
| ekf2 | `src/modules/ekf2/` | 状态估计 |
| gz_bridge | `src/modules/simulation/gz_bridge/` | Gazebo 仿真接口 |
| mavlink | `src/modules/mavlink/` | MAVLink 通信 |

---

> **文档结束**  
> 本文档基于代码审计编写，所有公式、参数、数据流均可在对应源文件中验证。  
> 详细参数清单见 `CHAINWING_PARAMETER_REFERENCE.md`  
> 实现细节见 `CHAINWING_SLAVE_IMPLEMENTATION.md`  
> 通信协议见 `CHAINWING_COMMUNICATION_GUIDE.md`
