# Chain-Wing 三体无人机 硬件在环仿真(HIL) 完整控制流程文档

> **版本**: v1.0 | **日期**: 2026-03-23 | **基于**: PX4 v1.14 + Gazebo Harmonic  
> **读者**: 需要理解硬件在环控制链路的工程师/评审人员  
> **关联文档**: CHAINWING_CONTROL_FLOW.md (SITL控制流程), CHAINWING_COMMUNICATION_GUIDE.md (通信协议)

---

## 目录

1. [三种仿真模式对比](#1-三种仿真模式对比)
2. [HIL系统总体架构](#2-hil系统总体架构)
3. [启动序列详解](#3-启动序列详解)
4. [SIH物理引擎架构](#4-sih物理引擎架构)
5. [HIL完整控制链路（8层）](#5-hil完整控制链路8层)
6. [PWMSim执行器输出模块](#6-pwmsim执行器输出模块)
7. [MAVLink HIL消息协议](#7-mavlink-hil消息协议)
8. [从机铰链修正在HIL中的工作方式](#8-从机铰链修正在hil中的工作方式)
9. [硬件部署控制流（3台Pixhawk）](#9-硬件部署控制流3台pixhawk)
10. [控制频率与时序分析](#10-控制频率与时序分析)
11. [SITL vs HIL vs 硬件 数据流对比](#11-sitl-vs-hil-vs-硬件-数据流对比)
12. [HIL参数速查表](#12-hil参数速查表)
13. [数值计算示例](#13-数值计算示例)
14. [故障排除](#14-故障排除)
15. [源文件清单](#15-源文件清单)

---

## 1. 三种仿真模式对比

### 1.1 模式总览

| 特性 | SITL (SYS_HITL=0) | HITL (SYS_HITL=1) | SIH (SYS_HITL=2) |
|------|-------------------|-------------------|-------------------|
| **运行平台** | PC (Linux/macOS) | 真实飞控硬件 | 真实飞控硬件 |
| **物理仿真** | 外部 Gazebo | 外部仿真器 (JSBSim等) | **板载物理引擎** |
| **传感器来源** | GZ桥接 → uORB | MAVLink HIL_SENSOR | SIH内部生成 |
| **执行器输出** | GZ话题 | MAVLink HIL_ACTUATOR_CONTROLS | actuator_outputs_sim → SIH |
| **实时性** | 锁步(lockstep) | 实时 | **实时** |
| **铰链仿真** | GZ revolute joint | 需外部建模 | SIH 通用固定翼模型 |
| **chainwing_slave** | ✅ 完全工作 | ✅ 可工作 | ✅ 可工作 |
| **多实例** | ✅ 支持 (-i 0/1/2) | ❌ 需多飞控 | ❌ 需多飞控 |
| **适用阶段** | 算法开发 | 系统集成 | **快速硬件验证** |

### 1.2 SYS_HITL 参数定义

```
文件: src/lib/systemlib/system_params.c:84

SYS_HITL:
  -1 = 外部 HITL（真实飞行器仿真）
   0 = 禁用（默认，SITL模式）
   1 = HITL 启用（外部仿真器提供传感器）
   2 = SIH 启用（板载物理引擎，无需外部仿真器）
```

### 1.3 Chain-Wing 专用 SIH 机架

```
文件: ROMFS/px4fmu_common/init.d/airframes/1103_chainwing_sih.hil

机架 ID: 1103
类型: SIH 固定翼 (SYS_HITL=2)
特点:
  - 3个电机: 差速偏航控制 (Y=-1.2, 0, +1.2 m)
  - 3个舵面: 左升降副翼 + 中央升降舵 + 右升降副翼
  - SIH物理参数源自 MATLAB 模型
```

---

## 2. HIL系统总体架构

### 2.1 SIH模式架构框图

```
┌═══════════════════════════════════════════════════════════════════════┐
│                    Pixhawk 飞控硬件 (NuttX RTOS)                      │
│                                                                       │
│  ┌──────────────┐   ┌──────────────┐   ┌──────────────┐              │
│  │  navigator    │   │ fw_pos_ctrl  │   │ fw_att_ctrl  │              │
│  │  (任务规划)   │──→│  (位置控制)  │──→│  (姿态控制)  │              │
│  └──────────────┘   └──────────────┘   └──────────────┘              │
│                                              │                        │
│                                              ▼                        │
│                    ┌──────────────┐   ┌──────────────┐               │
│                    │ fw_rate_ctrl │   │control_alloc  │               │
│                    │ (角速率控制) │──→│ (控制分配)    │               │
│                    └──────────────┘   └──────────────┘               │
│                                              │                        │
│                                              ▼                        │
│                                    actuator_outputs                   │
│                                              │                        │
│                                              ▼                        │
│  ┌─────────────────────────────────────────────────┐                 │
│  │            PWMSim (pwm_out_sim -m hil)          │                 │
│  │  电机: [0,1] 归一化    舵面: [-1,1] 归一化      │                 │
│  │  发布: actuator_outputs_sim                     │                 │
│  └─────────────────┬───────────────────────────────┘                 │
│                    │                                                  │
│         ┌──────────┼──────────┐                                      │
│         ▼          ▼          ▼                                      │
│  ┌────────┐  ┌──────────┐  ┌───────────────────┐                    │
│  │MAVLink │  │   SIH    │  │ chainwing_slave   │                    │
│  │HIL_ACT │  │ 物理引擎 │  │  (铰链修正控制)   │                    │
│  │CONTROLS│  │ (250 Hz) │  │   (50 Hz)         │                    │
│  │(200 Hz)│  └────┬─────┘  └───────────────────┘                    │
│  └────────┘       │                                                  │
│                   ▼                                                  │
│  ┌─────────────────────────────────────────────────┐                 │
│  │          仿真传感器输出                          │                 │
│  │  sensor_baro_sim + sensor_mag_sim               │                 │
│  │  sensor_gps_sim  + IMU (SIH内部)                │                 │
│  └─────────────────┬───────────────────────────────┘                 │
│                    │                                                  │
│                    ▼                                                  │
│              ┌──────────┐                                            │
│              │   EKF2   │ ←── 传感器融合 → 姿态/位置估计             │
│              └──────────┘                                            │
│                                                                       │
└═══════════════════════════════════════════════════════════════════════┘
```

### 2.2 与 SITL 的关键区别

| 层级 | SITL | SIH (HIL) |
|------|------|-----------|
| **物理引擎** | Gazebo (外部进程) | SIH (板载NuttX线程) |
| **传感器** | GZBridge → uORB | SIH内部 → PX4Accel/Gyro |
| **执行器** | GZMixingInterfaceServo → GZ话题 | PWMSim → actuator_outputs_sim → SIH |
| **铰链物理** | GZ revolute joint (真实铰链动力学) | **无铰链物理** (通用FW模型) |
| **trim叠加** | GZMixingInterfaceServo (有) | **无** (PWMSim不做trim) |
| **lockstep** | ✅ 仿真时间与物理同步 | ❌ 实时运行 |

**重要**: SIH 模式使用通用固定翼气动模型，**不模拟铰链物理**。chainwing_slave 模块仍然运行，但铰链角度估计来自 IMU 数据（无真实铰链运动），因此 trim 修正效果有限。

---

## 3. 启动序列详解

### 3.1 rcS 条件分支 (行 337-385)

```bash
# 文件: ROMFS/px4fmu_common/init.d/rcS

if param greater SYS_HITL 0           # ← HIL/SIH 模式
then
    pwm_out_sim start -m hil           # 启动虚拟PWM输出（忽略安全锁）
    sensors start -h                   # 传感器以 HIL 模式启动（禁用ADC）
    commander start -h                 # 指挥官以 HIL 模式启动
    param set GPS_1_CONFIG 0           # 禁用真实 GPS

    if param compare SYS_HITL 2        # ← 仅 SIH 模式
    then
        simulator_sih start            # 启动板载物理引擎
        sensor_baro_sim start          # 启动仿真气压计
        sensor_mag_sim start           # 启动仿真磁力计
        sensor_gps_sim start           # 启动仿真 GPS
    fi

else                                   # ← 真实硬件 / SITL
    . ${R}etc/init.d/rc.sensors        # 启动真实传感器驱动
    sensors start
    commander start
    pwm_out start                      # 启动真实 PWM 输出
fi

# ── 以下对 HIL 和真实硬件都执行 ──

. ${R}etc/init.d/rc.vehicle_setup      # 加载机型配置（rc.fw_apps等）
navigator start                        # 任务导航
```

### 3.2 固定翼控制栈启动 (rc.fw_apps)

```bash
# 文件: ROMFS/px4fmu_common/init.d/rc.fw_apps
# HIL 和 真实硬件 启动完全相同的控制模块

ekf2 start &                           # 状态估计器 (250 Hz)
control_allocator start                # 控制分配器
fw_rate_control start                  # 角速率控制器 (250 Hz)
fw_att_control start                   # 姿态控制器 (250 Hz)
fw_pos_control start                   # 位置控制器 (50 Hz)
airspeed_selector start                # 空速选择器
fw_autotune_attitude_control start     # 自整定
land_detector start fixedwing          # 着陆检测
```

### 3.3 启动时序图

```
时间 →
0ms        100ms       200ms       500ms       1000ms
│          │           │           │           │
├─ rcS 开始
│  ├─ SYS_HITL 检查
│  ├─ pwm_out_sim start -m hil ────────────────→ [运行中]
│  ├─ sensors start -h ────────────────────────→ [运行中]
│  ├─ commander start -h ──────────────────────→ [运行中]
│  ├─ simulator_sih start (仅SIH) ────────────→ [250Hz运行中]
│  ├─ sensor_baro/mag/gps_sim start ──────────→ [运行中]
│  │
│  ├─ rc.vehicle_setup
│  │  └─ rc.fw_apps
│  │     ├─ ekf2 start ───────────────────────→ [250Hz运行中]
│  │     ├─ control_allocator start ──────────→ [运行中]
│  │     ├─ fw_rate_control start ────────────→ [250Hz运行中]
│  │     ├─ fw_att_control start ─────────────→ [250Hz运行中]
│  │     └─ fw_pos_control start ─────────────→ [50Hz运行中]
│  │
│  ├─ 机架文件加载 (1103_chainwing_sih.hil)
│  │  └─ chainwing_slave start ───────────────→ [50Hz运行中]
│  │
│  └─ navigator start ────────────────────────→ [运行中]
│
└─ 系统就绪，等待解锁
```

---

## 4. SIH物理引擎架构

### 4.1 SIH 运行循环

```
文件: src/modules/simulation/simulator_sih/sih.cpp

Sih::realtime_loop() — 在 NuttX 实时线程中运行
│
├─ 初始化: HRT定时器, 信号量, 频率配置
│  └─ 默认 250 Hz (可配置 200-2000 Hz)
│
└─ 主循环 (每 4ms = 250Hz):
   │
   ├─ px4_sem_wait(&_data_semaphore)  ← 等待定时器唤醒
   │
   └─ sensor_step():
      │
      ├─ ① 参数检查更新
      │
      ├─ ② read_motors(dt)
      │  └─ 订阅 actuator_outputs_sim (来自 PWMSim)
      │     ├─ 电机: 一阶滞后滤波 (τ = _T_TAU)
      │     └─ 舵面: 直接传递
      │     → 输出: _u[0]=副翼, _u[1]=升降舵, _u[2]=方向舵, _u[3]=油门
      │
      ├─ ③ generate_force_and_torques()
      │  └─ generate_fw_aerodynamics():
      │     ├─ 5个气动段: wing_l, wing_r, tailplane, fin, fuselage
      │     ├─ 每段计算: 速度→迎角→CL/CD/CM→力和力矩
      │     ├─ 推力: _T_B = (_T_MAX * _u[3], 0, 0) — 前向推力
      │     └─ 气动力矩 = Σ(段力矩) + 角阻尼
      │
      ├─ ④ equations_of_motion(dt)
      │  ├─ 刚体动力学 (欧拉前向积分):
      │  │  ├─ ṗ = v                    (位置积分)
      │  │  ├─ v̇ = (F_aero + F_thrust + F_gravity) / m
      │  │  ├─ ω̇ = I⁻¹ × (τ - ω × Iω) (角加速度)
      │  │  └─ q̇ = q ⊗ Δq(ω·dt)        (四元数更新)
      │  └─ 地面碰撞检测 + 约束
      │
      ├─ ⑤ reconstruct_sensors_signals(now)
      │  ├─ 加速度计: a_body + 噪声
      │  ├─ 陀螺仪: ω_body + 偏差 + 噪声
      │  └─ 发布: _px4_accel.update(), _px4_gyro.update()
      │
      ├─ ⑥ send_airspeed() (50 Hz)
      │
      └─ ⑦ publish_ground_truth(now)
         ├─ vehicle_angular_velocity_groundtruth
         ├─ vehicle_attitude_groundtruth
         ├─ vehicle_local_position_groundtruth
         └─ vehicle_global_position_groundtruth
```

### 4.2 SIH 固定翼气动模型

```
文件: src/modules/simulation/simulator_sih/aero.hpp

AeroSeg 类 — 平板气动模型:
├─ 全360° 迎角 升力/阻力/力矩曲线
├─ 展弦比 0.1666-6.0 支持
├─ 控制面偏转最大 70°
├─ 螺旋桨滑流建模 (动量理论)
└─ 失速角随襟翼偏转自适应

5个气动段配置 (1103_chainwing_sih.hil):
┌─────────────┬────────┬────────┬──────────┬──────────┐
│ 段名         │ 翼展   │ 弦长   │ 安装角   │ 控制面   │
├─────────────┼────────┼────────┼──────────┼──────────┤
│ wing_l      │ 0.43m  │ 0.21m  │ -4°      │ 副翼(_u[0])│
│ wing_r      │ 0.43m  │ 0.21m  │ -4°      │ 副翼(_u[0])│
│ tailplane   │ 0.30m  │ 0.10m  │ 0°       │ 升降舵(_u[1])│
│ fin         │ 0.25m  │ 0.18m  │ 0°       │ 方向舵(_u[2])│
│ fuselage    │ -      │ -      │ -        │ 阻力     │
└─────────────┴────────┴────────┴──────────┴──────────┘

注意: 这是通用FW模型，不包含三体铰链结构。
SIH 将整架飞机视为单个刚体。
```

### 4.3 SIH 物理参数 (1103_chainwing_sih.hil)

```
# 质量与惯量 (源自 MATLAB 模型)
SIH_MASS    = 1.0 kg        # 总质量 (3单元)
SIH_IXX     = 1.02 kg·m²   # 滚转惯量
SIH_IYY     = 0.164 kg·m²  # 俯仰惯量
SIH_IZZ     = 1.17 kg·m²   # 偏航惯量
SIH_IXZ     = 0.01 kg·m²   # 惯量积

# 推力
SIH_T_MAX   = 15.0 N       # 3电机最大推力 (3×5N)

# 阻力
SIH_KDV     = 0.5           # 速度阻力系数

# 飞行器类型
SIH_VEHICLE_TYPE = 1        # 固定翼
```

---

## 5. HIL完整控制链路（8层）

### 5.1 第1层: 任务规划 (navigator, 1-10 Hz)

与 SITL 完全相同，参见 CHAINWING_CONTROL_FLOW.md §3。

### 5.2 第2层: 位置控制 (fw_pos_control, 50 Hz)

与 SITL 完全相同。TECS + NPFG 算法不区分仿真模式。

### 5.3 第3层: 姿态控制 (fw_att_control, 250 Hz)

与 SITL 完全相同。输出角速率设定值。

### 5.4 第4层: 角速率控制 (fw_rate_control, 250 Hz)

与 SITL 完全相同。输出归一化力矩 τ_roll, τ_pitch, τ_yaw 和推力。

### 5.5 第5层: 控制分配 (control_allocator)

```
效率矩阵（HIL与SITL完全相同）:

                    τ_roll   τ_pitch   τ_yaw
Servo_0 (LeftElevon)  [ +0.5     +0.5      0.0 ]
Servo_1 (Elevator)    [  0.0     +1.0      0.0 ]
Servo_2 (RightElevon) [ -0.5     +0.5      0.0 ]

Motor_0 (Left)   位置 Y=-1.2m
Motor_1 (Center)  位置 Y= 0.0m
Motor_2 (Right)  位置 Y=+1.2m

输出: actuator_outputs (归一化 [-1,1] 舵面, [0,1] 电机)
```

### 5.6 第6层: PWMSim 执行器仿真 (详见§6)

**这是 HIL 与 SITL 的关键分歧点。**

SITL: actuator_outputs → GZMixingInterfaceServo → GZ话题 → Gazebo  
HIL:  actuator_outputs → **PWMSim** → actuator_outputs_sim → **SIH**

### 5.7 第7层: SIH 物理引擎 (250 Hz, 详见§4)

SIH 读取 actuator_outputs_sim，计算气动力/力矩，积分刚体运动方程，输出仿真传感器数据。

### 5.8 第8层: 传感器反馈 → EKF2

```
SIH 内部生成:
├─ PX4Accelerometer (250 Hz): 加速度 + 噪声
├─ PX4Gyroscope (250 Hz): 角速率 + 偏差 + 噪声
├─ sensor_baro_sim (50 Hz): 气压高度
├─ sensor_mag_sim (50 Hz): 磁场
└─ sensor_gps_sim (1-10 Hz): GPS 位置/速度

       ↓ (全部通过 uORB)

EKF2 (250 Hz):
├─ 融合 IMU + 气压 + 磁力计 + GPS
├─ 输出: vehicle_attitude, vehicle_local_position
└─ 反馈给姿态/位置控制器 → 闭环
```

---

## 6. PWMSim执行器输出模块

### 6.1 启动与配置

```cpp
// 文件: src/modules/simulation/pwm_out_sim/PWMSim.cpp

// 构造函数 (行 44-53)
PWMSim::PWMSim(bool hil_mode_enabled) {
    setAllDisarmedValues(PWM_SIM_DISARMED_MAGIC);   // 900
    setAllFailsafeValues(PWM_SIM_FAILSAFE_MAGIC);   // 900
    setAllMinValues(PWM_SIM_PWM_MIN_MAGIC);          // 1000
    setAllMaxValues(PWM_SIM_PWM_MAX_MAGIC);          // 2000
    setIgnoreLockdown(hil_mode_enabled);  // ← HIL模式忽略安全锁
}
```

### 6.2 输出缩放逻辑

```cpp
// updateOutputs() 行 61-98
// 电机 (非可逆): [1000, 2000] → [0.0, 1.0]
if (function >= Motor1 && function <= MotorMax && !is_reversible) {
    output[i] = (pwm - 1000) / 1000;  // 归一化到 [0,1]
}
// 舵面 (可逆): [1000, 2000] → [-1.0, +1.0]
else {
    output[i] = (pwm - 1500) / 500;   // 归一化到 [-1,1]
}

// 发布到 actuator_outputs_sim
_actuator_outputs_sim_pub.publish(actuator_outputs);
```

### 6.3 HIL 执行器映射 (HIL_ACT_FUNCx)

```
# 1103_chainwing_sih.hil 行 69-76

HIL_ACT_FUNC1 = 201  → Servo 0 (左升降副翼)
HIL_ACT_FUNC2 = 202  → Servo 1 (中央升降舵)
HIL_ACT_FUNC3 = 203  → Servo 2 (右升降副翼)
HIL_ACT_FUNC4 = 101  → Motor 0 (左电机)
HIL_ACT_FUNC5 = 102  → Motor 1 (中央电机)
HIL_ACT_FUNC6 = 103  → Motor 2 (右电机)
```

### 6.4 PWMSim vs GZMixingInterfaceServo

```
                    PWMSim (HIL)              GZMixingInterfaceServo (SITL)
                    ──────────────            ──────────────────────────────
输入               actuator_outputs           actuator_outputs
trim叠加           ❌ 无                      ✅ chainwing_hinge_status
输出               actuator_outputs_sim       GZ servo 话题
消费者             SIH / MAVLink              Gazebo 物理引擎
缩放               [0,1] 电机 / [-1,1] 舵面  [-1,1] 全部
```

**关键区别**: PWMSim **不叠加铰链trim修正**，因为它是通用模块。在 HIL 模式下，chainwing_slave 仍然运行并计算 trim，但 **没有消费者** 读取 chainwing_hinge_status 来应用 trim。

---

## 7. MAVLink HIL消息协议

### 7.1 HIL 消息流

```
┌────────────────────────────────────────────────────────────────┐
│                    PX4 飞控 → 外部                             │
│                                                                │
│  HIL_ACTUATOR_CONTROLS (200 Hz)                                │
│  ├─ msg.controls[0-15]: 16通道执行器输出                      │
│  ├─ msg.mode: MAV_MODE_FLAG 标志位                            │
│  │  ├─ CUSTOM_MODE_ENABLED                                    │
│  │  ├─ AUTO_ENABLED / MANUAL_INPUT_ENABLED                    │
│  │  ├─ STABILIZE_ENABLED                                      │
│  │  ├─ SAFETY_ARMED                                           │
│  │  ├─ HIL_ENABLED                                            │
│  │  └─ GUIDED_ENABLED                                         │
│  └─ msg.flags = 0                                              │
│                                                                │
│  HIL_STATE_QUATERNION (25 Hz, 仅 SIH)                         │
│  └─ 地面真值（用于 GCS 显示，不参与控制）                     │
│                                                                │
├────────────────────────────────────────────────────────────────┤
│                    外部 → PX4 飞控                             │
│  (仅 SYS_HITL=1 外部HITL模式使用)                             │
│                                                                │
│  HIL_SENSOR (可变频率)                                         │
│  ├─ xacc, yacc, zacc (m/s²)                                   │
│  ├─ xgyro, ygyro, zgyro (rad/s)                               │
│  ├─ xmag, ymag, zmag (Gauss)                                  │
│  ├─ abs_pressure, diff_pressure (hPa)                         │
│  ├─ temperature (°C)                                           │
│  └─ fields_updated: 位掩码指示哪些字段有效                    │
│                                                                │
│  HIL_GPS (可变频率)                                            │
│  └─ lat, lon, alt, vel, hdg, satellites...                    │
└────────────────────────────────────────────────────────────────┘
```

### 7.2 MAVLink HIL 启用逻辑

```cpp
// 文件: src/modules/mavlink/mavlink_main.cpp 行 659-679

if (hil_enabled && !_hil_enabled && _datarate > 5000) {
    _hil_enabled = true;
    configure_stream("HIL_ACTUATOR_CONTROLS", 200.0f);  // 200 Hz

    if (_param_sys_hitl.get() == 2) {  // SIH 模式
        configure_stream("HIL_STATE_QUATERNION", 25.0f); // 地面真值
    } else {
        configure_stream("HIL_STATE_QUATERNION", 0.0f);  // HITL不需要
    }
}
```

### 7.3 HIL 传感器接收

```cpp
// 文件: src/modules/mavlink/mavlink_receiver.cpp 行 330-360

if (_mavlink->get_hil_enabled()) {
    switch (msg->msgid) {
    case MAVLINK_MSG_ID_HIL_SENSOR:
        handle_message_hil_sensor(msg);            // IMU数据
        break;
    case MAVLINK_MSG_ID_HIL_STATE_QUATERNION:
        handle_message_hil_state_quaternion(msg);  // 状态真值
        break;
    case MAVLINK_MSG_ID_HIL_OPTICAL_FLOW:
        handle_message_hil_optical_flow(msg);      // 光流
        break;
    }
}

// handle_message_hil_sensor() 行 2224-2280:
// - 按需创建 PX4Gyroscope (DRV_IMU_DEVTYPE_SIM = 1310988)
// - 按需创建 PX4Accelerometer
// - 按需创建 PX4Magnetometer (DRV_MAG_DEVTYPE_MAGSIM = 197388)
// - 时间戳使用当前 HRT 时间（非 MAVLink 消息时间戳）
```

---

## 8. 从机铰链修正在HIL中的工作方式

### 8.1 三种模式下的铰链修正对比

```
┌────────────────┬──────────────────────────────────────────────────────┐
│                │           铰链修正工作流程                           │
├────────────────┼──────────────────────────────────────────────────────┤
│ SITL (Gazebo)  │ chainwing_slave 运行                                │
│                │  → 读取 IMU (来自GZ物理铰链)                        │
│                │  → 互补滤波估计铰链角度                             │
│                │  → PD控制计算 trim                                  │
│                │  → 发布 chainwing_hinge_status                      │
│                │  → GZMixingInterfaceServo 读取并叠加 trim ✅        │
│                │  → Gazebo 舵面物理响应                              │
│                │                                                      │
│ SIH (HIL)      │ chainwing_slave 运行                                │
│                │  → 读取 IMU (来自SIH通用FW模型)                     │
│                │  → 互补滤波估计铰链角度                             │
│                │  → PD控制计算 trim                                  │
│                │  → 发布 chainwing_hinge_status                      │
│                │  → ❌ 无消费者读取 trim (PWMSim不做trim叠加)        │
│                │  → trim 值被忽略                                    │
│                │                                                      │
│ 真实硬件       │ 每个从机 Pixhawk 独立运行 chainwing_slave           │
│  (3台Pixhawk)  │  → 读取本机 IMU (真实传感器)                       │
│                │  → 铰链角度估计 (真实铰链运动)                      │
│                │  → PD控制计算 trim                                  │
│                │  → 叠加到本机 PWM 输出                              │
│                │  → MAVLink 通信汇报给主机 ✅                        │
└────────────────┴──────────────────────────────────────────────────────┘
```

### 8.2 SIH 模式限制分析

**为什么 SIH 中 trim 不起作用？**

1. SIH 使用 **单刚体** 模型 — 没有铰链自由度
2. 三个机体被视为一个整体 — 没有相对滚转运动
3. IMU 数据反映的是 **整机** 姿态，不是单元间相对姿态
4. chainwing_slave 的互补滤波器会趋向零（无铰链激励）
5. 即使计算出 trim，PWMSim 也不会叠加

**结论**: SIH 模式适合验证 **基础飞行控制**（姿态/位置/任务），但 **不适合验证铰链修正算法**。铰链修正验证需使用 SITL (Gazebo) 或真实硬件。

---

## 9. 硬件部署控制流（3台Pixhawk）

### 9.1 硬件架构

```
                    ┌─────────────────┐
                    │   主机 Pixhawk   │
                    │  MAV_SYS_ID = 1 │
                    │  CW_SLV_EN = 0  │
                    │                 │
                    │  完整 FW 控制栈: │
                    │  navigator      │
                    │  fw_pos_control │
                    │  fw_att_control │
                    │  fw_rate_control│
                    │  control_alloc  │
                    │  EKF2           │
                    └─────┬───┬───────┘
                     UART │   │ UART
         ┌───────────────┘   └───────────────┐
         │                                   │
         ▼                                   ▼
┌─────────────────┐               ┌─────────────────┐
│  左从机 Pixhawk  │               │  右从机 Pixhawk  │
│  MAV_SYS_ID = 2 │               │  MAV_SYS_ID = 3 │
│  CW_SLV_EN = 1  │               │  CW_SLV_EN = 1  │
│                 │               │                 │
│  chainwing_slave│               │  chainwing_slave│
│  (铰链修正)     │               │  (铰链修正)     │
│  EKF2 (本机IMU) │               │  EKF2 (本机IMU) │
│  PWM输出→舵机   │               │  PWM输出→舵机   │
└─────────────────┘               └─────────────────┘
```

### 9.2 硬件模式控制流

```
┌─ 主机 (MAV_SYS_ID=1) ──────────────────────────────────────────┐
│                                                                  │
│  GPS/IMU/气压 → EKF2 → 导航 → 位置控制 → 姿态控制               │
│                                    │                             │
│                             角速率控制 → 控制分配                │
│                                              │                   │
│                                    ┌─────────┼─────────┐        │
│                                    ▼         ▼         ▼        │
│                              Motor_1    Servo_1   (主机舵面)     │
│                             (中央电机) (升降舵)                  │
│                                                                  │
│  MAVLink 发送指令给从机:                                         │
│  DEBUG_FLOAT_ARRAY (id=43, "CW_CMD")                            │
│  ├─ data[0] = pitch_cmd (俯仰指令)                              │
│  ├─ data[1] = throttle_cmd (油门指令)                            │
│  └─ data[2] = roll_cmd (滚转指令)                               │
│                                                                  │
└──────────────────────────────────────────────────────────────────┘
          │  UART (921600 baud)
          ▼
┌─ 从机 (MAV_SYS_ID=2/3) ────────────────────────────────────────┐
│                                                                  │
│  接收主机指令 (CW_CMD) → processMasterCommands()                │
│                                                                  │
│  本机 IMU → 铰链角度估计 (互补滤波器)                           │
│  ├─ roll_rate = angular_vel.xyz[0]                               │
│  ├─ _hinge_angle += roll_rate × dt                               │
│  ├─ roll_error = euler.phi() - _roll_ref                        │
│  └─ _hinge_angle = 0.95 × _hinge_angle + 0.05 × roll_error    │
│                                                                  │
│  PD 控制:                                                        │
│  trim = Kp × hinge_angle + Kd × hinge_rate                     │
│  trim = constrain(trim, -trim_max, +trim_max)                   │
│                                                                  │
│  执行器输出:                                                     │
│  ├─ 电机: throttle = 主机油门指令                                │
│  └─ 舵面: servo = 主机指令 + trim (铰链修正)                    │
│                                                                  │
│  MAVLink 回传状态给主机:                                         │
│  DEBUG_FLOAT_ARRAY (id=42, "CW_HINGE")                          │
│  ├─ data[0] = hinge_angle_left (rad)                            │
│  ├─ data[1] = hinge_angle_right (rad)                           │
│  ├─ data[2] = hinge_rate_left (rad/s)                           │
│  ├─ data[3] = hinge_rate_right (rad/s)                          │
│  ├─ data[4] = trim_left                                         │
│  ├─ data[5] = trim_right                                        │
│  └─ data[6] = data_valid (1.0)                                  │
│                                                                  │
└──────────────────────────────────────────────────────────────────┘
```

### 9.3 硬件通信配置

```bash
# 主机 (MAV_SYS_ID=1):
mavlink start -d /dev/ttyS2 -b 921600 -m custom
mavlink stream -d /dev/ttyS2 -s DEBUG_FLOAT_ARRAY -r 10

# 从机 (MAV_SYS_ID=2/3):
mavlink start -d /dev/ttyS2 -b 921600 -m custom
mavlink stream -d /dev/ttyS2 -s DEBUG_FLOAT_ARRAY -r 10

# 重要: 使用 -m custom 而非 -m onboard
# -m onboard 会触发 ODOMETRY@30Hz 导致 60 warnings/sec 刷屏
```

---

## 10. 控制频率与时序分析

### 10.1 各模块运行频率

```
模块                        SITL频率    SIH(HIL)频率   真实硬件频率
──────────────────────────  ──────────  ────────────   ────────────
EKF2                        250 Hz      250 Hz         250 Hz
fw_att_control              250 Hz      250 Hz         250 Hz
fw_rate_control             250 Hz      250 Hz         250 Hz
fw_pos_control              50 Hz       50 Hz          50 Hz
control_allocator           250 Hz      250 Hz         250 Hz
chainwing_slave             50 Hz       50 Hz          50 Hz
navigator                   ~5 Hz       ~5 Hz          ~5 Hz
SIH physics                 N/A         250 Hz         N/A
GZ physics                  1000 Hz     N/A            N/A
PWMSim                      N/A         250 Hz         N/A
pwm_out (hardware)          N/A         N/A            400 Hz
MAVLink HIL_ACTUATOR_CTRL   N/A         200 Hz         N/A
MAVLink DEBUG_FLOAT_ARRAY   10 Hz       10 Hz          10 Hz
sensor_baro_sim             N/A         50 Hz          N/A
sensor_gps_sim              N/A         5-10 Hz        N/A
```

### 10.2 SIH 控制环总延迟

```
IMU采样 (SIH内部)        0 ms   (无传感器延迟)
  ↓
EKF2 融合                 4 ms   (250 Hz)
  ↓
fw_att_control            4 ms   (250 Hz)
  ↓
fw_rate_control           4 ms   (250 Hz)
  ↓
control_allocator         0 ms   (同周期)
  ↓
PWMSim                    0 ms   (同周期)
  ↓
SIH read_motors           4 ms   (250 Hz)
  ↓
SIH 物理计算              < 1 ms
  ↓
传感器输出                0 ms   (同周期)
──────────────────────────────────
总闭环延迟               ≈ 16 ms (约 62.5 Hz 等效带宽)
```

### 10.3 对比: SITL 控制环延迟

```
Gazebo IMU发布             0 ms
  ↓
GZBridge → uORB            1 ms   (锁步同步)
  ↓
EKF2                       4 ms
  ↓
姿态+速率控制              4 ms
  ↓
控制分配                   0 ms
  ↓
GZMixingInterfaceServo     0 ms
  ↓
GZ 话题 → Gazebo           1 ms
  ↓
Gazebo 物理步进            1 ms   (1000 Hz)
──────────────────────────────────
总闭环延迟               ≈ 11 ms (约 91 Hz 等效带宽)
```

---

## 11. SITL vs HIL vs 硬件 数据流对比

### 11.1 执行器输出路径

```
               SITL                    SIH (HIL)               真实硬件
               ────                    ─────────               ────────
control_alloc  control_alloc           control_alloc           control_alloc
    │              │                       │                       │
    ▼              ▼                       ▼                       ▼
actuator_out   actuator_out            actuator_out            actuator_out
    │              │                       │                       │
    ▼              ▼                       ▼                       ▼
GZMixing       PWMSim(-m hil)          pwm_out                 pwm_out
InterfaceServo     │                       │                       │
    │              ▼                       ▼                       ▼
    │        actuator_outputs_sim      PWM 硬件引脚            PWM 硬件引脚
    │              │                                               │
    ▼              ▼                                               ▼
GZ servo话题   SIH physics              ─── N/A ───            真实舵机
    │              │
    ▼              ▼
Gazebo物理     仿真传感器
```

### 11.2 传感器反馈路径

```
               SITL                    SIH (HIL)               真实硬件
               ────                    ─────────               ────────
Gazebo IMU     SIH内部生成              真实 IMU 芯片
    │              │                       │
    ▼              ▼                       ▼
GZBridge       PX4Accel/Gyro           ADC/SPI 驱动
    │          sensor_*_sim                │
    ▼              │                       ▼
uORB 发布      uORB 发布               uORB 发布
    │              │                       │
    ▼              ▼                       ▼
EKF2           EKF2                     EKF2
```

---

## 12. HIL参数速查表

### 12.1 SIH 系统参数

| 参数 | 默认值 | 说明 | 文件位置 |
|------|--------|------|----------|
| SYS_HITL | 0 | 0=禁用, 1=HITL, 2=SIH | system_params.c:84 |
| CBRK_SUPPLY_CHK | 894281 | 跳过电源检查 | 1103...hil:82 |
| CBRK_IO_SAFETY | 22027 | 跳过IO安全开关 | 1103...hil:83 |

### 12.2 SIH 物理参数

| 参数 | 值 | 说明 |
|------|------|------|
| SIH_MASS | 1.0 kg | 整机质量 |
| SIH_IXX | 1.02 kg·m² | 滚转惯量 |
| SIH_IYY | 0.164 kg·m² | 俯仰惯量 |
| SIH_IZZ | 1.17 kg·m² | 偏航惯量 |
| SIH_T_MAX | 15.0 N | 最大推力 |
| SIH_KDV | 0.5 | 阻力系数 |
| SIH_VEHICLE_TYPE | 1 | 固定翼 |

### 12.3 控制分配参数

| 参数 | 值 | 说明 |
|------|------|------|
| CA_AIRFRAME | 1 | 固定翼 |
| CA_ROTOR_COUNT | 3 | 3电机 |
| CA_ROTOR0_PY | -1.2 | 左电机位置 |
| CA_ROTOR1_PY | 0.0 | 中央电机位置 |
| CA_ROTOR2_PY | +1.2 | 右电机位置 |
| CA_SV_CS_COUNT | 3 | 3舵面 |
| HIL_ACT_FUNC1-6 | 201-203,101-103 | HIL执行器映射 |

### 12.4 从机控制参数

| 参数 | 值 | 说明 |
|------|------|------|
| CW_SLV_EN | 1 (从机) / 0 (主机) | 使能从机控制 |
| CW_SLV_KP | 1.5 | 比例增益 |
| CW_SLV_KD | 0.2 | 微分增益 |
| CW_SLV_TRIM_MAX | 0.3 | 最大trim |
| CW_SLV_LP_FREQ | 10.0 Hz | 低通滤波 |
| CW_SLV_COMM_EN | 1 | MAVLink通信使能 |

---

## 13. 数值计算示例

### 13.1 SIH 模式: 5° 滚转指令响应

```
初始状态: 水平飞行, V=20 m/s, roll=0°

Step 1: 位置控制器要求 roll=5° (0.087 rad)
  → attitude_setpoint.roll = 0.087 rad

Step 2: 姿态控制器
  roll_error = 0.087 - 0.0 = 0.087 rad
  rate_sp = 0.087 / FW_R_TC(0.5) = 0.174 rad/s

Step 3: 角速率控制器
  rate_error = 0.174 - 0.0 = 0.174 rad/s
  τ_roll = P(0.05) × 0.174 + FF(0.6) × 0.174 = 0.0087 + 0.1044 = 0.113

Step 4: 控制分配
  Servo_0 (LeftElevon)  = +0.5 × 0.113 = +0.057
  Servo_1 (Elevator)    =  0.0
  Servo_2 (RightElevon) = -0.5 × 0.113 = -0.057

Step 5: PWMSim 缩放
  PWM_0 = 1500 + 500 × 0.057 = 1528
  PWM_1 = 1500
  PWM_2 = 1500 - 500 × 0.057 = 1472
  → actuator_outputs_sim: [0.057, 0.0, -0.057, ...]

Step 6: SIH 物理
  _u[0] = 0.057 (副翼)
  → wing_l 偏转 +0.057 rad, wing_r 偏转 -0.057 rad
  → 差异升力产生滚转力矩
  → equations_of_motion: 滚转加速度 > 0
  → 飞机开始滚转

Step 7: 传感器反馈
  陀螺仪: roll_rate > 0
  加速度计: 侧向分量变化
  → EKF2 更新 vehicle_attitude
  → 闭环持续修正
```

### 13.2 SIH 中 chainwing_slave 的行为

```
因为 SIH 是单刚体，没有铰链:

chainwing_slave 50Hz 运行:
  读取 vehicle_angular_velocity → roll_rate = ω_x (整机滚转)
  互补滤波积分: _hinge_angle += roll_rate × dt
  但是: 衰减项 _hinge_angle *= exp(-dt/tau)
  且: 姿态修正 roll_error = euler.phi() - _roll_ref ≈ 0
  
  结果: _hinge_angle ≈ 0 (无铰链激励时趋于零)
  trim = Kp × 0 + Kd × 0 ≈ 0 (trim 趋于零)
  
  → chainwing_hinge_status 发布 (angle≈0, trim≈0)
  → 但 PWMSim 不读取此话题，即使非零也无影响
```

---

## 14. 故障排除

### 14.1 常见问题

| 问题 | 原因 | 解决方案 |
|------|------|----------|
| SIH 不启动 | SYS_HITL ≠ 2 | 使用 1103_chainwing_sih.hil 机架 |
| 无传感器数据 | sensor_*_sim 未启动 | 检查 rcS 启动序列 |
| 飞机无法解锁 | 安全检查未跳过 | 设置 CBRK_SUPPLY_CHK=894281 |
| ODOMETRY 刷屏 | 使用了 -m onboard | 改用 -m custom |
| trim 不起作用 | SIH 无铰链物理 | 使用 SITL/Gazebo 验证 |
| 飞行不稳定 | SIH 物理参数不匹配 | 调整 SIH_MASS/IXX/IYY/IZZ |
| GPS 定位失败 | GPS_1_CONFIG 被禁用 | SIH 使用 sensor_gps_sim |

### 14.2 SIH 模式验证命令

```bash
# 在 NuttX shell (nsh>) 中:
listener vehicle_attitude -r 2 -n 5        # 确认姿态估计
listener actuator_outputs_sim -r 2 -n 5    # 确认执行器输出
listener sensor_accel -r 2 -n 5            # 确认SIH加速度计
listener chainwing_hinge_status -r 2 -n 5  # 确认从机模块运行

# 查看模块状态:
chainwing_slave status
simulator_sih status
pwm_out_sim status
```

---

## 15. 源文件清单

### 15.1 HIL 核心文件

| 文件 | 行数 | 说明 |
|------|------|------|
| `src/modules/simulation/simulator_sih/sih.cpp` | ~450 | SIH 物理引擎主循环 |
| `src/modules/simulation/simulator_sih/sih.hpp` | ~220 | SIH 类定义 |
| `src/modules/simulation/simulator_sih/aero.hpp` | ~260 | 气动模型 |
| `src/modules/simulation/pwm_out_sim/PWMSim.cpp` | ~200 | HIL PWM 仿真输出 |
| `src/modules/simulation/pwm_out_sim/PWMSim.hpp` | ~90 | PWMSim 类定义 |
| `ROMFS/px4fmu_common/init.d/airframes/1103_chainwing_sih.hil` | 131 | Chain-Wing SIH 机架 |
| `ROMFS/px4fmu_common/init.d/rcS` | ~500 | 启动脚本 (行337-385: HIL分支) |

### 15.2 MAVLink HIL 文件

| 文件 | 说明 |
|------|------|
| `src/modules/mavlink/streams/HIL_ACTUATOR_CONTROLS.hpp` | 执行器输出流 (200Hz) |
| `src/modules/mavlink/streams/HIL_STATE_QUATERNION.hpp` | 状态真值流 (25Hz) |
| `src/modules/mavlink/mavlink_main.cpp` | HIL 流配置 (行655-679) |
| `src/modules/mavlink/mavlink_receiver.cpp` | HIL 传感器接收 (行330-360, 2224-2280) |

### 15.3 Chain-Wing 自定义文件 (HIL中也运行)

| 文件 | 说明 |
|------|------|
| `src/modules/chainwing_slave/ChainwingSlave.cpp` | 从机控制器 |
| `src/modules/chainwing_slave/ChainwingSlave.hpp` | 从机类定义 |
| `src/modules/chainwing_slave/chainwing_slave_params.c` | 6个CW参数 |
| `msg/ChainwingHingeStatus.msg` | uORB 消息定义 |

### 15.4 控制栈文件 (HIL/SITL/硬件 通用)

| 文件 | 说明 |
|------|------|
| `src/modules/fw_att_control/` | 姿态控制器 |
| `src/modules/fw_rate_control/` | 角速率控制器 |
| `src/modules/fw_pos_control/` | 位置控制器 |
| `src/modules/control_allocator/` | 控制分配器 |
| `src/modules/ekf2/` | 状态估计器 |
| `ROMFS/px4fmu_common/init.d/rc.fw_apps` | FW应用启动 |

---

> **文档结束** | CHAINWING_HIL_CONTROL_FLOW.md v1.0 | 2026-03-23
