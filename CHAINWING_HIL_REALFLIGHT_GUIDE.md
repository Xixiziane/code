# Chain-Wing UAV 硬件仿真(HIL)与真机实飞 实现指南

> **版本**: v1.0 | **日期**: 2026-03-23 | **基于**: PX4 v1.14.x

---

## 目录

- [§1 项目当前状态评估](#1-项目当前状态评估)
- [§2 三种运行模式架构对比](#2-三种运行模式架构对比)
- [§3 当前已有的硬件支持基础](#3-当前已有的硬件支持基础)
- [§4 SIH硬件在环仿真详解](#4-sih硬件在环仿真详解)
- [§5 SIH模式下启用从机铰链修正](#5-sih模式下启用从机铰链修正)
- [§6 真机实飞架构设计](#6-真机实飞架构设计)
- [§7 真机实飞：硬件机架文件改造](#7-真机实飞硬件机架文件改造)
- [§8 真机实飞：PWM舵面输出适配](#8-真机实飞pwm舵面输出适配)
- [§9 真机实飞：三机通信方案](#9-真机实飞三机通信方案)
- [§10 真机实飞：安全机制设计](#10-真机实飞安全机制设计)
- [§11 从SITL到真机的渐进验证路径](#11-从sitl到真机的渐进验证路径)
- [§12 需要修改/新增的文件清单](#12-需要修改新增的文件清单)
- [§13 常见问题与注意事项](#13-常见问题与注意事项)

---

## §1 项目当前状态评估

### 1.1 已实现（✅）

| 功能 | 文件 | 状态 |
|------|------|------|
| SITL 仿真（Gazebo） | `4008_gz_chainwing_3body` | ✅ 完整 |
| 从机控制器（ChainwingSlave） | `ChainwingSlave.cpp/hpp` | ✅ 完整 |
| 铰链角估计（互补滤波） | `ChainwingSlave.cpp:131-205` | ✅ 完整 |
| PD修正控制律 | `ChainwingSlave.cpp:109-110` | ✅ 完整 |
| GZ仿真舵面叠加 | `GZMixingInterfaceServo.cpp:76-86` | ✅ 完整 |
| MAVLink通信协议（设计） | `ChainwingSlave.cpp:222-267` | ✅ 代码存在 |
| SIH机架文件 | `1103_chainwing_sih.hil` | ✅ 基础存在 |
| 硬件机架文件 | `2150_chainwing` | ✅ 基础存在 |
| 6个CW_SLV_*参数 | `chainwing_slave_params.c` | ✅ 完整 |

### 1.2 需要补充才能支持HIL（⚠️）

| 缺失项 | 工作量 | 说明 |
|--------|--------|------|
| SIH机架加CW_SLV参数 | 小（10行） | `1103_chainwing_sih.hil` 添加从机参数 |
| SIH物理引擎无铰链 | 中（需评估） | SIH是刚体模型，无多体铰链 |

### 1.3 需要补充才能支持真机（❌）

| 缺失项 | 工作量 | 说明 |
|--------|--------|------|
| 硬件机架加CW_SLV参数 | 小（15行） | `2150_chainwing` 添加从机参数+启动 |
| PWM舵面trim叠加 | 中（~50行） | 需绕过GZMixingInterfaceServo |
| 多机UART通信验证 | 中 | MAVLink DEBUG_FLOAT_ARRAY over UART |
| 铰链角传感器替代方案 | 大 | 真机需编码器，非IMU互补滤波 |
| 安全失效保护 | 大 | 通信丢失、铰链卡死等failsafe |

---

## §2 三种运行模式架构对比

### 2.1 SITL（当前主力）

```
┌─────────────────────────────────────────────────────────────────┐
│                        PC 上运行                                 │
│                                                                   │
│  ┌──────────┐  uORB  ┌──────────────┐  uORB  ┌──────────────┐  │
│  │ PX4 SITL │───────→│ChainwingSlave│───────→│GZMixingServo │  │
│  │(px4_sitl)│        │  (50Hz PD)   │        │  (trim叠加)  │  │
│  └──────────┘        └──────────────┘        └──────┬───────┘  │
│       ↑                                              │          │
│       │                                         GZ Transport    │
│       │              ┌────────────────┐              │          │
│       └──────────────│   Gazebo Sim   │←─────────────┘          │
│         传感器反馈     │ (3体+2铰链)   │    舵面/电机命令         │
│                      └────────────────┘                         │
└─────────────────────────────────────────────────────────────────┘
```

**特点**: 所有运行在同一PC进程中，通过uORB通信，Gazebo提供物理引擎

### 2.2 SIH 硬件在环（下一步目标）

```
┌─────────────────────────────────────────────────┐
│                  Pixhawk 硬件                     │
│                                                   │
│  ┌──────────┐  uORB  ┌──────────────┐           │
│  │  PX4 FW  │───────→│ChainwingSlave│           │
│  │ 控制栈   │        │  (50Hz PD)   │           │
│  └──────────┘        └──────┬───────┘           │
│       ↑                      │                    │
│       │                 uORB │ chainwing_         │
│       │                      │ hinge_status       │
│  ┌────┴─────┐         ┌─────▼───────┐           │
│  │ SIH 物理 │         │ PWM Mixer   │           │
│  │  引擎    │         │ + trim叠加  │           │
│  │ (250Hz)  │         └──────┬──────┘           │
│  └──────────┘                │                    │
│                         PWM输出(回环)             │
│                   （不连接真实舵机）               │
└─────────────────────────────────────────────────┘
```

**特点**: 在真实Pixhawk上运行，SIH模块替代外部仿真器，PWM输出回环不驱动真实舵机

### 2.3 真机实飞（最终目标）

```
                    ┌───────────────┐
                    │   主机 (Master) │  MAV_SYS_ID=1
                    │   Pixhawk #1  │  CW_SLV_EN=0
                    │   标准FW控制   │
                    └───┬───────┬───┘
                  UART1 │       │ UART2
          ┌─────────────┘       └─────────────┐
          ▼                                   ▼
┌─────────────────┐                 ┌─────────────────┐
│ 左从机 (Slave-L) │                │ 右从机 (Slave-R) │
│   Pixhawk #2    │                │   Pixhawk #3    │
│  MAV_SYS_ID=2   │                │  MAV_SYS_ID=3   │
│  CW_SLV_EN=1    │                │  CW_SLV_EN=1    │
│  铰链估计+PD修正  │                │  铰链估计+PD修正  │
│  本地PWM输出     │                │  本地PWM输出     │
└─────────────────┘                └─────────────────┘
```

**特点**: 三台独立Pixhawk，主机做导航/控制，从机做铰链修正，UART MAVLink通信

---

## §3 当前已有的硬件支持基础

### 3.1 SIH 机架文件 (`1103_chainwing_sih.hil`)

**已有内容**:
```bash
# SYS_HITL=2 → SIH模式（板上物理引擎）
param set-default SYS_HITL 2

# SIH 物理参数
param set SIH_MASS 1.0          # 总质量 1.0 kg
param set SIH_IXX 1.02          # 滚转惯量
param set SIH_IYY 0.164         # 俯仰惯量
param set SIH_IZZ 1.17          # 偏航惯量
param set SIH_T_MAX 15.0        # 3电机总推力 15N
param set SIH_VEHICLE_TYPE 1    # 固定翼

# HIL 执行器映射
param set-default HIL_ACT_FUNC1 201  # Servo 0 - 左Elevon
param set-default HIL_ACT_FUNC2 202  # Servo 1 - 升降舵
param set-default HIL_ACT_FUNC3 203  # Servo 2 - 右Elevon
param set-default HIL_ACT_FUNC4 101  # Motor 0 - 左电机
param set-default HIL_ACT_FUNC5 102  # Motor 1 - 中电机
param set-default HIL_ACT_FUNC6 103  # Motor 2 - 右电机
```

**缺失**: CW_SLV_* 参数和 chainwing_slave start

### 3.2 硬件机架文件 (`2150_chainwing`)

**已有内容**:
```bash
# 3电机差动偏航 + 3舵面（左elevon, 升降舵, 右elevon）
CA_ROTOR_COUNT=3, CA_SV_CS_COUNT=3
# 控制分配矩阵（与SITL完全一致）
# 空速参数、姿态控制参数
```

**缺失**: CW_SLV_* 参数、chainwing_slave start、PWM通道映射

### 3.3 硬件目标板

| 板子 | 路径 | UART数量 | 适用 |
|------|------|----------|------|
| fmu-v5 | `boards/px4/fmu-v5/` | 5+个 | ✅ 主力 |
| fmu-v6x | `boards/px4/fmu-v6x/` | 8个 | ✅ 推荐 |
| fmu-v6c | `boards/px4/fmu-v6c/` | 6个 | ✅ 备选 |

---

## §4 SIH硬件在环仿真详解

### 4.1 SIH 是什么

SIH (Simulator In Hardware) 是 PX4 内置的物理引擎模块，运行在 Pixhawk 硬件上：
- **频率**: 250 Hz（实时刚体动力学）
- **输入**: 执行器输出（电机+舵面）
- **输出**: 模拟传感器数据（IMU、气压计、GPS、磁力计、空速）
- **用途**: 无需外部PC或仿真器，只需Pixhawk + QGC

### 4.2 SIH 的局限性

**⚠️ 关键限制：SIH 是单刚体模型，没有多体铰链！**

```
SITL (Gazebo):  [左翼]──铰链──[中心]──铰链──[右翼]  ← 三刚体 + 两铰链
SIH:            [================整体================]  ← 单刚体（无铰链）
```

**影响**:
- SIH 无法产生真实的铰链偏转
- `vehicle_angular_velocity` 只反映整机角速度，不包含相对滚转
- 互补滤波器估计的 `_hinge_angle_left/right` 始终 ≈ 0
- PD 控制器输出 trim ≈ 0 → 修正无效果

### 4.3 SIH 模式下从机修正的意义

虽然 SIH 无法模拟铰链物理，但启用从机模块有以下价值：

1. **验证模块启动和运行**：确认 chainwing_slave 在硬件上正常启动
2. **验证 uORB 数据流**：确认 `chainwing_hinge_status` 正常发布
3. **验证 MAVLink 通信**：通过 UART 测试 DEBUG_FLOAT_ARRAY 收发
4. **验证 PWM 通道映射**：确认舵面输出信号正确
5. **作为真机前的硬件集成测试**：验证编译、烧录、参数加载

---

## §5 SIH模式下启用从机铰链修正

### 5.1 修改 `1103_chainwing_sih.hil`

在文件末尾追加以下内容：

```bash
# ============================================================
# Chain-Wing Slave Controller Parameters
# ============================================================
# 注意: SIH 是单刚体模型，无法模拟铰链物理
# 启用从机模块主要用于硬件集成测试（通信、PWM、参数验证）

# 启用从机控制器
param set-default CW_SLV_EN 1

# PD 增益（与 SITL 一致，用于硬件验证）
# 基于 report.pdf 聚氨酯弹性体推导:
#   k=200 N·m/rad, c=12 N·m·s/rad → ωn=2.72 Hz, ζ=0.51
param set-default CW_SLV_KP 1.5
param set-default CW_SLV_KD 0.2

# 最大修正量 = 30% 舵面行程
param set-default CW_SLV_TRIM_MAX 0.3

# 角速率低通滤波 10 Hz
param set-default CW_SLV_LP_FREQ 10.0

# MAVLink 通信（SIH 模式下可启用用于测试）
# 设为 1 时需要额外配置 mavlink 实例:
#   mavlink start -d /dev/ttyS2 -b 921600 -m custom
#   mavlink stream -d /dev/ttyS2 -s DEBUG_FLOAT_ARRAY -r 10
param set-default CW_SLV_COMM_EN 0
```

### 5.2 添加模块自启动

在 `1103_chainwing_sih.hil` 末尾添加：

```bash
# 启动从机控制模块（50Hz 运行循环）
chainwing_slave start
```

### 5.3 确保编译包含模块

检查 `boards/px4/fmu-v5/default.px4board`（或目标板）中包含：

```
CONFIG_MODULES_CHAINWING_SLAVE=y
```

如果不存在，需要添加。

### 5.4 SIH 模式启动命令

```bash
# 编译 (以 fmu-v5 为例)
make px4_fmu-v5_default

# 烧录
make px4_fmu-v5_default upload

# QGC 中设置:
# SYS_AUTOSTART = 1103
# 重启后自动加载 1103_chainwing_sih.hil
```

### 5.5 SIH 模式验证清单

```bash
# 在 QGC MAVLink Console 中验证:

# 1. 确认模块运行
chainwing_slave status
# 期望输出: Running, 50Hz

# 2. 确认话题发布
listener chainwing_hinge_status -n 5
# 期望: data_valid=1, hinge_angle_left ≈ 0 (SIH无铰链)

# 3. 确认参数
param show CW_SLV*
# 期望: CW_SLV_EN=1, KP=1.5, KD=0.2 等

# 4. 测试MAVLink通信 (如果 COMM_EN=1)
listener debug_array -n 5
# 期望: id=42, name=CW_HINGE
```

---

## §6 真机实飞架构设计

### 6.1 单机方案 vs 多机方案

#### 方案A：单Pixhawk控制全部三个单元（推荐首选）

```
                ┌─────────────────────┐
                │     Pixhawk #1      │
                │                     │
                │  PX4 FW 控制栈      │
                │  ChainwingSlave     │
                │  PWM 输出:          │
                │    PWM1→左Elevon    │
                │    PWM2→升降舵      │
                │    PWM3→右Elevon    │
                │    PWM4→左电机      │
                │    PWM5→中电机      │
                │    PWM6→右电机      │
                └─────────────────────┘
```

**优点**: 最简单、无通信延迟、SITL验证直接适用
**缺点**: PWM线需要拉到翼尖（距离远、信号衰减）
**适用**: 翼展 <3m 的小型原型机

#### 方案B：三Pixhawk分布式控制（最终目标）

见 §2.3 架构图。每台 Pixhawk 独立运行 PX4，通过 UART MAVLink 通信。

**优点**: 各单元独立自主、PWM线短、可扩展
**缺点**: 通信延迟、复杂度高、需要更多硬件
**适用**: 翼展 >3m 或研究分布式控制

### 6.2 推荐实施路线

```
阶段1: SITL Gazebo（✅ 已完成）
  ↓
阶段2: SIH 硬件在环（本文 §4-§5）
  ↓
阶段3: 单机真机飞行（本文 §7-§8）
  ↓
阶段4: 三机分布式飞行（本文 §9）
```

---

## §7 真机实飞：硬件机架文件改造

### 7.1 完整的 `2150_chainwing` 修改建议

在现有文件末尾追加：

```bash
# ============================================================
# Chain-Wing Slave Controller (真机参数)
# ============================================================

# 启用从机铰链修正
param set-default CW_SLV_EN 1

# PD 增益 — 真机可能需要重新调整
# 初始值与 SITL 一致，飞行后根据日志调整
param set-default CW_SLV_KP 1.5
param set-default CW_SLV_KD 0.2

# 真机安全起见，初始 trim 限制小一些 (20%)
# 经过飞行验证后可逐步放大到 0.3
param set-default CW_SLV_TRIM_MAX 0.2

# 角速率低通滤波
# 真机振动更大，可能需要降低频率到 5-8 Hz
param set-default CW_SLV_LP_FREQ 8.0

# 多机通信（单机方案设为0，三机方案设为1）
param set-default CW_SLV_COMM_EN 0

# ============================================================
# PWM Output Mapping (MAIN outputs)
# ============================================================
# MAIN1-3: 电机 (已由 CA_ROTOR 映射)
# MAIN4: 右Elevon (servo_0 → 接收 trim_left)
# MAIN5: 升降舵   (servo_1 → 无trim)
# MAIN6: 左Elevon (servo_2 → 接收 trim_right)

# PWM 输出范围 (微秒)
param set-default PWM_MAIN_MIN1 1000
param set-default PWM_MAIN_MAX1 2000
param set-default PWM_MAIN_MIN4 1000
param set-default PWM_MAIN_MAX4 2000
param set-default PWM_MAIN_MIN5 1000
param set-default PWM_MAIN_MAX5 2000
param set-default PWM_MAIN_MIN6 1000
param set-default PWM_MAIN_MAX6 2000

# 舵面失效安全位置 (中位)
param set-default PWM_MAIN_FAIL4 1500
param set-default PWM_MAIN_FAIL5 1500
param set-default PWM_MAIN_FAIL6 1500

# 油门失效安全 (停机)
param set-default PWM_MAIN_FAIL1 900
param set-default PWM_MAIN_FAIL2 900
param set-default PWM_MAIN_FAIL3 900

# ============================================================
# 模块启动
# ============================================================
chainwing_slave start
```

---

## §8 真机实飞：PWM舵面输出适配

### 8.1 核心问题

**当前 trim 叠加在 `GZMixingInterfaceServo.cpp` 中，这是 Gazebo 专用模块！**

真机使用的是 `PWMOut` (NuttX) 或 `MixingOutput`，不经过 GZ 接口。

### 8.2 解决方案：在 `ControlAllocator` 输出端叠加

最优雅的方案是在控制分配器输出（`actuator_servos` uORB）之后、PWM驱动之前叠加trim。

**需要修改的文件**: `src/modules/chainwing_slave/ChainwingSlave.cpp`

**建议新增方法 `applyTrimToServos()`**:

```cpp
// 建议在 ChainwingSlave.cpp 中新增
void ChainwingSlave::applyTrimToServos()
{
    // 读取控制分配器输出
    actuator_servos_s servos{};
    if (!_actuator_servos_sub.copy(&servos)) {
        return;
    }

    // 计算 trim
    float trim_left = computeTrim(_hinge_angle_left, _hinge_rate_left);
    float trim_right = computeTrim(_hinge_angle_right, _hinge_rate_right);

    // 叠加 trim 到 servo 通道
    // servo_0 (index 0) = 左 elevon → +trim_left
    // servo_2 (index 2) = 右 elevon → +trim_right
    servos.control[0] = math::constrain(servos.control[0] + trim_left, -1.0f, 1.0f);
    // servo_1 (index 1) 不修改
    servos.control[2] = math::constrain(servos.control[2] + trim_right, -1.0f, 1.0f);

    // 重新发布到带trim的话题
    // 方案1: 直接修改 actuator_servos (需要小心竞争)
    // 方案2: 发布到 actuator_servos_trim 自定义话题 (更安全)
    servos.timestamp = hrt_absolute_time();
    _actuator_servos_trim_pub.publish(servos);
}
```

### 8.3 替代方案：使用 mixer 文件中的 trim 参数

PX4 的 PWM mixer 支持静态 trim offset。可以通过参数动态调整：

```bash
# QGC 中实时调整 trim (不需要代码修改)
param set CA_SV_CS0_TRIM 0.05   # 左 elevon 正向 trim
param set CA_SV_CS2_TRIM -0.03  # 右 elevon 负向 trim
```

**但这是静态值**，不能根据铰链角实时调整。适合初步飞行测试。

### 8.4 推荐方案比较

| 方案 | 复杂度 | 动态性 | 代码改动 | 推荐 |
|------|--------|--------|----------|------|
| A: 修改控制分配输出 | 中 | ✅ 实时 | ~50行 | ⭐⭐⭐⭐⭐ |
| B: 自定义 uORB 话题 | 大 | ✅ 实时 | ~100行 | ⭐⭐⭐ |
| C: 静态 CA_SV_CSx_TRIM | 小 | ❌ 固定 | 0行 | ⭐⭐（首飞测试用） |

---

## §9 真机实飞：三机通信方案

### 9.1 通信协议（已实现）

项目已有 MAVLink DEBUG_FLOAT_ARRAY 通信代码：

```
从机 → 主机 (id=42, "CW_HINGE"):
  data[0]: hinge_angle_left  (rad)
  data[1]: hinge_angle_right (rad)
  data[2]: hinge_rate_left   (rad/s)
  data[3]: hinge_rate_right  (rad/s)
  data[4]: trim_left         (normalized)
  data[5]: trim_right        (normalized)
  data[6]: data_valid        (1.0/0.0)

主机 → 从机 (id=43, "CW_CMD"):
  data[0]: pitch_cmd         (normalized)
  data[1]: throttle          (normalized)
  data[2]: roll_cmd          (normalized)
```

### 9.2 硬件连线

```
主机 Pixhawk #1                从机 Pixhawk #2 (左)
┌──────────┐                   ┌──────────┐
│ TELEM2   ├───── UART ───────┤ TELEM2   │
│ (TX/RX)  │    921600 bps    │ (TX/RX)  │
└──────────┘                   └──────────┘

主机 Pixhawk #1                从机 Pixhawk #3 (右)
┌──────────┐                   ┌──────────┐
│ TELEM3   ├───── UART ───────┤ TELEM2   │
│ (TX/RX)  │    921600 bps    │ (TX/RX)  │
└──────────┘                   └──────────┘
```

### 9.3 主机机架文件配置

```bash
# 主机 (MAV_SYS_ID=1, CW_SLV_EN=0)
# 添加两个 MAVLink UART 实例:

# 连接左从机
mavlink start -d /dev/ttyS2 -b 921600 -m custom
mavlink stream -d /dev/ttyS2 -s DEBUG_FLOAT_ARRAY -r 10

# 连接右从机
mavlink start -d /dev/ttyS3 -b 921600 -m custom
mavlink stream -d /dev/ttyS3 -s DEBUG_FLOAT_ARRAY -r 10
```

### 9.4 从机机架文件配置

```bash
# 从机 (MAV_SYS_ID=2或3, CW_SLV_EN=1)
param set MAV_SYS_ID 2       # 左从机=2, 右从机=3
param set CW_SLV_EN 1
param set CW_SLV_COMM_EN 1

# UART 连接主机
mavlink start -d /dev/ttyS2 -b 921600 -m custom
mavlink stream -d /dev/ttyS2 -s DEBUG_FLOAT_ARRAY -r 10

# 启动从机模块
chainwing_slave start
```

### 9.5 ⚠️ 关键提醒

**必须使用 `-m custom` 模式！**

`-m onboard` 会启用 ODOMETRY@30Hz 流，其中硬编码了 `estimator_type=MAV_ESTIMATOR_TYPE_AUTOPILOT(8)`，导致接收端每秒输出 60 次警告：

```
WARN [mavlink] ODOMETRY: estimator_type 8 unsupported
```

详见 `CHAINWING_COMMUNICATION_GUIDE.md §13.2`。

---

## §10 真机实飞：安全机制设计

### 10.1 通信超时保护

ChainwingSlave 已有 500ms 超时机制：

```cpp
// ChainwingSlave.cpp:263
static constexpr hrt_abstime MASTER_CMD_TIMEOUT_US = 500000; // 500ms
if (hrt_elapsed_time(&_last_master_cmd) > MASTER_CMD_TIMEOUT_US) {
    _master_cmd_valid = false;
}
```

**建议增强**（真机需要）：

```cpp
// 建议: 通信丢失后逐步减小 trim 而非突然归零
if (!_master_cmd_valid) {
    // 缓慢衰减 trim (2秒归零)
    float decay = expf(-dt / 2.0f);
    _hinge_angle_left *= decay;
    _hinge_angle_right *= decay;
}
```

### 10.2 铰链角限幅保护

已有 `TRIM_MAX` 限制（`computeTrim()` 中的 `math::constrain`）。

**建议增加硬件层面保护**：

```cpp
// 建议: 铰链角超限报警
if (fabsf(_hinge_angle_left) > 0.26f || fabsf(_hinge_angle_right) > 0.26f) {
    // 超过 15° → 发送 MAVLink 警告
    mavlink_log_warning(&_mavlink_log_pub,
        "CW HINGE EXCEEDED: L=%.1f R=%.1f deg",
        (double)math::degrees(_hinge_angle_left),
        (double)math::degrees(_hinge_angle_right));
}
```

### 10.3 电机差动限制

真机应限制差动推力范围，防止偏航过度：

```bash
# 建议参数 (在机架文件中)
param set-default FW_YAW_STAB_SC 1.0   # 降低偏航增益
param set-default CA_ROTOR0_CT 1.0      # 电机推力系数
```

---

## §11 从SITL到真机的渐进验证路径

### 阶段 1: SITL Gazebo ✅（已完成）

- [x] 完整3体铰链仿真
- [x] ChainwingSlave 50Hz PD 控制
- [x] 自动飞行任务验证

### 阶段 2: SIH 硬件在环

- [ ] 修改 `1103_chainwing_sih.hil`（添加 CW_SLV_* 参数）
- [ ] 编译 fmu-v5/v6x 固件，确认包含 chainwing_slave 模块
- [ ] 烧录 Pixhawk，通过 QGC 验证模块运行
- [ ] 通过 `listener` 确认 uORB 话题发布
- [ ] 如有第二台 Pixhawk，测试 UART MAVLink 通信

### 阶段 3: 地面系留测试

- [ ] 组装三单元原型机（翼面+铰链+电机+舵面）
- [ ] 单 Pixhawk 控制所有 6 通道 PWM
- [ ] 手动激励铰链（用手扭动翼尖），观察 elevon 修正响应
- [ ] 电机怠速运转，确认 PWM 通道映射
- [ ] QGC 中调整 PD 增益（CW_SLV_KP/KD）

### 阶段 4: 首飞（单 Pixhawk，CW_SLV_EN=0）

- [ ] 使用 `2150_chainwing` 机架，**禁用**从机修正
- [ ] 纯标准固定翼飞行，验证基础飞行性能
- [ ] 确认空速、姿态、导航正常
- [ ] 下载 .ulg 日志，分析铰链角（如果铰链上有编码器）

### 阶段 5: 启用修正飞行

- [ ] CW_SLV_EN=1，CW_SLV_TRIM_MAX=0.15（保守起步）
- [ ] 飞行 + 日志分析，确认 trim 输出合理
- [ ] 逐步增加 TRIM_MAX 到 0.3
- [ ] 对比有/无修正的飞行日志

### 阶段 6: 三机分布式（如需要）

- [ ] 三台 Pixhawk 分别烧录固件
- [ ] 配置 MAV_SYS_ID=1/2/3
- [ ] UART 连线 + MAVLink 通信测试
- [ ] 地面功能测试 → 系留飞行 → 自由飞行

---

## §12 需要修改/新增的文件清单

### 阶段2（SIH）需要的修改

| 文件 | 修改类型 | 内容 |
|------|----------|------|
| `ROMFS/.../1103_chainwing_sih.hil` | 修改 | 添加 CW_SLV_* 参数 + `chainwing_slave start` |
| `boards/px4/fmu-v5/default.px4board` | 检查 | 确认含 `CONFIG_MODULES_CHAINWING_SLAVE=y` |

### 阶段3-5（真机单Pixhawk）需要的修改

| 文件 | 修改类型 | 内容 |
|------|----------|------|
| `ROMFS/.../2150_chainwing` | 修改 | 添加 CW_SLV_* 参数 + PWM配置 + 模块启动 |
| `ChainwingSlave.cpp` | 修改 | 添加 PWM trim 叠加逻辑（绕过 GZMixingServo） |
| `boards/px4/fmu-v5/default.px4board` | 检查 | 确认含模块 |

### 阶段6（三机分布式）需要的修改

| 文件 | 修改类型 | 内容 |
|------|----------|------|
| 新增: 从机硬件机架文件 | 新建 | 基于 2150 修改，添加 UART MAVLink 配置 |
| `ChainwingSlave.cpp` | 修改 | 通信超时保护增强 |

---

## §13 常见问题与注意事项

### Q1: SIH 能模拟铰链变形吗？

**不能。** SIH 是单刚体6DoF模型，无法模拟多体铰链。铰链测试只能用 SITL Gazebo 或真机。

### Q2: 真机怎么测量铰链角？

**三种方案**:

| 方案 | 精度 | 成本 | 复杂度 |
|------|------|------|--------|
| IMU互补滤波（当前） | ±2-5° | 低（板载） | 低 |
| 磁编码器（AS5600） | ±0.1° | ￥15/个 | 中 |
| 电位器 | ±1° | ￥5/个 | 低 |

**推荐**: 首飞用 IMU 互补滤波（已实现），后续加磁编码器。

### Q3: 真机 PD 增益需要重新调整吗？

**大概率需要。** 因为：
- 真机振动比仿真大 → LP_FREQ 可能需要降到 5-8 Hz
- 真机舵面响应延迟 → KD 可能需要减小
- 真实气动力不同于仿真模型

**建议**: 首飞用保守参数（KP=0.8, KD=0.1, TRIM_MAX=0.15），逐步增加。

### Q4: `-m onboard` 还是 `-m custom`？

**必须用 `-m custom`！** 原因详见 §9.5。

### Q5: 单Pixhawk最多支持几个PWM通道？

| 板子 | MAIN | AUX | 总计 |
|------|------|-----|------|
| fmu-v5 | 8 | 6 | 14 |
| fmu-v6x | 8 | 8 | 16 |

Chain-Wing 需要 6 通道（3电机 + 3舵面），绰绰有余。

### Q6: 需要铰链角传感器还是可以用 IMU？

**短期**: IMU 互补滤波足够（已实现），精度约 ±2-5°，适合验证概念。

**长期**: 建议加装磁编码器（AS5600），精度 ±0.1°，I2C 接口简单。需要在 ChainwingSlave 中添加 I2C 读取代码替代互补滤波。

---

> **文档结束**
>
> 相关文档:
> - `CHAINWING_CONTROL_FLOW.md` — 完整8层控制流程
> - `CHAINWING_COMMUNICATION_GUIDE.md` — 通信协议详解
> - `CHAINWING_PARAMETER_REFERENCE.md` — 参数总表
> - `CHAINWING_SLAVE_IMPLEMENTATION.md` — 实现细节
