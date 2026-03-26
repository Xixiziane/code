# 链翼无人机主从机通信系统技术文档

> **版本**: v3.0  
> **日期**: 2026-03-26  
> **项目**: PX4_test — 链翼（Chain-Wing）三体固定翼无人机  
> **范围**: 完整项目概述 + 主从机 UART+MAVLink 通信方式 + 验证步骤  
> **v3.0 新增**: chainwing_master 模块、双模铰链角估计、CW_CMD data[3]=roll_attitude、自动化参数扫描脚本

---

## 目录

- [§1 项目概述](#1-项目概述)
- [§2 系统架构](#2-系统架构)
- [§3 通信架构详解](#3-通信架构详解)
- [§4 通信协议定义](#4-通信协议定义)
- [§5 代码实现详解](#5-代码实现详解)
- [§6 参数说明](#6-参数说明)
- [§7 SITL 仿真验证步骤](#7-sitl-仿真验证步骤)
- [§8 硬件验证步骤](#8-硬件验证步骤)
- [§9 数据流完整追踪](#9-数据流完整追踪)
- [§10 Logger 飞行日志记录](#10-logger-飞行日志记录)
- [§11 文件清单](#11-文件清单)
- [§12 故障排除](#12-故障排除)
- [§13 修改历史记录](#13-修改历史记录)
- [§14 自动化参数扫描脚本](#14-自动化参数扫描脚本) **[v3.0 新增]**

---

## §1 项目概述

### 1.1 什么是链翼无人机

链翼（Chain-Wing）无人机由三架固定翼单元通过翼尖铰链连接组成：

```
      ┌──────────┐   铰链    ┌──────────┐   铰链    ┌──────────┐
      │ 左侧从机 │◄────────►│  中心主机  │◄────────►│ 右侧从机 │
      │(left_unit)│ hinge_L  │(base_link)│ hinge_R  │(right_unit)│
      │ servo_0  │          │  servo_1  │          │  servo_2  │
      │ motor_0  │          │  motor_1  │          │  motor_2  │
      └──────────┘          └──────────┘          └──────────┘
```

- **主机（Master, base_link）**：运行标准 PX4 固定翼控制栈，负责整体姿态和航线
- **从机（Slave, left_unit / right_unit）**：运行 `chainwing_slave` 模块，通过 PD 控制器调整 elevon 修正量，保持与主机的共面性（修正**相对滚转**）

### 1.2 物理参数

| 参数 | 值 | 来源 |
|------|-----|------|
| 铰链刚度 | 200 N·m/rad | model.sdf |
| 铰链阻尼 | 12 N·m·s/rad | model.sdf |
| 铰链极限 | ±15° (±0.262 rad) | model.sdf |
| 巡航速度 | 20 m/s | MATLAB 模型 |
| 总质量 | ~15.6 kg (三体) | MATLAB 模型 |

### 1.3 仿真环境

| 组件 | 说明 |
|------|------|
| 飞控 | PX4 Autopilot v1.14+ |
| 仿真器 | Gazebo Harmonic (gz-sim) |
| 世界文件 | `flat_terrain.sdf` (ENU 坐标系) |
| 模型 | `chainwing_3body` (3 刚体 + 2 铰链) |
| 机架编号 | 4008 (`4008_gz_chainwing_3body`) |
| 地面站 | QGroundControl (QGC) |

---

## §2 系统架构

### 2.1 总体控制架构

```
┌─────────────────────────────────────────────────────────────────┐
│                         PX4 Autopilot                          │
│                                                                 │
│  ┌───────────┐  ┌─────────────┐  ┌──────────────┐             │
│  │ Navigator  │→│ FW Pos Ctrl │→│ FW Att Ctrl   │             │
│  │ (航线管理) │  │ (位置控制)  │  │ (姿态控制)   │             │
│  └───────────┘  └─────────────┘  └──────┬───────┘             │
│                                          │                      │
│                                          ▼                      │
│                                  ┌──────────────┐              │
│                                  │FW Rate Ctrl  │              │
│                                  │(角速率控制)   │              │
│                                  └──────┬───────┘              │
│                                          │                      │
│                                          ▼                      │
│                                  ┌──────────────┐              │
│                                  │Control Alloc │              │
│                                  │(控制分配)     │              │
│                                  └──────┬───────┘              │
│                                          │                      │
│         ┌────────────────────────────────┤                      │
│         │                                │                      │
│         ▼                                ▼                      │
│  ┌─────────────┐              ┌──────────────────┐             │
│  │actuator_     │              │actuator_servos   │             │
│  │motors (uORB) │              │(uORB)            │             │
│  └──────┬──────┘              └────────┬─────────┘             │
│         │                               │                      │
│         ▼                               ▼                      │
│  ┌─────────────┐              ┌──────────────────┐             │
│  │GZ Mixing    │              │GZ Mixing Servo   │◄────┐      │
│  │Interface ESC│              │Interface         │     │      │
│  └──────┬──────┘              └────────┬─────────┘     │      │
│         │                               │               │      │
│         │                     ┌─────────┘               │      │
│         │                     │ + trim_left/right       │      │
│         │                     │                         │      │
│         │                     ▼                         │      │
│         │              ┌──────────────┐          ┌─────┴────┐ │
│         │              │ GZ Servo 0/1/2│          │chainwing │ │
│         │              │ (elevon+修正) │          │_slave    │ │
│         │              └──────────────┘          │(铰链PD)  │ │
│         │                                        └──────────┘ │
│         ▼                                                      │
│  ┌─────────────┐                                               │
│  │ GZ ESC 0/1/2│                                               │
│  │ (电机推力)  │                                               │
│  └─────────────┘                                               │
└─────────────────────────────────────────────────────────────────┘
```

### 2.2 chainwing_slave 模块在系统中的位置

`chainwing_slave` 模块是一个 **旁路修正器**：

1. 它 **不替代** 标准 PX4 飞控栈中的任何模块
2. 它 **额外计算** 铰链角度偏差的修正量 (`trim_left`, `trim_right`)
3. `GZMixingInterfaceServo` 在输出伺服信号时，将修正量 **叠加** 到左/右 elevon 上：

```
δ_total = clamp(δ_main + δ_trim, -1.0, 1.0)
```

### 2.3 硬件三机架构

在真实硬件上，三架飞机各自运行独立的 PX4 固件：

```
┌─────────────────┐  UART/MAVLink  ┌──────────────────┐  UART/MAVLink  ┌─────────────────┐
│   左侧从机       │◄─────────────►│    中心主机        │◄─────────────►│   右侧从机       │
│ MAV_SYS_ID=2    │  TELEM2       │ MAV_SYS_ID=1     │  SERIAL5      │ MAV_SYS_ID=3    │
│ CW_SLV_EN=1     │  /dev/ttyS2   │ CW_SLV_EN=0      │  /dev/ttyS6   │ CW_SLV_EN=1     │
│ CW_SLV_COMM_EN=1│  921600 bps   │ CW_MST_EN=1      │  921600 bps   │ CW_SLV_COMM_EN=1│
│ CW_SLV_PWM_EN=1 │               │ chainwing_master  │               │ CW_SLV_PWM_EN=1 │
└─────────────────┘               └──────────────────┘               └─────────────────┘
```

### 2.4 主机模块（chainwing_master）

`chainwing_master` 模块运行在 **中心主机** 上（仅限硬件三机模式），负责：

1. **发布 CW_CMD**（10 Hz）：将主机的姿态指令和 **当前 roll 姿态角** 发送给从机
2. **接收 CW_HINGE**：接收从机铰链状态反馈，用于遥测/日志

```
CW_CMD (id=43) 内容：
  data[0] = 主机 pitch 力矩指令 (vehicle_torque_setpoint.xyz[1])
  data[1] = 主机 throttle 指令 (vehicle_thrust_setpoint.xyz[0])
  data[2] = 主机 roll 力矩指令 (vehicle_torque_setpoint.xyz[0])
  data[3] = 主机当前 roll 姿态角 (euler.phi(), rad) ← 从机用此计算铰链角
```

**注意**：在单机 SITL 模式下 **不需要** 启用 `CW_MST_EN`。SITL 中只有一个 PX4 实例，
`chainwing_slave` 使用启动基准近似算法估计铰链角。

### 2.5 双模铰链角估计

从机根据通信状态自动选择铰链角计算方式：

| 模式 | 条件 | 公式 | 精度 |
|------|------|------|------|
| **硬件模式** | `CW_SLV_COMM_EN=1` + 收到有效 CW_CMD | `hinge_angle = slave_roll - master_roll_attitude` | 真实相对测量 |
| **SITL 模式** | `CW_SLV_COMM_EN=0` 或无主机数据 | `hinge_angle ≈ roll - _roll_ref (启动基准)` | 近似，单实例可用 |

---

## §3 通信架构详解

### 3.1 通信方式选型

| 方案 | 优点 | 缺点 | 选择 |
|------|------|------|------|
| 自定义 MAVLink 消息 | 最优结构 | 需 XML 定义+代码生成+驱动编写，工作量大 | ❌ |
| MAVLink NAMED_VALUE_FLOAT | 简单 | 每消息只1个 float，多字段需多消息 | ❌ |
| **DEBUG_FLOAT_ARRAY** | **58个float/包，PX4内置桥接** | ID/名称需约定 | ✅ **选用** |
| DroneCAN/UAVCAN | 硬件级可靠 | 需 CAN 总线硬件，SITL 难测试 | ❌ |

**选用 DEBUG_FLOAT_ARRAY 的核心原因：**

1. **零驱动开发**：PX4 已有完整的 uORB ↔ MAVLink 自动桥接
2. **容量充足**：每包 58 个 float，远超我们需要的 7 个（铰链）+ 3 个（指令）
3. **双向通信**：发送端 publish `debug_array` uORB → MAVLink 自动发出；接收端 MAVLink 收到 → 自动 publish `debug_array` uORB
4. **内置流控**：`custom` 模式 + `mavlink stream -s DEBUG_FLOAT_ARRAY -r 10` 精确控制速率

### 3.2 PX4 内置桥接机制

```
                     发送端（Slave）                     接收端（Master）
                     ───────────                        ─────────────
 chainwing_slave     ┌──────────┐                       ┌──────────┐
 publishDebugArray()─►debug_array│                       │debug_array├─► 主机应用程序
                     │  (uORB)  │                       │  (uORB)  │      订阅读取
                     └────┬─────┘                       └────▲─────┘
                          │                                   │
                          ▼                                   │
                 ┌────────────────┐                  ┌────────────────┐
                 │ MAVLink Stream │   UART 串口      │MAVLink Receiver│
                 │DEBUG_FLOAT_    │ ◄──921600bps──► │handle_message_ │
                 │ARRAY (10 Hz)  │   /dev/ttyS2     │debug_float_    │
                 └────────────────┘                  │array()         │
                                                     └────────────────┘
```

**关键代码路径：**

| 方向 | 步骤 | 文件 | 行号 |
|------|------|------|------|
| **发送** (uORB → MAVLink) | `_debug_array_pub.publish()` | ChainwingSlave.cpp | 241 |
| **发送** (MAVLink 流) | `DEBUG_FLOAT_ARRAY.hpp` | mavlink/streams/ | 自动 |
| **接收** (MAVLink → uORB) | `handle_message_debug_float_array()` | mavlink_receiver.cpp | 2813-2830 |
| **接收** (uORB → 应用) | `_debug_array_sub.update()` | ChainwingSlave.cpp | 251 |

### 3.3 MAVLink 模式选择：必须用 CUSTOM，不要用 ONBOARD

> ⚠️ **重要**: 多实例/硬件通信必须使用 `-m custom` 模式，**不要使用** `-m onboard`！

**`-m onboard` 的问题**: 自动包含 ODOMETRY@30Hz 流。ODOMETRY.hpp 硬编码
`estimator_type = MAV_ESTIMATOR_TYPE_AUTOPILOT (8)`，但 mavlink_receiver.cpp
只接受 VISION/VIO/MOCAP/NAIVE 类型。两个实例互连后，每秒产生 **60 条**
`WARN [mavlink] ODOMETRY: estimator_type 8 unsupported` 警告，刷屏导致无法操作。

**正确做法**: 使用 `-m custom`（不配置任何流）+ 手动添加 `DEBUG_FLOAT_ARRAY` 流：

```bash
# 步骤 1: 启动 MAVLink 实例（custom 模式 = 空流）
mavlink start -x -u 24550 -o 24551 -r 4000 -m custom

# 步骤 2: 仅添加需要的流（10 Hz 足够铰链通信）
mavlink stream -u 24550 -s DEBUG_FLOAT_ARRAY -r 10
```

**对比**:

| 模式 | ODOMETRY | DEBUG_FLOAT_ARRAY | 总流量 | 多实例安全 |
|------|----------|-------------------|--------|-----------|
| `-m onboard` | 30 Hz ❌ (刷屏) | 10 Hz | ~4000 B/s | ❌ |
| `-m custom` + stream | 无 ✅ | 10 Hz | ~200 B/s | ✅ |

**10 Hz 发送频率**意味着：
- `chainwing_slave` 以 50 Hz 发布 `debug_array` uORB 消息
- MAVLink 流每 100ms 取一次最新值发送
- 接收端获得 10 Hz 更新的铰链状态数据

---

## §4 通信协议定义

### 4.1 Slave → Master：铰链状态 (CW_HINGE)

| 字段 | 类型 | 说明 |
|------|------|------|
| **id** | uint16 | `42` (CW_HINGE_STATUS_ID) |
| **name** | char[10] | `"CW_HINGE"` |
| **data[0]** | float | `hinge_angle_left` — 左铰链角度 (rad) |
| **data[1]** | float | `hinge_angle_right` — 右铰链角度 (rad) |
| **data[2]** | float | `hinge_rate_left` — 左铰链角速率 (rad/s) |
| **data[3]** | float | `hinge_rate_right` — 右铰链角速率 (rad/s) |
| **data[4]** | float | `trim_left` — 左修正量 (归一化 [-1, 1]) |
| **data[5]** | float | `trim_right` — 右修正量 (归一化 [-1, 1]) |
| **data[6]** | float | `data_valid` — 数据有效标志 (1.0 或 0.0) |
| data[7-57] | float | 保留（未使用，值为 0.0） |

### 4.2 Master → Slave：主机指令 (CW_CMD)

| 字段 | 类型 | 说明 |
|------|------|------|
| **id** | uint16 | `43` (CW_MASTER_CMD_ID) |
| **name** | char[10] | `"CW_CMD"` |
| **data[0]** | float | `master_pitch_cmd` — 俯仰指令 (归一化 [-1, 1]) |
| **data[1]** | float | `master_throttle` — 油门指令 (归一化 [0, 1]) |
| **data[2]** | float | `master_roll_cmd` — 横滚指令 (归一化 [-1, 1]) |
| **data[3]** | float | `master_roll_attitude` — 主机当前横滚姿态角 (rad)，用于计算相对铰链角 |
| data[4-57] | float | 保留（未使用，值为 0.0） |

### 4.3 协议常量定义

```cpp
// ChainwingSlave.hpp:76-79
static constexpr uint16_t CW_HINGE_STATUS_ID = 42;    // Slave → Master
static constexpr uint16_t CW_MASTER_CMD_ID   = 43;    // Master → Slave
static constexpr hrt_abstime MASTER_CMD_TIMEOUT_US = 500000;  // 500ms 超时
```

### 4.4 协议时序图

```
   主机 (Master)                      从机 (Slave)
       │                                   │
       │         CW_CMD (id=43)            │
       │──────────────────────────────────►│ processMasterCommands()
       │   pitch=0.5, throttle=0.7,        │ → _master_pitch_cmd = 0.5
       │   roll=0.0, roll_att=0.02         │ → _master_throttle = 0.7
       │                                   │ → _master_roll_cmd = 0.0
       │                                   │ → _master_roll_attitude = 0.02
       │                                   │
       │         CW_HINGE (id=42)          │
       │◄──────────────────────────────────│ publishDebugArray()
       │   angle_L=-0.08, angle_R=0.08,   │
       │   rate_L=0.03, rate_R=-0.03,     │
       │   trim_L=-0.024, trim_R=0.024,   │
       │   valid=1.0                       │
       │                                   │
       │         100ms (10 Hz)             │
       │         ──────────►               │
       │                                   │
       │         CW_CMD (id=43)            │
       │──────────────────────────────────►│
       │                                   │
       │         CW_HINGE (id=42)          │
       │◄──────────────────────────────────│
       │                                   │
       │   ... 每 100ms 循环 ...           │
       │                                   │
       │                                   │
       │   如果 500ms 没收到 CW_CMD:       │
       │                                   │ → _master_cmd_valid = false
       │                                   │ → 自动切换到 SITL 近似模式
       │                                   │ → PX4_WARN("Master command timeout")
```

---

## §5 代码实现详解

### 5.1 文件结构

```
src/modules/chainwing_slave/
├── ChainwingSlave.hpp         ← 头文件：类定义、协议常量、成员变量
├── ChainwingSlave.cpp         ← 实现：Run()主循环、PD控制、通信
├── chainwing_slave_params.c   ← 参数定义：6个参数
├── CMakeLists.txt             ← 编译配置
└── Kconfig                    ← 编译菜单项
```

### 5.2 ChainwingSlave.hpp — 头文件核心结构

```cpp
class ChainwingSlave : public ModuleBase<ChainwingSlave>,
                       public ModuleParams,
                       public px4::ScheduledWorkItem
{
public:
    // ============ 通信协议常量 ============
    static constexpr uint16_t CW_HINGE_STATUS_ID = 42;   // 铰链状态 ID
    static constexpr uint16_t CW_MASTER_CMD_ID   = 43;   // 主机指令 ID
    static constexpr hrt_abstime MASTER_CMD_TIMEOUT_US = 500000; // 500ms 超时

private:
    // ============ 核心方法 ============
    void Run() override;                    // 50 Hz 主循环
    void updateHingeEstimate(float dt);     // IMU积分估计铰链角
    float computeTrim(float angle, float rate); // PD控制器计算修正量
    void publishDebugArray();               // 发布铰链状态到 MAVLink
    void processMasterCommands();           // 接收主机指令

    // ============ uORB 订阅/发布 ============
    uORB::Subscription _debug_array_sub{ORB_ID(debug_array)};       // 接收主机指令
    uORB::Publication<debug_array_s> _debug_array_pub{ORB_ID(debug_array)};  // 发送铰链状态
    uORB::Publication<chainwing_hinge_status_s> _hinge_status_pub{...};      // 本地铰链状态

    // ============ 主机指令状态 ============
    float _master_pitch_cmd{0.0f};    // 主机俯仰指令
    float _master_throttle{0.0f};     // 主机油门指令
    float _master_roll_cmd{0.0f};     // 主机横滚指令
    hrt_abstime _last_master_cmd{0};  // 上次收到指令的时间
    bool _master_cmd_valid{false};    // 指令是否在有效期内

    // ============ 参数 ============
    DEFINE_PARAMETERS(
        (ParamFloat<px4::params::CW_SLV_KP>)      _param_kp,         // PD 比例增益
        (ParamFloat<px4::params::CW_SLV_KD>)      _param_kd,         // PD 微分增益
        (ParamFloat<px4::params::CW_SLV_TRIM_MAX>) _param_trim_max,  // 最大修正量
        (ParamFloat<px4::params::CW_SLV_LP_FREQ>)  _param_lp_freq,  // 低通滤波频率
        (ParamInt<px4::params::CW_SLV_EN>)         _param_enable,    // 使能开关
        (ParamInt<px4::params::CW_SLV_COMM_EN>)    _param_comm_enable // 通信使能
    )
};
```

### 5.3 ChainwingSlave.cpp — 主循环 Run()

```cpp
void ChainwingSlave::Run()
{
    // 1. 参数更新检查
    if (_parameter_update_sub.updated()) {
        parameter_update_s param_update;
        _parameter_update_sub.copy(&param_update);
        updateParams();
    }

    // 2. 使能检查
    if (_param_enable.get() == 0) { return; }

    // 3. 计算时间步长 dt
    const float dt = math::constrain((now - _last_run) * 1e-6f, 0.001f, 0.1f);

    // 4. 更新铰链角度估计（IMU积分 + 互补滤波）
    updateHingeEstimate(dt);

    // 5. 计算 PD 修正量
    const float trim_left = computeTrim(_hinge_angle_left, _hinge_rate_left);
    const float trim_right = computeTrim(_hinge_angle_right, _hinge_rate_right);

    // 6. 发布 chainwing_hinge_status（本地 uORB）
    chainwing_hinge_status_s status{};
    status.trim_left = trim_left;
    status.trim_right = trim_right;
    _hinge_status_pub.publish(status);

    // 7. MAVLink 通信（仅当 CW_SLV_COMM_EN=1 时）
    if (_param_comm_enable.get() != 0) {
        publishDebugArray();       // 发送铰链状态给主机
        processMasterCommands();   // 接收主机指令
    }
}
```

### 5.4 publishDebugArray() — 铰链状态发送

```cpp
void ChainwingSlave::publishDebugArray()
{
    debug_array_s dbg{};
    dbg.timestamp = hrt_absolute_time();
    dbg.id = CW_HINGE_STATUS_ID;                    // id = 42
    strncpy(dbg.name, "CW_HINGE", sizeof(dbg.name)); // 名称标识
    dbg.name[sizeof(dbg.name) - 1] = '\0';

    dbg.data[0] = _hinge_angle_left;                 // 左铰链角度
    dbg.data[1] = _hinge_angle_right;                // 右铰链角度
    dbg.data[2] = _hinge_rate_left;                  // 左铰链角速率
    dbg.data[3] = _hinge_rate_right;                 // 右铰链角速率
    dbg.data[4] = computeTrim(_hinge_angle_left, _hinge_rate_left);   // 左修正量
    dbg.data[5] = computeTrim(_hinge_angle_right, _hinge_rate_right); // 右修正量
    dbg.data[6] = _ref_initialized ? 1.0f : 0.0f;   // 数据有效标志

    _debug_array_pub.publish(dbg);
    // → MAVLink 流自动将此 uORB 消息序列化为 DEBUG_FLOAT_ARRAY 并通过 UART 发送
}
```

### 5.5 processMasterCommands() — 主机指令接收

```cpp
void ChainwingSlave::processMasterCommands()
{
    debug_array_s cmd{};

    // 循环读取所有待处理的 debug_array 消息
    while (_debug_array_sub.update(&cmd)) {
        // 过滤：只处理 id=43 且名称为 "CW_CMD" 的消息
        if (cmd.id == CW_MASTER_CMD_ID && strncmp(cmd.name, "CW_CMD", 6) == 0) {
            // 输入验证：constrain 防止非法值
            _master_pitch_cmd = math::constrain(cmd.data[0], -1.0f, 1.0f);
            _master_throttle  = math::constrain(cmd.data[1],  0.0f, 1.0f);
            _master_roll_cmd  = math::constrain(cmd.data[2], -1.0f, 1.0f);
            _last_master_cmd  = hrt_absolute_time();
            _master_cmd_valid = true;
        }
    }

    // 超时检测：500ms 未收到主机指令则标记无效
    if (_master_cmd_valid && hrt_elapsed_time(&_last_master_cmd) > MASTER_CMD_TIMEOUT_US) {
        _master_cmd_valid = false;
        PX4_WARN("Master command timeout");
    }
}
```

### 5.6 updateHingeEstimate() — 铰链角度估计

```cpp
void ChainwingSlave::updateHingeEstimate(float dt)
{
    // 1. 读取 IMU 角速率
    vehicle_angular_velocity_s angular_vel{};
    if (!_vehicle_angular_velocity_sub.copy(&angular_vel)) { return; }

    // 2. 初始化参考姿态（首次有效姿态时）
    if (!_ref_initialized && attitude_valid) {
        _roll_ref = euler.phi();  // hinge axis = X = roll
        _ref_initialized = true;
    }

    // 3. 低通滤波角速率
    const float alpha = dt / (dt + 1.0f / (2.0f * M_PI_F * lp_freq));
    _hinge_rate_left  = (1-alpha) * _hinge_rate_left  + alpha * roll_rate;
    _hinge_rate_right = (1-alpha) * _hinge_rate_right + alpha * (-roll_rate);

    // 4. 积分 + 衰减（防止漂移）
    const float decay = expf(-dt / 2.0f);  // τ=2s
    _hinge_angle_left  = decay * (_hinge_angle_left  + _hinge_rate_left  * dt);
    _hinge_angle_right = decay * (_hinge_angle_right + _hinge_rate_right * dt);

    // 5. 互补滤波：姿态修正长期漂移
    const float cf_alpha = 0.02f;
    _hinge_angle_left  = (1-cf_alpha)*_hinge_angle_left  + cf_alpha*roll_error;
    _hinge_angle_right = (1-cf_alpha)*_hinge_angle_right + cf_alpha*(-roll_error);
}
```

### 5.7 computeTrim() — PD 控制器

```cpp
float ChainwingSlave::computeTrim(float angle, float rate)
{
    // PD 控制律：δ_trim = Kp × θ_hinge + Kd × θ̇_hinge
    float trim = _param_kp.get() * angle + _param_kd.get() * rate;

    // 限幅到最大修正量
    return math::constrain(trim, -_param_trim_max.get(), _param_trim_max.get());
}
```

### 5.8 GZMixingInterfaceServo — 修正量叠加

```cpp
// src/modules/simulation/gz_bridge/GZMixingInterfaceServo.cpp:56-99
bool GZMixingInterfaceServo::updateOutputs(...)
{
    // 读取铰链修正量
    chainwing_hinge_status_s hinge_status{};
    bool hinge_valid = _hinge_status_sub.copy(&hinge_status) && hinge_status.data_valid;

    for (auto &servo_pub : _servos_pub) {
        double output = (outputs[i] - 500) / 500.0;  // 标准 PX4 输出

        // 叠加修正量到左/右从机升降舵
        if (hinge_valid) {
            if (i == 0) { output += hinge_status.trim_left; }   // 左从机 servo_0
            if (i == 2) { output += hinge_status.trim_right; }  // 右从机 servo_2
            output = math::constrain(output, -1.0, 1.0);        // 限幅
        }

        servo_pub.Publish(output);
    }
}
```

---

## §6 参数说明

### 6.1 完整参数列表

| 参数名 | 类型 | 默认值 | 范围 | 说明 |
|--------|------|--------|------|------|
| `CW_SLV_EN` | INT32 | 0 | 0/1 | 从机控制器使能开关 |
| `CW_SLV_KP` | FLOAT | 0.3 | [0, 5] | PD 比例增益（无量纲） |
| `CW_SLV_KD` | FLOAT | 0.05 | [0, 2] | PD 微分增益（无量纲） |
| `CW_SLV_TRIM_MAX` | FLOAT | 0.3 | [0, 1] | 最大修正量（归一化行程） |
| `CW_SLV_LP_FREQ` | FLOAT | 10.0 | [0, 50] | 角速率低通滤波频率 (Hz) |
| `CW_SLV_COMM_EN` | INT32 | 0 | 0/1 | MAVLink 通信使能开关 |
| `CW_SLV_PWM_EN` | INT32 | 0 | 0/1 | 硬件 PWM trim 叠加使能（仅硬件模式） |
| `CW_MST_EN` | INT32 | 0 | 0/1 | 主机通信模块使能（仅主机 Pixhawk） |

### 6.2 参数分组

在 QGC 地面站中：
- **从机参数**: `CW_SLV_*` — Chain-Wing Slave 分组
- **主机参数**: `CW_MST_*` — Chain-Wing Master 分组

### 6.3 关键参数含义

**CW_SLV_EN (使能开关)**
- `0`：模块虽然运行，但 `Run()` 立即返回，不做任何计算
- `1`：正常工作，估计铰链角度并计算修正量

**CW_SLV_COMM_EN (通信使能)**
- `0`：仅使用 uORB 本地通信（适用于单机 SITL 仿真）
- `1`：启用 `publishDebugArray()` 和 `processMasterCommands()`，通过 MAVLink 进行双向通信（适用于硬件三机验证）。此模式下铰链角使用**真实相对 roll** 计算。

**CW_SLV_PWM_EN (硬件 PWM 叠加)**
- `0`：修正量仅通过 GZMixingInterfaceServo 叠加（SITL 模式）
- `1`：从机直接读取 actuator_servos，叠加 trim 后重新发布（硬件模式）

**CW_MST_EN (主机模块使能)**
- `0`：不发送 CW_CMD（单机模式/SITL）
- `1`：主机以 10 Hz 发布 CW_CMD，包含当前 roll 姿态角。**仅在三机分布式模式的主机上启用。**

**CW_SLV_KP (比例增益)**
- 5° 偏差时：trim = 0.3 × (5×π/180) = 0.026 (2.6% 行程)
- 10° 偏差时：trim = 0.3 × (10×π/180) = 0.052 (5.2% 行程)

---

## §7 SITL 仿真验证步骤

### 7.1 单实例验证（当前可用）

**目的**：验证 `chainwing_slave` 模块基本功能和通信接口

#### 步骤 1：启动仿真

```bash
# 终端 1：编译并启动
cd ~/PX4-Autopilot
make px4_sitl gz_chainwing_3body
```

#### 步骤 2：确认模块运行

```
# PX4 Shell (pxh>)
pxh> chainwing_slave status
```

预期输出：
```
Chain-wing slave controller
  Enabled: YES
  Communication: DISABLED
  Reference initialized: YES
  Hinge angles: left=-0.084 deg, right=0.084 deg
  Hinge rates:  left=0.032, right=-0.032 rad/s
  PD gains: Kp=0.30, Kd=0.05, max_trim=0.30
```

#### 步骤 3：启用通信

```
# PX4 Shell (pxh>)
pxh> param set CW_SLV_COMM_EN 1
```

#### 步骤 4：验证 debug_array 发布

```
# PX4 Shell (pxh>)
pxh> listener debug_array
```

预期输出（应看到 id=42 的消息）：
```
TOPIC: debug_array
    timestamp: 12345678
    id: 42
    name: CW_HINGE
    data: [-0.0844, 0.0844, 0.0317, -0.0317, -0.0237, 0.0237, 1.0000, ...]
```

#### 步骤 5：验证 chainwing_hinge_status 记录

```
# PX4 Shell (pxh>)
pxh> listener chainwing_hinge_status
```

预期输出：
```
TOPIC: chainwing_hinge_status
    timestamp: 12345678
    hinge_angle_left: -0.08435
    hinge_angle_right: 0.08435
    hinge_rate_left: 0.03173
    hinge_rate_right: -0.03173
    trim_left: -0.02372
    trim_right: 0.02372
    data_valid: True
```

#### 步骤 6：验证模块状态（启用通信后）

```
pxh> param set CW_SLV_COMM_EN 1
pxh> chainwing_slave status
```

预期输出新增通信信息：
```
  Communication: ENABLED
  Master cmd valid: NO
  Master pitch=0.00, throttle=0.00, roll=0.00
```

### 7.2 多实例验证（双PX4实例）

**目的**：验证两个 PX4 实例之间的 MAVLink 通信

#### 步骤 1：启动两个 PX4 实例

```bash
# 终端 1：实例 0（主机）
cd ~/PX4-Autopilot
PX4_SYS_AUTOSTART=4008 PX4_SIM_MODEL=chainwing_3body PX4_GZ_WORLD=flat_terrain \
  ./build/px4_sitl_default/bin/px4 -i 0

# 终端 2：实例 1（从机）
cd ~/PX4-Autopilot
PX4_SYS_AUTOSTART=4008 PX4_SIM_MODEL=chainwing_3body PX4_GZ_WORLD=flat_terrain \
  ./build/px4_sitl_default/bin/px4 -i 1
```

#### 步骤 2：建立 UDP 通信链路

> ⚠️ 必须使用 `-m custom`，**不要用** `-m onboard`（会导致 ODOMETRY 刷屏）

```
# 实例 0 (pxh>)  — 主机
pxh> mavlink start -x -u 24550 -o 24551 -r 4000 -m custom
pxh> mavlink stream -u 24550 -s DEBUG_FLOAT_ARRAY -r 10

# 实例 1 (pxh>)  — 从机
pxh> mavlink start -x -u 24551 -o 24550 -r 4000 -m custom
pxh> mavlink stream -u 24551 -s DEBUG_FLOAT_ARRAY -r 10
pxh> param set CW_SLV_COMM_EN 1
```

参数说明：
- `-x`：禁用 ftp 以减少流量
- `-u 24550`：本机监听 UDP 端口
- `-o 24551`：对方接收 UDP 端口
- `-r 4000`：速率 4000 B/s
- `-m custom`：空流模式（不含 ODOMETRY/ATTITUDE 等无用流）
- `mavlink stream -s DEBUG_FLOAT_ARRAY -r 10`：仅添加铰链通信所需的流

#### 步骤 3：从主机发送测试指令

在主机（实例 0）的 PX4 Shell 中手动发布测试指令：

```
# 目前主机端需要手动用 debug_array 发布 CW_CMD
# 未来可集成到主机控制栈中
```

#### 步骤 4：验证从机接收

```
# 实例 1 (pxh>)  — 从机
pxh> chainwing_slave status
```

如果主机发送了 CW_CMD (id=43)，将看到：
```
  Master cmd valid: YES
  Master pitch=0.50, throttle=0.70, roll=0.00
```

如果没收到（正常，因为主机端尚未集成发送代码）：
```
  Master cmd valid: NO
  Master pitch=0.00, throttle=0.00, roll=0.00
```

---

## §8 硬件验证步骤

### 8.1 硬件需求

| 设备 | 数量 | 说明 |
|------|------|------|
| Pixhawk 飞控板 | 2 | 推荐 Pixhawk 4/5X/6C |
| UART 串口线 | 1 | TX-RX 交叉连接 |
| USB 数据线 | 2 | 各自连接到电脑 |
| QGroundControl | 1 | 地面站软件 |

### 8.2 串口接线

```
┌───────────────┐              ┌───────────────┐
│   主机 Pixhawk │              │  从机 Pixhawk  │
│               │              │               │
│ TELEM2 (ttyS2)│              │ TELEM2 (ttyS2)│
│   TX ──────────────────────────── RX         │
│   RX ──────────────────────────── TX         │
│   GND ─────────────────────────── GND        │
│               │              │               │
│ USB ──► PC    │              │ USB ──► PC    │
│ (QGC连接)     │              │ (QGC连接)     │
└───────────────┘              └───────────────┘
```

**注意**：TX-RX 交叉！主机 TX → 从机 RX，主机 RX ← 从机 TX

### 8.3 固件烧录

两块 Pixhawk 烧录 **相同的固件**，仅参数不同：

```bash
# 编译硬件固件（以 Pixhawk 6C 为例）
make px4_fmu-v6c_default
# 或直接 make px4_fmu-v6c_default upload
```

### 8.4 主机参数配置

```
# QGC → Parameters → 搜索
MAV_SYS_ID = 1         # 系统 ID
CW_SLV_EN = 0          # 主机不启用从机控制器
CW_SLV_COMM_EN = 0     # 主机暂时不需要通信（未来需要时设为 1）
```

### 8.5 从机参数配置

```
# QGC → Parameters → 搜索
MAV_SYS_ID = 2         # 系统 ID（与主机不同）
CW_SLV_EN = 1          # 启用从机控制器
CW_SLV_COMM_EN = 1     # 启用 MAVLink 通信
CW_SLV_KP = 0.3        # PD 比例增益
CW_SLV_KD = 0.05       # PD 微分增益
CW_SLV_TRIM_MAX = 0.3  # 最大修正量
CW_SLV_LP_FREQ = 10.0  # 低通滤波频率
```

### 8.6 启动 MAVLink 串口通信

> ⚠️ 使用 `-m custom` 模式，不要用 `-m onboard`（会导致 ODOMETRY 刷屏）

```
# 从机 PX4 Shell (通过 QGC MAVLink Console 或 nsh)
pxh> mavlink start -d /dev/ttyS2 -b 921600 -m custom
pxh> mavlink stream -d /dev/ttyS2 -s DEBUG_FLOAT_ARRAY -r 10
```

参数说明：
- `-d /dev/ttyS2`：TELEM2 串口设备
- `-b 921600`：波特率 921600 bps
- `-m custom`：空流模式，避免发送 ODOMETRY 等无用消息
- `mavlink stream`：仅添加 DEBUG_FLOAT_ARRAY 流（10 Hz，铰链通信）

**如需永久生效**，在机架文件最后添加：
```bash
# 在 chainwing_slave start 之前
mavlink start -d /dev/ttyS2 -b 921600 -m custom
mavlink stream -d /dev/ttyS2 -s DEBUG_FLOAT_ARRAY -r 10
chainwing_slave start
```

### 8.7 验证通信

#### 从机端验证

```
pxh> chainwing_slave status
```

预期看到：
```
Chain-wing slave controller
  Enabled: YES
  Communication: ENABLED
  Reference initialized: YES
  Hinge angles: left=0.000 deg, right=0.000 deg
  ...
  Master cmd valid: NO        ← 正常（主机端尚未发送指令）
```

#### 验证 MAVLink 链路

```
pxh> mavlink status
```

检查 TELEM2 实例：
```
instance #1:
    ...
    mode: Onboard
    mavlink chan: #1
    type: serial (/dev/ttyS2)
    baudrate: 921600
    ...
    rate tx: 1234 B/s
    rate rx: 567 B/s      ← 如果有接收说明链路正常
```

#### 监控 debug_array

```
pxh> listener debug_array -n 5
```

应看到 id=42 的 CW_HINGE 消息以约 10 Hz 输出。

### 8.8 完整验证清单

| # | 验证项 | 命令 | 预期结果 | ✓ |
|---|--------|------|----------|---|
| 1 | 固件编译 | `make px4_sitl_default` | 无错误 | □ |
| 2 | 模块加载 | `chainwing_slave status` | 输出状态信息 | □ |
| 3 | 参数设置 | `param show CW_SLV*` | 6 个参数正确 | □ |
| 4 | 通信使能 | `param set CW_SLV_COMM_EN 1` | 无错误 | □ |
| 5 | debug_array 发布 | `listener debug_array` | 看到 id=42 | □ |
| 6 | hinge_status 发布 | `listener chainwing_hinge_status` | 看到数据 | □ |
| 7 | MAVLink 启动 | `mavlink start -d ... -m custom` | 无错误 | □ |
| 7b | 添加流 | `mavlink stream -d ... -s DEBUG_FLOAT_ARRAY -r 10` | 无错误 | □ |
| 8 | MAVLink 状态 | `mavlink status` | 显示 custom 实例 | □ |
| 9 | 串口收发 | `mavlink status` → rate rx | rx > 0 B/s | □ |
| 10 | 日志记录 | 飞行后检查 .ulg | 含 chainwing_hinge_status | □ |

---

## §9 数据流完整追踪

### 9.1 铰链修正量的完整路径

```
IMU (vehicle_angular_velocity)
    │
    ▼
ChainwingSlave::updateHingeEstimate()
    │ 积分 + 互补滤波
    ▼
_hinge_angle_left, _hinge_angle_right  (内部状态)
    │
    ▼
ChainwingSlave::computeTrim()
    │ PD 控制：δ_trim = Kp × θ + Kd × θ̇
    ▼
chainwing_hinge_status.trim_left / trim_right  (uORB)
    │
    ├──► GZMixingInterfaceServo::updateOutputs()
    │        │ output += hinge_status.trim_left (servo_0)
    │        │ output += hinge_status.trim_right (servo_2)
    │        ▼
    │    GZ servo_0/1/2 → 物理升降舵偏转
    │
    └──► publishDebugArray() [当 CW_SLV_COMM_EN=1]
             │ debug_array (id=42, "CW_HINGE")
             ▼
         MAVLink Stream → UART → 主机
```

### 9.2 主机指令的完整路径

```
主机 PX4 应用程序
    │ debug_array (id=43, "CW_CMD")
    ▼
MAVLink Stream → UART → 从机
    │
    ▼
mavlink_receiver.cpp::handle_message_debug_float_array()
    │ MAVLink → debug_array uORB
    ▼
ChainwingSlave::processMasterCommands()
    │ 过滤 id=43 + "CW_CMD"
    │ math::constrain() 输入验证
    ▼
_master_pitch_cmd, _master_throttle, _master_roll_cmd  (内部状态)
    │
    ▼
（当前版本尚未使用这些值，预留给未来控制整合）
```

---

## §10 Logger 飞行日志记录

### 10.1 自动记录

`chainwing_hinge_status` 已添加到 PX4 Logger 的可选话题列表：

```cpp
// src/modules/logger/logged_topics.cpp:57
add_optional_topic("chainwing_hinge_status", 100);
```

- **自动记录**：当 `chainwing_slave` 模块运行时，`.ulg` 日志文件自动包含铰链数据
- **记录频率**：最高 100 Hz（实际取决于模块发布频率 50 Hz）
- **无需手动操作**

### 10.2 日志分析

```bash
# 1. 找到 .ulg 文件
# SITL: build/px4_sitl_default/rootfs/log/
# 硬件: SD卡 /fs/microsd/log/

# 2. 使用 pyulog 转 CSV
pip install pyulog
ulog2csv your_flight.ulg

# 3. 查看生成的 CSV 文件
ls *.csv | grep chainwing
# → your_flight_chainwing_hinge_status_0.csv

# 4. 使用 PlotJuggler 可视化
# 打开 PlotJuggler → 拖入 .ulg 文件 → 选择 chainwing_hinge_status
```

### 10.3 Flight Review 在线分析

上传 `.ulg` 到 https://review.px4.io/ — 但注意自定义话题（chainwing_hinge_status）不会在标准图表中显示，需要使用 PlotJuggler 本地分析。

---

## §11 文件清单

### 11.1 新增文件

| 文件路径 | 说明 | 行数 |
|----------|------|------|
| `src/modules/chainwing_slave/ChainwingSlave.hpp` | 从机模块头文件：类定义、协议常量、成员变量 | 179 |
| `src/modules/chainwing_slave/ChainwingSlave.cpp` | 从机模块实现：主循环、PD控制、通信接口 | 436 |
| `src/modules/chainwing_slave/chainwing_slave_params.c` | 7 个 CW_SLV_* 参数定义 | 174 |
| `src/modules/chainwing_slave/CMakeLists.txt` | CMake 编译配置 | 42 |
| `src/modules/chainwing_slave/Kconfig` | 编译菜单项 | 6 |
| `src/modules/chainwing_master/ChainwingMaster.hpp` | 主机模块头文件：类定义 | 128 |
| `src/modules/chainwing_master/ChainwingMaster.cpp` | 主机模块实现：10Hz CW_CMD 发布 + CW_HINGE 接收 | 270 |
| `src/modules/chainwing_master/chainwing_master_params.c` | CW_MST_EN 参数定义 | 59 |
| `src/modules/chainwing_master/CMakeLists.txt` | CMake 编译配置 | 42 |
| `src/modules/chainwing_master/Kconfig` | 编译菜单项 | 6 |
| `msg/ChainwingHingeStatus.msg` | uORB 消息定义（铰链状态） | 16 |
| `ROMFS/.../4008_gz_chainwing_3body` | SITL 机架文件 | 232 |
| `ROMFS/.../2150_chainwing` | 硬件机架文件（含 master + slave 配置） | 136 |
| `scripts/param_sweep_sitl.sh` | 自动化参数扫描 Bash 脚本 | 250 |
| `scripts/analyze_param_sweep.py` | 日志分析 + 对比图生成 Python 脚本 | 600 |
| `scripts/sweep_config.json` | 22 轮参数扫描配置 | 362 |
| `scripts/README.md` | 脚本使用说明 | 232 |

### 11.2 修改文件

| 文件路径 | 修改内容 |
|----------|----------|
| `boards/px4/sitl/default.px4board` | 添加 `CONFIG_MODULES_CHAINWING_SLAVE=y` + `CONFIG_MODULES_CHAINWING_MASTER=y` |
| `boards/px4/fmu-v3/default.px4board` | 添加 `CONFIG_MODULES_CHAINWING_SLAVE=y` + `CONFIG_MODULES_CHAINWING_MASTER=y` |
| `msg/CMakeLists.txt` | 注册 `ChainwingHingeStatus.msg` |
| `src/modules/logger/logged_topics.cpp` | 添加 `chainwing_hinge_status` 可选话题 |
| `src/modules/simulation/gz_bridge/GZMixingInterfaceServo.cpp` | 读取 hinge_status 并叠加 trim |

### 11.3 GZ 仿真模型文件

| 文件路径 | 说明 |
|----------|------|
| `Tools/simulation/gz/models/chainwing_3body/model.sdf` | 三体模型：base_link + left_unit + right_unit + 2 铰链 |
| `Tools/simulation/gz/worlds/flat_terrain.sdf` | 平地世界文件 (ENU 坐标系) |

---

## §12 故障排除

### 12.1 常见问题

| 问题 | 原因 | 解决方案 |
|------|------|----------|
| `chainwing_slave: command not found` | 模块未编译 | 检查 `default.px4board` 中 `CONFIG_MODULES_CHAINWING_SLAVE=y` |
| `listener chainwing_hinge_status` 2秒超时 | 模块未运行或 CW_SLV_EN=0 | `chainwing_slave start` + `param set CW_SLV_EN 1` |
| `listener debug_array` 无 id=42 消息 | CW_SLV_COMM_EN=0 | `param set CW_SLV_COMM_EN 1` |
| MAVLink `rate rx: 0 B/s` | 串口接线错误或波特率不匹配 | 检查 TX-RX 交叉接线，确认双方 `-b 921600` |
| `Master command timeout` | 主机未发送 CW_CMD | 正常（主机未启动或 CW_MST_EN=0） | 在主机上 `param set CW_MST_EN 1` 并重启 |
| listener 打印太快（几秒打完） | SITL 锁步模式：sim time ≠ wall clock | 使用 `gz topic` 或 logger+PlotJuggler 替代 |
| 编译报 `Invalid unit` | 参数 @unit 不在允许列表 | 增益参数不加 @unit（无量纲） |

### 12.2 诊断命令速查

```bash
# 模块状态
pxh> chainwing_slave status

# 参数检查
pxh> param show CW_SLV*

# uORB 消息检查
pxh> listener chainwing_hinge_status
pxh> listener debug_array

# MAVLink 链路状态
pxh> mavlink status

# 系统总览
pxh> top            # 查看模块是否在运行
pxh> uorb top       # 查看 uORB 话题发布频率
```

---

> **文档结束**
>
> 本文档完整描述了链翼无人机主从机通信系统的设计、实现和验证方法。
> 所有代码均基于 PX4 内置的 DEBUG_FLOAT_ARRAY MAVLink 消息桥接机制，
> 零自定义驱动开发，可直接用于串口+MAVLink 硬件通信验证。

---

## §13 修改历史记录

### 13.1 铰链参数三次迭代

| 版本 | 日期 | 刚度 k (N·m/rad) | 阻尼 c (N·m·s/rad) | 极限 | Kp | Kd | 结果 |
|------|------|:--:|:--:|:--:|:--:|:--:|------|
| v1（初始） | 2026-03-20 | 500 | 50 | ±5° | 0.3 | 0.05 | ❌ 弹簧太硬，控制器仅4.9%作用 |
| v2（首次优化） | 2026-03-22 | 50 | 5 | ±10° | 2.0 | 0.3 | ❌ ωn=1.36Hz与控制器耦合，飞行不稳 |
| **v3（当前）** | 2026-03-22 | **200** | **12** | **±15°** | **1.5** | **0.2** | ⏳ ωn=2.72Hz，ζ=0.51，控制器占38% |

**v3 推导依据**：report.pdf §3.2.1 聚氨酯弹性体物理参数 + 频率匹配缩放法。

### 13.2 MAVLink 模式修正

| 修改 | 原值 | 新值 | 原因 |
|------|------|------|------|
| MAVLink 模式 | `-m onboard` | `-m custom` | onboard 含 ODOMETRY@30Hz，estimator_type=8 被拒绝，产生刷屏警告 |
| 流配置 | 无（依赖默认） | `mavlink stream -s DEBUG_FLOAT_ARRAY -r 10` | 仅发送需要的数据流 |

**根因**：`ODOMETRY.hpp:140` 硬编码 `MAV_ESTIMATOR_TYPE_AUTOPILOT(8)`，`mavlink_receiver.cpp:1429-1453` 不支持该类型。

**正确命令**：
```bash
# SITL 多实例
mavlink start -x -u 24550 -o 24551 -r 4000 -m custom
mavlink stream -u 24550 -s DEBUG_FLOAT_ARRAY -r 10

# 硬件 UART
mavlink start -d /dev/ttyS2 -b 921600 -m custom
mavlink stream -d /dev/ttyS2 -s DEBUG_FLOAT_ARRAY -r 10
```

### 13.3 模块自启动

| 修改 | 文件 | 说明 |
|------|------|------|
| 添加 `chainwing_slave start` | 4008_gz_chainwing_3body:228 | 模块随机架文件自动启动，无需手动输入 |
| 添加 `chainwing_hinge_status` | logged_topics.cpp:57 | 铰链状态自动记录到 .ulg 飞行日志 |



### 13.4 CW 模块关闭行为确认

当 `CW_SLV_EN=0` 时：
1. `ChainwingSlave::Run()` 立即返回（line 88-91）
2. 不发布 `chainwing_hinge_status`
3. `GZMixingInterfaceServo` 的 `copy()` 返回 false → `hinge_valid=false`
4. trim 修正块被跳过 → 舵面输出纯控制分配器值

**结论**：关闭 CW 模块 = 修正完全不起作用，舵面为标准 PX4 输出。

### 13.5 v2.0 更新汇总

| 变更类型 | 文件 | 描述 |
|----------|------|------|
| **BUG修复** | ChainwingSlave.cpp | 互补滤波器 theta()→phi()，pitch→roll |
| **BUG修复** | ChainwingSlave.hpp | _pitch_ref → _roll_ref |
| **BUG修复** | ChainwingHingeStatus.msg | 注释：elevator→elevon, nose up→relative roll |
| 参数优化 | model.sdf | k=500→200, c=50→12, ±5°→±15° |
| 参数优化 | chainwing_slave_params.c | KP=0.3→1.5, KD=0.05→0.2 |
| 功能新增 | ChainwingSlave.cpp | MAVLink DEBUG_FLOAT_ARRAY 通信接口 |
| 功能新增 | chainwing_slave_params.c | CW_SLV_COMM_EN 参数 |
| 配置修复 | 4008_gz_chainwing_3body | chainwing_slave start 自启动 |
| 配置修复 | logged_topics.cpp | chainwing_hinge_status 日志记录 |
| MAVLink修复 | 文档+注释 | -m onboard → -m custom |

### 13.6 v3.0 更新汇总

| 变更类型 | 文件 | 描述 |
|----------|------|------|
| **功能新增** | `src/modules/chainwing_master/` | 新增 chainwing_master 模块（10Hz CW_CMD 发布） |
| **协议升级** | ChainwingSlave.cpp | CW_CMD data[3] = master_roll_attitude (rad) |
| **双模铰链** | ChainwingSlave.cpp | 硬件模式: slave_roll - master_roll; SITL: roll - _roll_ref |
| **参数新增** | chainwing_master_params.c | CW_MST_EN 主机使能开关 |
| **参数新增** | chainwing_slave_params.c | CW_SLV_PWM_EN 硬件 PWM 叠加 |
| **配置更新** | fmu-v3/default.px4board | 添加 CONFIG_MODULES_CHAINWING_MASTER=y |
| **机架更新** | 2150_chainwing | 添加 CW_MST_EN + master UART 配置 + 双模块自启动 |
| **自动化脚本** | scripts/ | 22 轮参数扫描 + 日志分析 + 对比图生成 |

---

## §14 自动化参数扫描脚本

### 14.1 概述

项目提供了自动化参数扫描工具，可以自动遍历多组参数组合，运行 GZ SITL 仿真，保存日志，并生成对比分析图。

**位于**: `scripts/` 目录

| 文件 | 功能 |
|------|------|
| `param_sweep_sitl.sh` | Bash 编排脚本：启动/参数设置/飞行/保存日志 |
| `analyze_param_sweep.py` | Python 分析脚本：读取 .ulg 日志 → 生成 8 种图表 + CSV |
| `sweep_config.json` | 扫描配置文件：22 轮 × 9 阶段 × 10 个参数 |
| `README.md` | 脚本详细使用说明 |

### 14.2 安装依赖

```bash
# 系统工具
sudo apt install jq

# Python 分析依赖
pip3 install pyulog matplotlib numpy

# PX4 编译（如未编译）
cd PX4_test
DONT_RUN=1 make px4_sitl_default gz_chainwing_3body
```

### 14.3 运行参数扫描

```bash
# 运行全部 22 轮（约 66 分钟）
bash scripts/param_sweep_sitl.sh

# 只运行特定轮次
bash scripts/param_sweep_sitl.sh --run 5

# 只运行某阶段（例如 Yaw P 扫描）
bash scripts/param_sweep_sitl.sh --phase B_yaw_P

# 使用自定义配置文件
bash scripts/param_sweep_sitl.sh my_config.json
```

### 14.4 分析日志

```bash
# 生成 PNG 图表
python3 scripts/analyze_param_sweep.py sweep_logs/

# 指定输出目录
python3 scripts/analyze_param_sweep.py sweep_logs/ --output results/

# 生成 PDF 报告
python3 scripts/analyze_param_sweep.py sweep_logs/ --pdf
```

### 14.5 扫描阶段设计（22 轮 × 9 阶段）

| 阶段 | 名称 | 轮次 | 扫描参数 | 扫描值 |
|------|------|------|----------|--------|
| A | baseline | 1 | — | PX4 默认值，CW_SLV_EN=0 |
| B | yaw_P | 2-4 | FW_YR_P | 0.15 → 0.3 → 0.6 |
| C | yaw_I | 5-6 | FW_YR_I | 0.2 → 0.5 |
| D | yaw_FF | 7-8 | FW_YR_FF | 0.5 → 0.7 |
| E | yaw_D | 9-11 | FW_YR_D | 0.005 → 0.01 → 0.02 |
| F | heading_hold | 12-13 | FW_YAW_STAB_SC | 1.0 → 2.0 |
| G | roll_P | 14-16 | FW_RR_P | 0.15 → 0.3 → 0.5 |
| H | pitch_P | 17-19 | FW_PR_P | 0.2 → 0.5 → 0.9 |
| I | hinge | 20-22 | CW_SLV_KP/KD | 1.0/0.1 → 1.5/0.2 → 2.0/0.3 |

**设计原则**：每阶段只改一个参数，先调好的参数锁定后继续下一阶段。

### 14.6 输出文件

| 文件 | 内容 |
|------|------|
| `01_attitude_comparison.png` | 多轮 Roll/Pitch/Yaw 对比时间序列 |
| `02a_yaw_rate_tracking.png` | 各轮偏航角速率跟踪对比 |
| `02b_roll_rate_tracking.png` | 各轮滚转角速率跟踪对比 |
| `02c_pitch_rate_tracking.png` | 各轮俯仰角速率跟踪对比 |
| `03_servo_output.png` | 舵面输出对比 |
| `04_hinge_correction.png` | 铰链修正效果对比 |
| `05_metrics_comparison.png` | 9 项性能指标柱状图 |
| `06_metrics_table.png` | 性能汇总表格图 |
| `metrics_summary.csv` | CSV 格式汇总数据 |

### 14.7 性能指标说明

| 指标 | 说明 | 越小越好 |
|------|------|:--------:|
| Yaw Rate RMS (°/s) | 偏航角速率跟踪误差均方根 | ✅ |
| Roll Rate RMS (°/s) | 滚转角速率跟踪误差均方根 | ✅ |
| Pitch Rate RMS (°/s) | 俯仰角速率跟踪误差均方根 | ✅ |
| Roll RMS (°) | 滚转角偏差均方根 | ✅ |
| Pitch RMS (°) | 俯仰角偏差均方根 | ✅ |
| Yaw Deviation RMS (°) | 偏航角偏差均方根 | ✅ |
| Servo Activity | 舵面变化率（越低越平滑） | ✅ |
| Hinge Angle RMS (°) | 铰链角偏差均方根 | ✅ |
| Max Hinge Angle (°) | 最大铰链角 | ✅ |
