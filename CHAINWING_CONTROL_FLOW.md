# 链翼无人机（Chain-Wing UAV）控制流程详解

本文档详细阐述了链翼无人机在 PX4 固件中的所有修改内容、修改原因，以及完整的控制流程。

---

## 一、链翼无人机的核心控制难题

链翼无人机由三架固定翼飞机通过翼尖半柔性连接形成大展弦比组合体。这种构型带来两个核心控制难题：

### 1.1 横滚轴稳定性弱
- **原因**：多机翼尖连接处刚度较弱，受扰动后各飞行单元容易产生相对滚转运动
- **表现**：左侧飞行单元向左倾斜时，右侧飞行单元可能向右倾斜，导致整体横滚振荡
- **传统方法不适用**：单个飞行单元没有副翼，无法用传统副翼控制横滚

### 1.2 偏航轴稳定性弱
- **原因**：三机组合后质量分布在很长的展向上（1.8m翼展），绕偏航轴的转动惯量极大
- **表现**：侧向投影面积小、无方向舵，航向保持能力差
- **传统方法不适用**：标准PX4偏航控制器只做协调转弯（无侧滑），不主动保持航向

---

## 二、完整的控制流程图

```
┌──────────────────────────────────────────────────────────────────┐
│                    传感器输入 (Sensor Input)                       │
│  IMU → 姿态解算 → roll, pitch, yaw (欧拉角)                       │
│  磁罗盘 → 航向角 heading                                         │
│  气压计 → 高度                                                    │
│  空速管 → airspeed                                               │
└─────────────────────┬────────────────────────────────────────────┘
                      │
                      ▼
┌──────────────────────────────────────────────────────────────────┐
│              姿态设定点生成 (Attitude Setpoint)                    │
│                                                                  │
│  增稳模式 (STABILIZED):                                           │
│    roll_sp  = 横滚摇杆 × FW_MAN_R_MAX                            │
│    pitch_sp = 俯仰摇杆 × FW_MAN_P_MAX + FW_PSP_OFF(4.5°)        │
│    yaw_sp   = ┌─ FW_YAW_STAB_SC > 0: 锁定的航向设定点 ◄──── 新增 │
│               └─ FW_YAW_STAB_SC = 0: 当前航向(不控制)             │
│                                                                  │
│  自主模式 (AUTO):                                                 │
│    由导航控制器 (NPFG/L1) 计算姿态设定点                            │
└─────────────────────┬────────────────────────────────────────────┘
                      │
                      ▼
┌──────────────────────────────────────────────────────────────────┐
│             姿态控制器 (Attitude Controller)                       │
│             fw_att_control 模块                                   │
│                                                                  │
│  Roll Controller:                                                │
│    角速率设定点 = f(roll_error, FW_R_TC)                           │
│                                                                  │
│  Pitch Controller:                                               │
│    角速率设定点 = f(pitch_error, FW_P_TC)                          │
│                                                                  │
│  Yaw Controller (ecl_yaw_controller): ◄─────────────── 修改处     │
│    ① 协调转弯: yaw_rate = tan(roll) × cos(pitch) × g / V         │
│    ② 航向保持(新增): ◄─────────────────────────────── 新增逻辑     │
│       heading_error = wrap_pi(yaw_setpoint - yaw)                │
│       yaw_rate += heading_error × FW_YAW_STAB_SC                │
│       yaw_rate = constrain(yaw_rate, ±FW_Y_RMAX)                │
│                                                                  │
│  输出: body_rates_setpoint [roll_rate, pitch_rate, yaw_rate]      │
└─────────────────────┬────────────────────────────────────────────┘
                      │
                      ▼
┌──────────────────────────────────────────────────────────────────┐
│             角速率控制器 (Rate Controller)                         │
│             fw_rate_control 模块                                  │
│                                                                  │
│  Roll Rate PID  → roll_torque  (横滚力矩指令)                     │
│  Pitch Rate PID → pitch_torque (俯仰力矩指令)                     │
│  Yaw Rate PID   → yaw_torque  (偏航力矩指令)                      │
│  Throttle       → thrust       (油门指令)                         │
│                                                                  │
│  输出: [roll_torque, pitch_torque, yaw_torque, thrust]            │
└─────────────────────┬────────────────────────────────────────────┘
                      │
                      ▼
┌──────────────────────────────────────────────────────────────────┐
│             控制分配器 (Control Allocator)                         │
│             control_allocator 模块                                │
│                                                                  │
│  效能矩阵 (Effectiveness Matrix):                                 │
│  基于 CA_ROTOR*_PY 位置参数自动计算:                               │
│                                                                  │
│  ┌─────────┬──────────┬──────────┬──────────┐                    │
│  │ 力矩\电机│ 左(Y=-0.6)│ 中(Y=0.0) │ 右(Y=+0.6)│                 │
│  ├─────────┼──────────┼──────────┼──────────┤                    │
│  │横滚力矩  │    0      │    0      │    0      │                 │
│  │俯仰力矩  │    0      │    0      │    0      │                 │
│  │偏航力矩  │ ct×(-0.6) │  ct×(0.0) │ ct×(+0.6) │ ◄── 差动推力   │
│  │推力      │    ct     │    ct     │    ct     │                 │
│  └─────────┴──────────┴──────────┴──────────┘                    │
│                                                                  │
│  moment = ct × position.cross(axis)                              │
│  左电机: position=(-0.6) → 偏航力矩 = ct × (-0.6)                │
│  右电机: position=(+0.6) → 偏航力矩 = ct × (+0.6)                │
│                                                                  │
│  → 偏航指令 > 0 时: 右电机加速 + 左电机减速 → 向右偏航              │
│  → 偏航指令 < 0 时: 左电机加速 + 右电机减速 → 向左偏航              │
│                                                                  │
│  舵面效能矩阵 (基于 CA_SV_CS*_TRQ_* 参数):                        │
│  ┌─────────┬─────────────┬──────────┬──────────────┐             │
│  │ 力矩\舵面│ 左升降副翼    │ 中升降舵  │ 右升降副翼    │             │
│  │         │ (右elevon)   │(elevator)│ (左elevon)   │             │
│  ├─────────┼─────────────┼──────────┼──────────────┤             │
│  │横滚力矩  │   +0.5      │   0.0    │   -0.5      │             │
│  │俯仰力矩  │   +0.5      │   +1.0   │   +0.5      │             │
│  │偏航力矩  │    0.0      │   0.0    │    0.0      │             │
│  └─────────┴─────────────┴──────────┴──────────────┘             │
│                                                                  │
│  → 左升降副翼(CS0) = 右升降副翼类型: 向上偏转 → 抬头 + 向右滚       │
│  → 右升降副翼(CS2) = 左升降副翼类型: 向下偏转 → 低头 + 向右滚       │
│  → 两者协同: 横滚扰动时产生差动力矩，自动回正                       │
│                                                                  │
│  输出: [motor0_cmd, motor1_cmd, motor2_cmd,                       │
│         servo0_cmd, servo1_cmd, servo2_cmd]                       │
└─────────────────────┬────────────────────────────────────────────┘
                      │
                      ▼
┌──────────────────────────────────────────────────────────────────┐
│             执行器 (Actuators)                                     │
│                                                                  │
│  电机0(左): 转速 → 推力 (+ 偏航差动)                               │
│  电机1(中): 转速 → 推力 (基准)                                     │
│  电机2(右): 转速 → 推力 (+ 偏航差动)                               │
│  舵机0: 左飞行单元升降副翼偏角                                      │
│  舵机1: 中飞行单元升降舵偏角                                        │
│  舵机2: 右飞行单元升降副翼偏角                                      │
└──────────────────────────────────────────────────────────────────┘
```

---

## 三、逐文件修改详解

### 3.1 偏航控制器 — `ecl_yaw_controller.h` + `ecl_yaw_controller.cpp`

#### 修改内容
```cpp
// ecl_yaw_controller.h — 新增：
void set_heading_hold_gain(float gain) { _heading_hold_gain = gain; }
float _heading_hold_gain{0.f};

// ecl_yaw_controller.cpp — 在协调转弯计算之后新增：
if (_heading_hold_gain > FLT_EPSILON &&
    PX4_ISFINITE(ctl_data.yaw_setpoint) && PX4_ISFINITE(ctl_data.yaw)) {
    const float heading_error = wrap_pi(ctl_data.yaw_setpoint - ctl_data.yaw);
    const float heading_rate_correction = heading_error * _heading_hold_gain;
    _body_rate_setpoint += math::constrain(heading_rate_correction, -_max_rate, _max_rate);
    _body_rate_setpoint = math::constrain(_body_rate_setpoint, -_max_rate, _max_rate);
}
```

#### 为什么要改

**PX4 原生偏航控制器只做协调转弯：**

```
yaw_rate = tan(roll) × cos(pitch) × g / airspeed
```

这个公式的含义是：飞机横滚时，为了保持无侧滑飞行（零侧力），需要一定的偏航角速率。这对于有副翼+方向舵的普通飞机足够了，因为航向会由横滚控制间接维持。

**但链翼无人机不一样：**
- 偏航转动惯量是普通飞机的3-5倍（质量分布在1.8m展向上）
- 没有方向舵，侧向力很小
- 仅靠协调转弯无法抵抗侧风等偏航扰动

**我们的解决方案：**

在协调转弯输出的基础上，叠加一个航向误差P控制器：

```
总偏航速率 = 协调转弯偏航速率 + 航向误差 × FW_YAW_STAB_SC
```

这与报告3.3.3节描述的"偏航轴自稳控制算法"完全对应：

> "飞行器偏航轴自稳控制算法实时解算飞行器目标航向与实际航向的差异，得出所需的纠偏舵量"

#### 对应报告数据
- 报告描述：PID控制器，持续接收磁罗盘航向角反馈与陀螺仪偏航角速率反馈
- PX4实现：P控制器（航向误差→偏航速率修正），D部分由下游的角速率控制器提供

---

### 3.2 姿态控制器 — `FixedwingAttitudeControl.cpp` + `.hpp`

#### 修改内容（三处）

**① 增稳模式航向设定点锁定 (第108-119行)**
```cpp
if (_param_fw_yaw_stab_sc.get() > FLT_EPSILON) {
    if (!_heading_setpoint_initialized) {
        _heading_setpoint = yaw_body;  // 锁定当前航向
        _heading_setpoint_initialized = true;
    }
    _att_sp.yaw_body = _heading_setpoint;  // 用锁定的航向作为目标
} else {
    _att_sp.yaw_body = yaw_body;  // 原始行为：不控制偏航
}
```

**为什么要改：**

PX4原生 `STABILIZED` 模式中：
```cpp
_att_sp.yaw_body = yaw_body;  // 偏航设定点 = 当前航向
```
这意味着偏航设定点**始终等于当前航向**——即飞控从不试图纠正偏航偏差。这对普通飞机没问题（它们通过横滚控制航向），但链翼无人机需要一个固定的航向目标来追踪。

**② 偏航摇杆更新航向设定点 (第390-396行)**
```cpp
if (_param_fw_yaw_stab_sc.get() > FLT_EPSILON) {
    const float yaw_stick = _manual_control_setpoint.yaw;
    if (fabsf(yaw_stick) > 0.05f) {
        _heading_setpoint += yaw_stick * radians(_param_fw_y_rmax.get()) * dt;
        _heading_setpoint = wrap_pi(_heading_setpoint);
    }
} else {
    // 原始行为：直接叠加偏航角速率
    body_rates_setpoint(2) += ...;
}
```

**为什么要改：**

原生PX4中，偏航摇杆直接叠加偏航角速率。但在航向保持模式下，应该是**积分式的航向修改**：
- 摇杆归中 → 航向目标不变 → 飞机保持当前航向
- 摇杆偏转 → 航向目标以一定速率旋转 → 飞机跟随转弯
- 摇杆回中 → 航向目标锁定在新方向 → 飞机保持新航向

0.05的死区防止摇杆微小抖动导致航向漂移。

**③ 着陆/模式切换时重置 (第312行)**
```cpp
_heading_setpoint_initialized = false;
```

确保模式切换或着陆后，重新锁定航向。

---

### 3.3 新参数 — `fw_att_control_params.c`

```c
PARAM_DEFINE_FLOAT(FW_YAW_STAB_SC, 0.0f);  // 默认关闭
```

**设计理由：**
- `= 0`：所有新增代码走 `else` 分支，行为与原始PX4完全一致
- `= 2.0`（链翼推荐值）：1弧度航向误差 → 2 rad/s偏航速率修正
- 这是控制链翼增稳功能的唯一开关

---

### 3.4 机架配置 — `2150_chainwing` (硬件) + `4007_gz_chainwing` (仿真)

#### 三电机差动推力配置

```sh
CA_ROTOR_COUNT 3
CA_ROTOR0_PY -0.6   # 左单元
CA_ROTOR1_PY  0.0   # 中间单元
CA_ROTOR2_PY  0.6   # 右单元
```

**工作原理：** PX4控制分配器中的关键公式：
```cpp
moment = ct * position.cross(axis)
```

当三个电机沿Y轴分布时，`position.cross(Z轴)` 产生偏航力矩：
- 左电机(Y=-0.6): 偏航效能 = ct × (-0.6)
- 右电机(Y=+0.6): 偏航效能 = ct × (+0.6)

控制分配器自动求解：偏航指令 → 左右电机转速差 → 差动推力力矩

**对应报告3.3.3节：**
> "电机差动推力控制算法将偏航通道的控制指令经混控系数缩放后并行输出至油门通道...通过独立调节两侧电机的转速，产生绕组合体垂直轴的偏航控制力矩"

#### 升降副翼反向配置

```sh
# 左飞行单元 → 右升降副翼类型
CA_SV_CS0_TYPE 6    # RightElevon
CA_SV_CS0_TRQ_R 0.5

# 右飞行单元 → 左升降副翼类型
CA_SV_CS2_TYPE 5    # LeftElevon
CA_SV_CS2_TRQ_R -0.5
```

**对应报告表3.3的工作原理：**

当链翼组合体受扰动**向左滚转**时：
1. PX4控制器输出**正的横滚力矩**修正指令
2. 左单元的右升降副翼(TRQ_R=+0.5)：向上偏转 → 产生**抬头力矩** → 左翼尖向上（向右回正）✓
3. 右单元的左升降副翼(TRQ_R=-0.5)：向下偏转 → 产生**低头力矩** → 右翼尖向下（向右回正）✓
4. 两侧差动力矩协同将整机回正

> 报告原文："左侧飞行单元舵面向上偏转，获得抬头力矩使飞行单元向右回正；右侧飞行单元舵面向下偏转，获得低头力矩使飞行单元向右回正"

---

### 3.5 GZ仿真模型 — `model.sdf`

#### 关键组件与参数

| 组件 | 参数来源 | 值 |
|------|---------|-----|
| 总质量 | 报告：单机0.5kg × 3 | 1.5 kg |
| 总翼展 | 报告：单机0.6m × 3 | 1.8 m |
| 翼弦 | 报告：AR=3.333 | 0.18 m |
| 零升迎角 | NACA 5412翼型 | ~0.08 rad |
| 升力线斜率 | 报告XFLR5分析 | 4.75 /rad |
| 失速迎角 | 报告数据 | 13° ≈ 0.227 rad |
| 配平迎角 | 报告：升阻比最大处 | 4.5° |

#### 惯性矩修正（第二次提交）

```
修正前: Iyy=0.000167, Izz=0.000168
修正后: Iyy=0.000166704, Izz=0.000167604
```

**原因**：Gazebo要求惯性张量满足三角不等式 `Ixx + Iyy ≥ Izz`。修正前 `9.75e-7 + 0.000167 = 0.000167975 < 0.000168`，差 `2.5e-8`，Gazebo拒绝创建模型。修正后使用rc_cessna的精确值，满足不等式。

---

## 四、控制闭环完整数据流

以"侧风导致航向偏右5°"为例：

```
1. 传感器检测: yaw = 95° (实际), yaw_setpoint = 90° (目标)
                          │
2. 航向误差计算:          │
   heading_error = wrap_pi(90° - 95°) = -5° = -0.087 rad
                          │
3. 偏航速率修正:          │
   yaw_rate_correction = -0.087 × 2.0 = -0.174 rad/s (向左偏航)
                          │
4. 协调转弯 + 航向保持:   │
   total_yaw_rate = coordinated_turn_rate + (-0.174)
                          │
5. 角速率控制器:          │
   yaw_torque = PID(total_yaw_rate - actual_yaw_rate)
                          │
6. 控制分配:              │
   yaw_torque < 0 → 需要向左偏航
   → 左电机(Y=-0.6) 加速: motor0 += |yaw_torque| / 0.6
   → 右电机(Y=+0.6) 减速: motor2 -= |yaw_torque| / 0.6
   → 中电机不变
                          │
7. 差动推力效果:          │
   左推力 > 右推力 → 组合体绕垂直轴向左转 → 航向回到90°
```

---

## 五、与报告的对应关系总结

| 报告描述 | PX4实现 | 修改文件 |
|---------|---------|---------|
| 辅助增稳控制 | PX4原生姿态控制 + 我们的航向保持 | `FixedwingAttitudeControl.cpp` |
| 升降副翼反向配置(表3.3) | `CA_SV_CS0_TYPE=6, CS2_TYPE=5` | `2150_chainwing` |
| 电机差动推力控制 | `CA_ROTOR*_PY` 位置 → 自动差动 | `2150_chainwing` |
| 偏航轴自稳PID控制器 | 航向误差P控制 + 角速率PID | `ecl_yaw_controller.cpp` |
| 增稳模式偏航控制 | 航向设定点锁定 + 摇杆积分 | `FixedwingAttitudeControl.cpp` |
| 参数化开关 | `FW_YAW_STAB_SC`=0关闭/=2.0启用 | `fw_att_control_params.c` |

---

## 六、向后兼容性保证

**所有修改均由 `FW_YAW_STAB_SC` 参数控制：**

- `FW_YAW_STAB_SC = 0`（默认）: 所有新增代码走 `else` 分支，行为与未修改的PX4完全一致
- `FW_YAW_STAB_SC > 0`（链翼模式）: 启用航向保持 + 差动推力偏航控制
- 机架文件为可选配置，不影响其他机型

---

## 七、GZ 仿真 + QGC 调参完整指南

**可以！** PX4 SITL 仿真天然支持 QGC 连接。下面是从零开始的完整步骤。

### 7.1 环境准备

#### 方式A: WSL2 (Windows 用户推荐)

```bash
# 1. 确保使用 WSL2 + Ubuntu 22.04
wsl --install -d Ubuntu-22.04

# 2. 在 WSL2 中安装 PX4 工具链
cd ~/PX4_test   # 或你的 PX4 源码目录
bash Tools/setup/ubuntu.sh

# 3. 安装 Gazebo Garden
sudo apt-get update
sudo apt-get install gz-garden

# 4. 如果 WSL2 中没有 GUI (用于 Gazebo 画面)
# 需要安装 WSLg 或 X Server (Windows 11 自带 WSLg)
# Windows 10 需要安装 VcXsrv 或 X410:
#   export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0
```

#### 方式B: 原生 Ubuntu 22.04

```bash
# 1. 安装 PX4 工具链
cd ~/PX4_test
bash Tools/setup/ubuntu.sh

# 2. 安装 Gazebo Garden
sudo apt-get install gz-garden
```

#### QGC 安装

```bash
# 方式1: 在 Windows 上安装 QGC (WSL 用户推荐)
# 从 https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html
# 下载 Windows 版 QGC 安装包

# 方式2: 在 Linux 上安装 QGC
sudo usermod -a -G dialout $USER
sudo apt-get install fuse libfuse2
# 下载 QGC.AppImage 并运行
```

### 7.2 启动仿真

```bash
# 在 PX4 源码目录下执行:
cd ~/PX4_test
make px4_sitl gz_chainwing
```

这条命令会：
1. ✅ 编译 PX4 SITL 固件（含链翼增稳代码）
2. ✅ 启动 Gazebo，加载 `default` 世界 + `chainwing` 模型
3. ✅ 启动 PX4 飞控，自动加载 `4007_gz_chainwing` 机架配置
4. ✅ **自动启动 MAVLink UDP 广播到端口 14550**（QGC 默认监听端口）

启动成功后你会看到 PX4 shell 提示符 `pxh>`

### 7.3 连接 QGC

#### WSL 用户 (QGC 在 Windows 上)

**QGC 会自动连接！** PX4 SITL 会广播 MAVLink 到 UDP 14550，QGC 默认监听此端口。

1. 先在 WSL 中启动仿真: `make px4_sitl gz_chainwing`
2. 在 Windows 上打开 QGroundControl
3. QGC 应该在几秒内自动检测到飞行器并连接

**如果自动连接失败**（WSL 网络隔离问题）：

```bash
# 在 WSL 中查看 WSL IP 地址:
ip addr show eth0 | grep "inet "
# 输出类似: inet 172.xx.xx.xx/20

# 然后在 QGC 中手动添加连接:
# QGC → 应用设置(齿轮图标) → 通讯连接 → 添加
#   类型: UDP
#   监听端口: 14550
#   添加目标主机: 172.xx.xx.xx:18570
#   (其中 172.xx.xx.xx 是 WSL 的 IP)
```

#### 原生 Linux 用户

1. 先在终端中启动仿真: `make px4_sitl gz_chainwing`
2. 在同一台机器上打开 QGC
3. QGC 自动连接（localhost UDP 14550）

### 7.4 QGC 中的参数调整

连接成功后，在 QGC 中:

**参数页面入口**: 点击顶部齿轮图标 → **参数 (Parameters)**

#### 7.4.1 链翼核心参数 (搜索关键词: `FW_`)

| 参数名 | 含义 | 默认值 | 链翼推荐值 | 调整建议 |
|--------|------|-------|-----------|---------|
| **FW_YAW_STAB_SC** | 偏航航向保持增益 | 0.0 | **2.0** | 增大→更强航向保持，过大会振荡 |
| **FW_R_TC** | 横滚时间常数 | 0.4 | 0.4 | 减小→响应更快，过小会振荡 |
| **FW_P_TC** | 俯仰时间常数 | 0.4 | 0.4 | 减小→响应更快，过小会振荡 |
| **FW_Y_RMAX** | 最大偏航速率 | 50 | **30** | 限制偏航速率避免结构过载 |
| **FW_PSP_OFF** | 俯仰配平偏移 | 0.0 | **4.5** | 报告中的最佳配平迎角 |

#### 7.4.2 角速率控制器增益 (搜索关键词: `FW_RR` / `FW_PR` / `FW_YR`)

| 参数名 | 含义 | 链翼推荐值 | 调整方法 |
|--------|------|-----------|---------|
| **FW_RR_P** | 横滚速率P增益 | 0.3 | 横滚振荡→减小; 响应慢→增大 |
| **FW_RR_I** | 横滚速率I增益 | 0.5 | 稳态偏差→增大; 超调→减小 |
| **FW_RR_FF** | 横滚速率前馈 | 0.5 | 提高响应速度 |
| **FW_PR_P** | 俯仰速率P增益 | 0.9 | 俯仰振荡→减小; 响应慢→增大 |
| **FW_PR_I** | 俯仰速率I增益 | 0.5 | 稳态偏差→增大 |
| **FW_PR_FF** | 俯仰速率前馈 | 0.5 | 提高响应速度 |
| **FW_YR_P** | 偏航速率P增益 | 0.6 | 偏航振荡→减小; 航向偏→增大 |
| **FW_YR_I** | 偏航速率I增益 | 0.5 | 稳态航向偏差→增大 |
| **FW_YR_FF** | 偏航速率前馈 | 0.5 | 提高航向响应 |

#### 7.4.3 油门与空速 (搜索关键词: `FW_THR` / `FW_AIRSPD`)

| 参数名 | 含义 | 链翼推荐值 | 说明 |
|--------|------|-----------|-----|
| **FW_THR_TRIM** | 巡航油门 | 0.25 | 平飞所需油门 |
| **FW_THR_MAX** | 最大油门 | 0.6 | 限制最大推力 |
| **FW_THR_MIN** | 最小油门 | 0.05 | 怠速 |
| **FW_AIRSPD_TRIM** | 巡航空速 | 12 m/s | 报告数据 |
| **FW_AIRSPD_MIN** | 最小空速 | 8 m/s | 低于此减速报警 |
| **FW_AIRSPD_STALL** | 失速空速 | 6 m/s | 低于此失速 |

#### 7.4.4 控制分配参数 (搜索关键词: `CA_`)

| 参数名 | 含义 | 值 | **不建议随意修改** |
|--------|------|---|---|
| CA_ROTOR_COUNT | 电机数量 | 3 | 固定 |
| CA_ROTOR0_PY | 左电机Y位置 | -0.6 | 决定差动推力力臂 |
| CA_ROTOR2_PY | 右电机Y位置 | 0.6 | 决定差动推力力臂 |
| CA_SV_CS0_TRQ_R | 左升降副翼横滚效能 | 0.5 | 反向配置关键 |
| CA_SV_CS2_TRQ_R | 右升降副翼横滚效能 | -0.5 | 反向配置关键 |

### 7.5 仿真飞行测试步骤

#### 第一步: 基本检查

```bash
# 在 PX4 shell (pxh>) 中:
commander status          # 查看飞控状态
param show FW_YAW_STAB_SC  # 确认航向保持增益
param show FW_PSP_OFF       # 确认俯仰偏移
listener vehicle_attitude   # 查看实时姿态数据
```

#### 第二步: 手动起飞 (STABILIZED 模式)

```bash
# 方式1: PX4 shell 命令
commander mode stabilized   # 切换到增稳模式
commander arm               # 解锁
commander takeoff            # 起飞

# 方式2: 在 QGC 中
# 点击左上角飞行模式 → 选择 "Stabilized"
# 滑动底部解锁滑块
# 点击 "起飞" 按钮
```

#### 第三步: 观察飞行状态

在 QGC 中观察:
- **姿态指示器**: 横滚/俯仰是否稳定
- **航向指示器**: 航向是否保持（偏航增稳效果）
- **空速显示**: 是否在 8-20 m/s 范围内
- **地图视图**: 航迹是否直线

#### 第四步: 实时调参测试

1. 在 QGC **参数页面** 修改 `FW_YAW_STAB_SC`:
   - 设为 0 → 关闭航向保持 → 观察航向是否漂移
   - 设为 1.0 → 轻度航向保持
   - 设为 2.0 → 标准链翼航向保持
   - 设为 3.0 → 强航向保持 → 观察是否偏航振荡

2. 测试横滚稳定性:
   - 观察升降副翼反向配置效果
   - 在 QGC 飞行数据 → 分析视图中查看 `vehicle_attitude.roll` 曲线

3. 测试差动推力:
   - 在 QGC 分析视图中查看 `actuator_outputs.output[0]` 到 `output[2]`
   - 偏航修正时应看到左右电机输出差异

#### 第五步: 自主飞行测试

```bash
# 在 QGC 中:
# 1. 切换到 "计划" 视图
# 2. 在地图上点击添加航点
# 3. 上传任务
# 4. 切换到 Mission 模式
commander mode auto:mission
```

### 7.6 调参工作流建议

```
第一轮: 基本飞行
├── 确认能起飞、平飞、不坠毁
├── 如果俯仰振荡 → 减小 FW_PR_P
├── 如果横滚振荡 → 减小 FW_RR_P
└── 如果偏航振荡 → 减小 FW_YR_P 或 FW_YAW_STAB_SC

第二轮: 航向保持
├── 增稳模式下松开摇杆，观察航向是否保持
├── 航向漂移 → 增大 FW_YAW_STAB_SC (从 1.0 开始逐步增大)
├── 偏航振荡 → 减小 FW_YAW_STAB_SC 或 FW_YR_P
└── 航向保持良好但转弯迟钝 → 增大 FW_Y_RMAX

第三轮: 精细调整
├── 空速保持 → 调整 FW_THR_TRIM
├── 高度保持 → 调整 FW_T_CLMB_MAX / FW_T_SINK_MAX
└── 自主航线跟踪 → 调整 NPFG_PERIOD / NAV_ACC_RAD
```

### 7.7 常见问题排查

| 问题 | 原因 | 解决方案 |
|------|------|---------|
| QGC 无法连接 | 网络/端口问题 | 检查防火墙；WSL 用户手动添加 UDP 连接 |
| Gazebo 中无飞机模型 | 惯性矩错误(已修复) | 确保使用最新代码（含惯性矩修正） |
| 起飞后立即坠毁 | 油门/俯仰参数 | 检查 FW_THR_TRIM、FW_PSP_OFF |
| 飞机持续偏航 | 航向保持未启用 | 确认 FW_YAW_STAB_SC > 0 |
| 横滚振荡 | 增益过大 | 减小 FW_RR_P 和 FW_RR_I |
| 偏航振荡 | 航向增益过大 | 减小 FW_YAW_STAB_SC |
| 参数修改后没效果 | 需要重启 | 部分参数需要 `reboot` 后生效 |
| Gazebo 界面卡顿 | GPU/WSL 性能 | 尝试 `export LIBGL_ALWAYS_SOFTWARE=1` |

### 7.8 QGC 中的实时监控视图

在 QGC **分析 (Analyze)** 页面可以实时监控以下数据:

```
vehicle_attitude.roll     — 实时横滚角（观察横滚稳定性）
vehicle_attitude.pitch    — 实时俯仰角（观察俯仰稳定性）
vehicle_attitude.yaw      — 实时偏航角（观察航向保持效果）
vehicle_rates.roll        — 横滚角速率
vehicle_rates.pitch       — 俯仰角速率
vehicle_rates.yaw         — 偏航角速率（观察差动推力效果）
airspeed.true_airspeed_m_s — 空速
actuator_outputs.output[0] — 左电机输出
actuator_outputs.output[1] — 中电机输出
actuator_outputs.output[2] — 右电机输出（对比左右电机看差动推力）
```

### 7.9 多机仿真 (可选)

如果需要在同一仿真中测试多架链翼无人机:

```bash
# 第一架 (实例 0, MAVLink 端口 14550)
PX4_SYS_AUTOSTART=4007 PX4_GZ_MODEL=chainwing ./build/px4_sitl_default/bin/px4 -i 0

# 第二架 (实例 1, MAVLink 端口 14551)
PX4_SYS_AUTOSTART=4007 PX4_GZ_MODEL=chainwing ./build/px4_sitl_default/bin/px4 -i 1

# 在 QGC 中:
# 应用设置 → 通讯连接 → 添加 → UDP → 端口 14551
# 这样可以同时监控两架飞机
```

---

## 八、EKF2 姿态估计警告诊断报告

### 8.1 问题现象

在 GZ 仿真中，链翼无人机在爬升阶段出现以下警告：
- EKF2 认为横滚/俯仰姿态数据不可信
- 表现为 `estimator_status` 中的 innovation test ratio 超限

### 8.2 诊断结论

| 假设 | 结论 | 说明 |
|------|------|------|
| PX4代码太老？ | ❌ 不是 | EKF2工作正常，警告是对异常数据的合理反应 |
| IMU噪声/延迟？ | ❌ 不是主因 | IMU配置(250Hz, 无噪声)与正常工作的rc_cessna相同 |
| 惯性参数不匹配？ | ⚠️ 部分原因 | Iyy偏大1.74×，三角不等式余量极小(0.02) |
| 非典型气动力矩？ | ✅ **根本原因** | 气动模型a0参数错误导致飞机在爬升时失速 |

### 8.3 根本原因详解

#### 原因 #1（最关键）：气动模型 `a0` 参数错误

`model.sdf` 中三个机翼段的 LiftDrag 插件 `<a0>` 值设置为 `+0.08 rad (+4.58°)`。

**但 NACA 5412 翼型的零升迎角应该是 `-4°` 左右（约 `-0.07 rad`）。**

Gazebo LiftDrag 插件中，升力系数公式为：
```
CL = cla × (alpha - a0)
```

当前配置的后果：
```
配平迎角 4.5° 时:
  CL = 4.75 × (0.0785 - 0.08) = -0.007 → 几乎零升力！

平飞所需迎角: ~11° (距失速角13°仅2.1°裕度)

任何爬升 → 迎角超过13° → 失速 → 升力骤降 → 加速度突变
→ EKF2预测与IMU数据严重偏离 → 触发姿态不可信警告
```

如果修正为 `a0 = -0.07`：
```
配平迎角 4.5° 时:
  CL = 4.75 × (0.0785 + 0.07) = 0.706 → 正常升力
  Lift = 19.8 N > Weight 14.7 N ✓

平飞所需迎角: ~2.3° (距失速角13°有10.7°裕度)

爬升10° → 迎角12.3° → 仍在失速角以内 ✓
```

**对比已知能正常工作的 rc_cessna 模型：**
```
rc_cessna: 失速裕度 14.3° → 爬升无问题
chainwing: 失速裕度  2.1° → 任何爬升立即失速
```

#### 原因 #2：水平尾翼面积过小

```
当前尾翼面积:  0.020 m² (占机翼面积 6.2%)
正常飞机:      15-30% 的机翼面积
```

尾翼面积不足导致俯仰控制力矩弱，爬升时无法有效限制俯仰角增大，加速进入失速。

#### 原因 #3：惯性参数精度不足

```
              配置值    估算值    偏差
Ixx (横滚):   0.30      0.41     偏小 (0.74×)
Iyy (俯仰):   0.10      0.06     偏大 (1.74×)
Izz (偏航):   0.38      0.46     偏小 (0.82×)
```

- Iyy 偏大 1.74 倍 → 物理引擎中俯仰响应比控制器预期慢
- 三角不等式 `Ixx + Iyy ≥ Izz` 余量仅 0.02，接近物理极限

### 8.4 建议修改方案

#### 优先级 1：修正 `a0` 参数（修复根本原因）

**文件**: `Tools/simulation/gz/models/chainwing/model.sdf`

将所有 3 个机翼段 LiftDrag 插件中的 `<a0>` 值从 `0.08` 改为 `-0.07`：

```xml
<!-- 修改前 -->
<a0>0.08</a0>

<!-- 修改后 -->
<a0>-0.07</a0>
```

需要修改的位置（3处机翼LiftDrag插件，不包括水平尾翼）。

#### 优先级 2：增大水平尾翼面积

**文件**: `Tools/simulation/gz/models/chainwing/model.sdf`

```xml
<!-- 修改前 -->
<area>0.020</area>

<!-- 修改后 -->
<area>0.050</area>
```

仅修改水平尾翼的 LiftDrag 插件（第4个，cp 在 `-0.50 0 0` 的那个）。

#### 优先级 3：修正惯性参数

**文件**: `Tools/simulation/gz/models/chainwing/model.sdf`

```xml
<!-- 修改前 -->
<ixx>0.30</ixx>
<iyy>0.10</iyy>
<izz>0.38</izz>

<!-- 修改后 -->
<ixx>0.40</ixx>
<iyy>0.06</iyy>
<izz>0.45</izz>
```

#### 优先级 4：EKF2 参数调优（治标方案）

在 QGC 或 PX4 shell 中设置：
```bash
param set EKF2_ACC_NOISE 0.5     # 默认0.35, 增大容忍度
param set EKF2_GYR_NOISE 0.02    # 默认0.015
param set EKF2_ACC_B_NOISE 0.005  # 默认0.003
```

或添加到 `4007_gz_chainwing` 机架文件中：
```bash
param set-default EKF2_ACC_NOISE 0.5
param set-default EKF2_GYR_NOISE 0.02
param set-default EKF2_ACC_B_NOISE 0.005
```

> ⚠️ 这只是降低 EKF2 的灵敏度，应先修正上面的根本原因。

#### 优先级 5：限制爬升角度

在 `4007_gz_chainwing` 中添加：
```bash
param set-default FW_T_CLMB_MAX 5    # 降低最大爬升率
param set-default FW_P_LIM_MAX 15     # 限制最大仰角
```
