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
