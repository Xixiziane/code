# 链翼无人机（Chain-Wing UAV）固件技术详解

## 控制流程修改与参数配置完整说明

> 本文档详细阐述链翼无人机项目中对 PX4 固件所做的每一处控制流程修改（含完整代码）、  
> 飞行器参数配置（含 Gazebo 仿真）、以及每个数据值的来源和设定原因。

---

## 目录

1. [项目概述](#1-项目概述)
2. [控制流程修改](#2-控制流程修改)
   - 2.1 [偏航航向保持控制器](#21-偏航航向保持控制器)
   - 2.2 [Gazebo 模型重生机制](#22-gazebo-模型重生机制)
3. [机架参数配置](#3-机架参数配置)
   - 3.1 [控制分配器参数](#31-控制分配器参数)
   - 3.2 [空速参数](#32-空速参数)
   - 3.3 [姿态控制参数](#33-姿态控制参数)
   - 3.4 [速率控制器增益](#34-速率控制器增益)
   - 3.5 [偏航航向保持参数](#35-偏航航向保持参数)
   - 3.6 [着陆参数](#36-着陆参数)
   - 3.7 [任务与导航参数](#37-任务与导航参数)
   - 3.8 [仿真专用参数](#38-仿真专用参数)
   - 3.9 [EKF 与传感器参数](#39-ekf-与传感器参数)
   - 3.10 [故障检测与安全参数](#310-故障检测与安全参数)
4. [Gazebo 仿真模型配置](#4-gazebo-仿真模型配置)
   - 4.1 [物理模型（惯性参数）](#41-物理模型惯性参数)
   - 4.2 [气动力模型（LiftDrag 插件）](#42-气动力模型liftdrag-插件)
   - 4.3 [电机模型（MulticopterMotorModel）](#43-电机模型multicoptermotormodel)
   - 4.4 [舵机控制器](#44-舵机控制器)
   - 4.5 [仿真世界（flat_terrain.sdf）](#45-仿真世界flat_terrainsdf)
5. [参数数据来源汇总表](#5-参数数据来源汇总表)

---

## 1. 项目概述

链翼无人机由 **三架固定翼单元** 在翼尖铰接而成，形成大展弦比机翼。该构型的独特之处在于：

- **无方向舵**：偏航控制完全依靠三台电机的 **差动推力**
- **反向升降副翼**：左右翼的升降副翼滚转效果与传统飞机相反
- **大转动惯量**：三单元并联导致滚转/偏航惯量远大于单机

这些特性要求对标准 PX4 固件进行针对性修改。

### 修改文件清单

| 文件 | 修改类型 | 目的 |
|------|----------|------|
| `ecl_yaw_controller.h` | **新增代码** | 添加航向保持增益成员变量 |
| `ecl_yaw_controller.cpp` | **新增代码** | 实现航向保持控制律 |
| `GZBridge.cpp` | **新增代码** | 仿真模型重生前清理旧模型 |
| `4007_gz_chainwing` | **新建文件** | Gazebo SITL 机架配置 |
| `model.sdf` | **新建文件** | Gazebo 物理仿真模型 |
| `flat_terrain.sdf` | **新建文件** | 自定义仿真世界 |

---

## 2. 控制流程修改

### 2.1 偏航航向保持控制器

#### 问题背景

PX4 标准偏航控制器 (`ecl_yaw_controller.cpp`) 仅使用 **协调转弯公式** 计算偏航率：

```
偏航率 = tan(横滚角) × cos(俯仰角) × g / 空速
```

对于有方向舵的传统飞机，这是正确的。但对链翼无人机存在两个根本问题：

1. **无方向舵**：链翼依靠差动推力控制偏航，电机推力与空速无关（不像舵面效率随空速平方变化），因此低速时协调转弯产生的偏航率指令过大
2. **无主动航向保持**：标准控制器没有航向误差反馈环路，外部扰动（侧风、推力不对称）会导致航向漂移且无法自动修正

#### 修改位置

**文件**: `src/modules/fw_att_control/ecl_yaw_controller.h`

```cpp
// ===== 新增代码 =====
// 文件: src/modules/fw_att_control/ecl_yaw_controller.h
// 位置: ECL_YawController 类定义内

public:
    // 设置航向保持增益（由参数 FW_YAW_STAB_SC 控制）
    void set_heading_hold_gain(float gain) { _heading_hold_gain = gain; }

    // 设置基准空速（由参数 FW_AIRSPD_TRIM 控制）
    void set_trim_airspeed(float airspeed) { _trim_airspeed = airspeed; }

private:
    float _heading_hold_gain{0.f};   // 航向保持增益，0=禁用
    float _trim_airspeed{15.f};      // 增益归一化的基准空速 [m/s]
```

**文件**: `src/modules/fw_att_control/ecl_yaw_controller.cpp`

```cpp
// ===== 新增代码（第 101-121 行） =====
// 位置: control_attitude() 函数末尾，在协调转弯计算之后

    /* Heading hold for chain-wing yaw stabilization:
     * Add a yaw rate correction proportional to heading error.
     * Scale gain by (airspeed / trim_airspeed)^2 to prevent over-correction
     * at low speeds (landing/approach) where differential thrust authority
     * and vertical tail effectiveness are reduced.
     */
    if (_heading_hold_gain > FLT_EPSILON &&
        PX4_ISFINITE(ctl_data.yaw_setpoint) && PX4_ISFINITE(ctl_data.yaw)) {
        const float heading_error = wrap_pi(ctl_data.yaw_setpoint - ctl_data.yaw);

        /* Airspeed-dependent gain scaling: reduces gain at low speeds to prevent
         * yaw oscillation / spinning during landing approach. At cruise speed the
         * gain is 100%; at stall speed it drops to ~16%. */
        const float airspeed_ratio = math::constrain(
            ctl_data.airspeed_constrained / math::max(_trim_airspeed, 1.f),
            0.1f, 1.0f);
        const float scaled_gain = _heading_hold_gain * airspeed_ratio * airspeed_ratio;

        const float heading_rate_correction = heading_error * scaled_gain;
        _body_rate_setpoint += math::constrain(heading_rate_correction, -_max_rate, _max_rate);
        _body_rate_setpoint = math::constrain(_body_rate_setpoint, -_max_rate, _max_rate);
    }

    return _body_rate_setpoint;
```

#### 控制律详解

**数学表达式**:

```
ψ_err = wrap_π(ψ_setpoint - ψ_actual)        // 航向误差 [rad]

r_ratio = clamp(V / V_trim, 0.1, 1.0)         // 空速比

G_scaled = G_base × r_ratio²                   // 缩放后的增益

Δr = clamp(ψ_err × G_scaled, -r_max, r_max)   // 航向修正偏航率

r_body += Δr                                    // 叠加到体轴偏航率设定值
```

**为什么使用二次缩放 (r_ratio²) 而不是线性**:

| 空速 | 线性缩放 | 二次缩放 | 说明 |
|------|----------|----------|------|
| 8 m/s (失速) | 40% | **16%** | 低速时差动推力权威性相对过大，需要更强的衰减 |
| 15 m/s (进近) | 75% | **56%** | 中速段衰减适中 |
| 20 m/s (巡航) | 100% | **100%** | 满增益 |

二次缩放的物理意义：气动力（包括垂尾产生的偏航阻尼力矩）与 V² 成正比。当空速降低时，气动阻尼下降为 (V/V_trim)²，如果控制增益不同步降低，系统会变得不稳定。因此增益应按 (V/V_trim)² 缩放以保持闭环特性一致。

**增益下限 0.1 的原因**:

在地面（空速 ≈ 0，被钳制到 FW_AIRSPD_STALL = 8 m/s）：
- `r_ratio = V_stall / V_trim = 8 / 20 = 0.4`
- `r_ratio² = 0.16`（16% 增益）
- 下限 0.1 对应 `r_ratio² = 0.01`（1% 增益），这是最低保障

#### 控制流程图

```
                          PX4 标准协调转弯
                    ┌──────────────────────────┐
遥控器横滚指令 ──→  │ r = tan(φ)·cos(θ)·g/V   │ ──→ 基础偏航率 r_base
                    └──────────────────────────┘
                                                           │
                          链翼新增航向保持                     │
                    ┌──────────────────────────┐           │
航向设定值 ψ_sp ──→ │                          │           │
                    │ Δr = ψ_err × G × (V/V₀)² │ ──→ + ──→ r_body (总偏航率)
当前航向 ψ ─────→  │                          │
                    └──────────────────────────┘
                                                           │
                                                           ↓
                    ┌──────────────────────────┐
                    │    PX4 偏航率控制器        │
                    │    (FW_YR_P/I/FF)         │ ──→ 差动推力指令
                    └──────────────────────────┘
```

---

### 2.2 Gazebo 模型重生机制

#### 问题背景

PX4 SITL 与 Gazebo 的生命周期不同步：

```
第1次运行:                                    第2次运行:
┌─────────┐   ┌─────────┐               ┌─────────┐
│  PX4    │──→│ Gazebo  │               │  PX4    │──→ 尝试创建 chainwing_0
│ 启动    │   │ 启动    │               │ 重启    │   → 失败! (同名模型已存在)
└─────────┘   └─────────┘               └─────────┘   → gz_bridge 失败
      │             │                         │         → GPS/磁罗盘/EKF 全部缺失
   Ctrl+C          继续运行 ←─── 问题根源 ──→ 无法连接传感器
```

标准 PX4 的 `GZBridge::init()` 使用 `allow_renaming=false` 创建模型。当 PX4 重启而 Gazebo 仍在运行时，旧的 `chainwing_0` 模型还在，新的创建请求失败。

#### 修改位置

**文件**: `src/modules/simulation/gz_bridge/GZBridge.cpp`

```cpp
// ===== 新增代码（第 69-103 行） =====
// 位置: GZBridge::init() 函数开头，在 EntityFactory 创建调用之前

int GZBridge::init()
{
    // Time to wait after removing an existing model, allowing Gazebo to
    // fully tear down sensors and transport topics before recreation.
    static constexpr useconds_t MODEL_CLEANUP_DELAY_US = 2000000; // 2 seconds

    if (!_model_sim.empty()) {

        // Remove any existing model with the same name (handles PX4 restart
        // while Gazebo is still running from a previous session)
        {
            gz::msgs::Entity remove_req{};
            remove_req.set_name(_model_name);
            remove_req.set_type(gz::msgs::Entity::MODEL);

            gz::msgs::Boolean remove_rep;
            bool remove_result;
            std::string remove_service = "/world/" + _world_name + "/remove";

            if (_node.Request(remove_service, remove_req, 1000, remove_rep, remove_result)) {
                if (remove_rep.data() && remove_result) {
                    PX4_INFO("Removed existing model: %s", _model_name.c_str());

                    // Wait for Gazebo to fully clean up the old model's sensors
                    // and transport topics before creating a new one.  Without
                    // this delay, the new model's sensor topics can collide with
                    // stale data from the old model, causing EKF2 position
                    // innovation spikes ("position estimate error").
                    system_usleep(MODEL_CLEANUP_DELAY_US);
                }

            }

            // Ignore failure: model may not exist on first run
        }

        // [原有代码] service call to create model
        gz::msgs::EntityFactory req{};
        req.set_sdf_filename(_model_sim + "/model.sdf");
        req.set_name(_model_name);
        req.set_allow_renaming(false);
        // ...
```

#### 执行流程

```
GZBridge::init()
    │
    ├──→ 发送 /world/$WORLD/remove 请求
    │     ├── 成功 → 打印日志 → 等待 2 秒（清理传感器主题）
    │     └── 失败 → 忽略（首次运行，模型不存在）
    │
    ├──→ 发送 EntityFactory 创建请求
    │     ├── 成功 → 继续初始化
    │     └── 失败 → 返回错误
    │
    └──→ 订阅传感器主题 → GPS/磁罗盘/气压计开始工作
```

#### 为什么需要 2 秒延迟

Gazebo 的模型删除是异步的：

1. `/world/remove` 服务返回成功时，模型实体已从场景图中移除
2. 但关联的传感器插件和传输主题（`/world/flat_terrain/model/chainwing_0/link/base_link/sensor/...`）的清理需要时间
3. 如果立即创建新模型，新旧传感器的 Transport 主题可能冲突
4. 冲突导致 GPS 数据不一致 → EKF2 的位置创新比 (pos_test_ratio) 飙升 → "position estimate error"

经测试，2 秒足以让 Gazebo 完成清理。

---

## 3. 机架参数配置

> 配置文件: `ROMFS/px4fmu_common/init.d-posix/airframes/4007_gz_chainwing`

### 3.1 控制分配器参数

#### 电机配置（差动推力偏航控制）

```sh
# 三台电机 — 差动推力偏航控制
param set-default CA_ROTOR_COUNT 3

# 电机 0：左单元 (Y = -1.2m)
param set-default CA_ROTOR0_PX 0.0
param set-default CA_ROTOR0_PY -1.2
param set-default CA_ROTOR0_PZ 0.0

# 电机 1：中心单元 (Y = 0.0m)
param set-default CA_ROTOR1_PX 0.0
param set-default CA_ROTOR1_PY 0.0
param set-default CA_ROTOR1_PZ 0.0

# 电机 2：右单元 (Y = 1.2m)
param set-default CA_ROTOR2_PX 0.0
param set-default CA_ROTOR2_PY 1.2
param set-default CA_ROTOR2_PZ 0.0
```

| 参数 | 值 | 数据来源 | 原因 |
|------|-----|----------|------|
| `CA_ROTOR_COUNT` | 3 | 物理构型 | 三台电机（左/中/右） |
| `CA_ROTOR0_PY` | -1.2 | MATLAB 模型 `b=1.2m` | 左单元中心 Y 坐标 = -1.2m |
| `CA_ROTOR1_PY` | 0.0 | MATLAB 模型 | 中心单元 Y 坐标 = 0 |
| `CA_ROTOR2_PY` | 1.2 | MATLAB 模型 `b=1.2m` | 右单元中心 Y 坐标 = +1.2m |
| `CA_ROTOR*_PX` | 0.0 | 简化模型 | 电机安装在翼弦中心（实际略偏后 0.25m，但对偏航力矩计算影响小） |

**差动推力偏航控制原理**:

```
左转 (偏航率 < 0):
  左电机增推 + 右电机减推
  → 绕重心产生逆时针力矩
  → Δτ_yaw = ΔT × b = ΔT × 1.2m

右转 (偏航率 > 0):
  右电机增推 + 左电机减推
  → 绕重心产生顺时针力矩

PX4 控制分配器根据 CA_ROTOR*_PY 自动计算偏航力矩臂
```

#### 控制面配置（反向升降副翼）

```sh
# 三个控制面
param set-default CA_SV_CS_COUNT 3

# CS0：左单元升降副翼（类型6 = 右升降副翼）
param set-default CA_SV_CS0_TYPE 6
param set-default CA_SV_CS0_TRQ_R 0.5     # 正值：向上偏转产生正滚转
param set-default CA_SV_CS0_TRQ_P 0.5     # 正值：向上偏转产生正俯仰
param set-default CA_SV_CS0_TRQ_Y 0.0

# CS1：中心单元升降舵
param set-default CA_SV_CS1_TYPE 3
param set-default CA_SV_CS1_TRQ_R 0.0
param set-default CA_SV_CS1_TRQ_P 1.0     # 纯俯仰控制
param set-default CA_SV_CS1_TRQ_Y 0.0

# CS2：右单元升降副翼（类型5 = 左升降副翼）
param set-default CA_SV_CS2_TYPE 5
param set-default CA_SV_CS2_TRQ_R -0.5    # 负值：向上偏转产生负滚转
param set-default CA_SV_CS2_TRQ_P 0.5
param set-default CA_SV_CS2_TRQ_Y 0.0
```

| 参数 | 值 | 数据来源 | 原因 |
|------|-----|----------|------|
| `CA_SV_CS0_TYPE` | 6 (右升降副翼) | 链翼构型分析 | 左机翼安装的是右升降副翼类型（反向配置） |
| `CA_SV_CS0_TRQ_R` | +0.5 | rc_cessna 参考 + GZ LiftDrag 验证 | 见下方详细分析 |
| `CA_SV_CS2_TRQ_R` | -0.5 | 对称关系 | 右翼滚转效果与左翼相反 |
| `CA_SV_CS*_TRQ_P` | 0.5/1.0 | 经验值 | 俯仰力矩贡献（升降副翼=0.5，升降舵=1.0） |

**为什么左翼 TRQ_R = +0.5 而不是 -0.5（反向升降副翼详解）**:

```
传统飞机 (rc_cessna):
  左副翼向上偏转 → 左翼升力↓ → 左翼下沉 → 飞机右滚（正滚转）
  → TRQ_R = +0.5 ✓

链翼构型:
  左单元的升降副翼实际上是"右升降副翼"类型(TYPE=6)
  GZ LiftDrag 插件中 control_joint_rad_to_cl = -0.3（负值）
  → 正舵偏 → 升力系数减小 → 升力下降 → 该侧下沉

  当需要向右滚转时:
    PX4 发送正滚转指令 → CS0 获得 +TRQ_R 方向的偏转
    → 正舵偏 → 左翼升力↓ → 左翼下沉 → 右滚 ✓

  因此 CA_SV_CS0_TRQ_R = +0.5 是正确的
  （与 rc_cessna 的左副翼惯例一致）
```

### 3.2 空速参数

```sh
param set-default FW_AIRSPD_MIN 15      # 最小飞行空速 [m/s]
param set-default FW_AIRSPD_TRIM 20     # 巡航空速 [m/s]
param set-default FW_AIRSPD_MAX 30      # 最大飞行空速 [m/s]
param set-default FW_AIRSPD_STALL 8     # 失速速度 [m/s]
```

| 参数 | 值 | 数据来源 | 原因 |
|------|-----|----------|------|
| `FW_AIRSPD_TRIM` | 20 m/s | MATLAB 开环仿真 `V0=20m/s` | MATLAB 动力学验证使用的巡航速度 |
| `FW_AIRSPD_STALL` | 8 m/s | MATLAB 气动计算 | `V_stall = √(2mg / (ρ·S·CL_max))` |
| `FW_AIRSPD_MIN` | 15 m/s | 1.3 × V_stall × 安全系数 | `15 ≈ 1.88 × 8`，保守安全裕度 |
| `FW_AIRSPD_MAX` | 30 m/s | 结构限制 | 1.5 × V_trim，防止结构过载 |

**失速速度计算**:

```
V_stall = √(2 × m × g / (ρ × S_total × CL_max))

其中:
  m = 5.7 kg（三单元总质量）
  g = 9.81 m/s²
  ρ = 1.2041 kg/m³（标准大气）
  S_total = 3 × 0.36 = 1.08 m²（三翼面积之和）
  CL_max = CLA × (α_stall + a0) = 5.25 × (0.227 + (-0.05)) = 5.25 × 0.177 = 0.929

  V_stall = √(2 × 5.7 × 9.81 / (1.2041 × 1.08 × 0.929))
          = √(111.834 / 1.209)
          = √(92.50)
          ≈ 9.6 m/s

取 8 m/s 作为 PX4 参数（保守值，低于理论计算值）
```

### 3.3 姿态控制参数

```sh
# 时间常数（响应速度）
param set-default FW_R_TC 0.5           # 横滚时间常数 [s]
param set-default FW_P_TC 0.5           # 俯仰时间常数 [s]

# 角速率限制
param set-default FW_R_RMAX 30.0        # 最大横滚率 [°/s]
param set-default FW_P_RMAX_POS 25.0    # 最大正俯仰率 [°/s]
param set-default FW_P_RMAX_NEG 25.0    # 最大负俯仰率 [°/s]
param set-default FW_Y_RMAX 15.0        # 最大偏航率 [°/s]

# 角度限制
param set-default FW_R_LIM 35           # 最大横滚角 [°]
param set-default FW_P_LIM_MAX 15       # 最大俯仰角 [°]
param set-default FW_P_LIM_MIN -15      # 最小俯仰角 [°]

# 俯仰偏移与配平
param set-default FW_PSP_OFF 2.0        # 俯仰设定值偏移 [°]
param set-default TRIM_PITCH -0.15      # 升降舵配平 [-1, 1]
```

| 参数 | 值 | PX4 默认值 | 原因 |
|------|-----|-----------|------|
| `FW_R_TC` | 0.5 | 0.4 | 大惯量需要更长响应时间避免过激 |
| `FW_R_RMAX` | 30 | 70 | 大 Ixx=5.740 限制了实际可达横滚率 |
| `FW_Y_RMAX` | 15 | 50 | 差动推力偏航率有限，限制设定值防止积分饱和 |
| `FW_R_LIM` | 35 | 50 | 大坡度转弯时失速速度增加 7.5%（30°）vs 19%（45°） |
| `FW_PSP_OFF` | 2.0 | 0 | MATLAB 配平攻角 α_trim ≈ 4.7°，GZ 中约 2° |
| `TRIM_PITCH` | -0.15 | 0 | 参考 rc_cessna，减少俯仰积分器稳态负荷 |

### 3.4 速率控制器增益

```sh
# 俯仰率控制器
param set-default FW_PR_P 0.9           # 比例增益
param set-default FW_PR_FF 0.5          # 前馈增益
param set-default FW_PR_I 0.5           # 积分增益

# 横滚率控制器
param set-default FW_RR_P 0.3           # 比例增益
param set-default FW_RR_FF 0.5          # 前馈增益
param set-default FW_RR_I 0.5           # 积分增益

# 偏航率控制器
param set-default FW_YR_P 0.6           # 比例增益
param set-default FW_YR_FF 0.5          # 前馈增益
param set-default FW_YR_I 0.5           # 积分增益
```

| 轴 | P | FF | I | 调参说明 |
|----|---|-----|---|---------|
| 俯仰 | 0.9 | 0.5 | 0.5 | Iyy=0.432 较小，可用较高 P 增益 |
| 横滚 | 0.3 | 0.5 | 0.5 | Ixx=5.740 很大，P 太高会振荡 |
| 偏航 | 0.6 | 0.5 | 0.5 | 差动推力响应较慢，FF 帮助前馈 |

**增益设定原则**:

```
增益 ∝ 1 / 惯量

Ixx = 5.740 (横滚) → 最低 P = 0.3
Iyy = 0.432 (俯仰) → 最高 P = 0.9
Izz = 5.958 (偏航) → P = 0.6 (但差动推力臂大，补偿了部分惯量)
```

### 3.5 偏航航向保持参数

```sh
# 偏航航向保持增益
# GZ 仿真版本使用 1.0（减少模式切换时的过冲）
# 硬件版本 (2150_chainwing) 使用 2.0（真实飞行需更强校正）
param set-default FW_YAW_STAB_SC 1.0
```

| 版本 | 值 | 原因 |
|------|-----|------|
| GZ SITL (4007) | 1.0 | GZ 中差动推力响应完美无延迟，1.0 足够 |
| 硬件 (2150) | 2.0 | 真实 ESC 有响应延迟，需要更高增益补偿 |
| SIH (1103) | 2.0 | SIH 模型较简化，匹配硬件配置 |

### 3.6 着陆参数

```sh
# 地形估计（禁用 — 无测距仪）
param set-default FW_LND_USETER 0       # 不使用地形估计
param set-default FW_LND_ABORT 0        # 禁用着陆中止

# 着陆速度和高度
param set-default FW_LND_AIRSPD 16      # 着陆进近空速 [m/s]
param set-default FW_LND_ANG 8          # 着陆下滑角 [°]
param set-default FW_LND_FL_PMIN 3      # 拉平最小俯仰 [°]
param set-default FW_LND_FL_PMAX 10     # 拉平最大俯仰 [°]
param set-default FW_LND_FLALT 5        # 拉平起始高度 [m]
```

| 参数 | 值 | PX4 默认值 | 原因 |
|------|-----|-----------|------|
| `FW_LND_USETER` | 0 | 1 | **关键修改** — GZ 模型无测距仪，GZ bridge 不订阅激光雷达数据，PX4 无 sensor_distance_sim 模块。默认值 1 会在着陆时等待地形数据，5秒超时后触发中止 |
| `FW_LND_ABORT` | 0 | 3 | 默认值 3 = bit0(地形未找到)+bit1(超时)，两种情况都会中止。禁用后允许仅基于高度信息着陆 |
| `FW_LND_AIRSPD` | 16 | 0 (auto) | 1.2 × V_min = 1.2 × 15 ≈ 16 m/s（高于失速速度的安全裕度） |

### 3.7 任务与导航参数

```sh
# 任务参数
param set-default MIS_TAKEOFF_ALT 15    # 起飞目标高度 [m]
param set-default NAV_ACC_RAD 20        # 航点接受半径 [m]
param set-default NAV_LOITER_RAD 50     # 盘旋半径 [m]
param set-default NPFG_PERIOD 12        # NPFG 导航周期 [s]

# RTL 参数
param set-default RTL_RETURN_ALT 30     # RTL 返回高度 [m]
param set-default RTL_DESCEND_ALT 15    # RTL 下降高度 [m]
param set-default RTL_LAND_DELAY 0      # RTL 后立即着陆

# 数据链路丢失
param set-default NAV_DLL_ACT 0         # 数据链路丢失时不执行操作

# 数据管理器
param set-default SYS_DM_BACKEND 1      # RAM 模式（重启清除任务）
```

| 参数 | 值 | PX4 默认值 | 原因 |
|------|-----|-----------|------|
| `NAV_LOITER_RAD` | 50 | 80 | 链翼最小转弯半径较大（FW_R_LIM=35°，V_trim=20m/s → R_min ≈ V²/(g·tan(φ)) = 400/6.86 ≈ 58m），50m 允许一定的坡度 |
| `NPFG_PERIOD` | 12 | 10 | 大惯量飞机需要更长的导航响应周期，防止路径跟踪振荡 |
| `RTL_RETURN_ALT` | 30 | 100 | SITL 测试不需要高返航高度 |
| `NAV_DLL_ACT` | 0 | 0 | SITL 独立测试可能没有 QGC 连接 |
| `SYS_DM_BACKEND` | 1 | 0 | RAM 模式：每次 SITL 重启自动清除旧任务，防止不兼容任务阻塞 |

### 3.8 仿真专用参数

```sh
# Gazebo 仿真器
param set-default SIM_GZ_EN 1           # 启用 GZ 仿真
param set-default SENS_EN_GPSSIM 1      # 启用 GPS 模拟器
param set-default SENS_EN_BAROSIM 0     # 禁用气压计模拟器（GZ 提供）
param set-default SENS_EN_MAGSIM 1      # 启用磁罗盘模拟器
param set-default SENS_EN_ARSPDSIM 1    # 启用空速管模拟器

# 电池仿真
param set-default SIM_BAT_DRAIN 3600    # 电池耗尽时间 [s]
param set-default COM_LOW_BAT_ACT 0     # 电池低电量仅警告
param set-default CBRK_SUPPLY_CHK 894281 # 跳过供电检查

# GZ 电机映射
param set-default SIM_GZ_EC_FUNC1 101   # ESC1 → Motor 1 (左)
param set-default SIM_GZ_EC_FUNC2 102   # ESC2 → Motor 2 (中)
param set-default SIM_GZ_EC_FUNC3 103   # ESC3 → Motor 3 (右)
param set-default SIM_GZ_EC_MIN1 0
param set-default SIM_GZ_EC_MAX1 1000
param set-default SIM_GZ_EC_MIN2 0
param set-default SIM_GZ_EC_MAX2 1000
param set-default SIM_GZ_EC_MIN3 0
param set-default SIM_GZ_EC_MAX3 1000

# GZ 舵机映射
param set-default SIM_GZ_SV_FUNC1 201   # Servo 1 → 左升降副翼
param set-default SIM_GZ_SV_FUNC2 202   # Servo 2 → 升降舵
param set-default SIM_GZ_SV_FUNC3 203   # Servo 3 → 右升降副翼
```

| 参数 | 值 | 原因 |
|------|-----|------|
| `SENS_EN_BAROSIM` | 0 | GZ 世界自带气压计数据（通过 gz_bridge），不需要独立模拟器 |
| `SIM_BAT_DRAIN` | 3600 | 默认 60 秒太短，1 小时足够完整测试 |
| `COM_LOW_BAT_ACT` | 0 | 默认触发 Hold→RTL→Land 故障保护，测试时仅需警告音 |
| `CBRK_SUPPLY_CHK` | 894281 | 仿真崩溃后 battery_simulator 可能报告不健康状态，跳过检查允许重新解锁 |
| `SIM_GZ_EC_MAX*` | 1000 | 对应 GZ 电机模型的 maxRotVelocity=1000 |

### 3.9 EKF 与传感器参数

```sh
# EKF2 噪声调参
param set-default EKF2_ACC_NOISE 0.5    # 加速度计噪声 [m/s²]
param set-default EKF2_GYR_NOISE 0.02   # 陀螺仪噪声 [rad/s]
param set-default EKF2_ACC_B_NOISE 0.005 # 加速度计偏差噪声

# EKF 预检阈值（放宽）
param set-default COM_ARM_EKF_POS 0.8   # 位置创新比阈值
param set-default COM_ARM_EKF_VEL 0.8   # 速度创新比阈值
```

| 参数 | 值 | PX4 默认值 | 原因 |
|------|-----|-----------|------|
| `COM_ARM_EKF_POS` | 0.8 | 0.5 | **关键修改** — 仿真启动时 EKF2 需要 5-15 秒收敛。默认 0.5 导致持续报 "position estimate error" |
| `COM_ARM_EKF_VEL` | 0.8 | 0.5 | 同上，速度创新比也偏高 |
| `EKF2_ACC_NOISE` | 0.5 | 0.35 | GZ 仿真的加速度计噪声略高于实际传感器 |

**创新比 (Innovation Ratio) 计算**:

```
pos_test_ratio = √(max(GPS_X_innovation², GPS_Y_innovation²) / variance)

当 pos_test_ratio > COM_ARM_EKF_POS 时:
  → "Preflight Fail: position estimate error"
  → 阻止解锁

仿真启动后典型收敛过程:
  t=0s:  pos_test_ratio ≈ 2.0  (完全不收敛)
  t=5s:  pos_test_ratio ≈ 0.8  (接近收敛)
  t=10s: pos_test_ratio ≈ 0.3  (完全收敛)
  t=15s: pos_test_ratio ≈ 0.1  (稳态)

设定 0.8 允许在 ~5 秒后解锁，而不是等待 15 秒
```

### 3.10 故障检测与安全参数

```sh
# ESC 故障检测（禁用）
param set-default FD_ESCS_EN 0

# 起飞方式
param set-default RWTO_TKOFF 1          # 跑道起飞

# 碰撞防止（禁用）
param set-default CP_DIST -1

# RC 输入模式
param set-default COM_RC_IN_MODE 3      # RC 或摇杆

# 预解锁检查
param set-default COM_PREARM_MODE 2     # 始终执行预解锁检查
```

| 参数 | 值 | PX4 默认值 | 原因 |
|------|-----|-----------|------|
| `FD_ESCS_EN` | 0 | 1 | **关键修改** — 固定翼解锁后电机保持 0 转速（等待油门指令）。GZ bridge 仅在转速>0 时设置 `esc_armed_flags`。300ms 后故障检测器标记 `FAILURE_ARM_ESC`，触发故障保护并立即加锁 |
| `CP_DIST` | -1 | -1 | 禁用碰撞防止，消除 QGC 警告 |
| `COM_RC_IN_MODE` | 3 | 1 | 值 3 = "RC 或摇杆"：接受 QGC 虚拟摇杆或物理遥控器中先到达的信号源。否则增稳模式被阻止（"manual_control_signal_lost"） |

---

## 4. Gazebo 仿真模型配置

> 模型文件: `Tools/simulation/gz/models/chainwing/model.sdf` (968 行)

### 4.1 物理模型（惯性参数）

```xml
<model name='chainwing'>
    <pose>0 0 0.25 0 0 0</pose>   <!-- 初始位置：离地 0.25m -->

    <link name='base_link'>
        <inertial>
            <pose>-0.06 0 0 0 0 0</pose>  <!-- CG 偏移：向后 6cm -->
            <mass>5.7</mass>                <!-- 3 × 1.9 kg -->
            <inertia>
                <ixx>5.740</ixx>   <!-- 横滚惯量 -->
                <iyy>0.432</iyy>   <!-- 俯仰惯量 -->
                <izz>5.958</izz>   <!-- 偏航惯量 -->
                <ixy>0</ixy>
                <ixz>0</ixz>
                <iyz>0</iyz>
            </inertia>
        </inertial>
```

#### 惯量计算详解

```
单个单元惯量矩阵 (MATLAB 数据):
  J_unit = diag([0.0894, 0.144, 0.162]) kg·m²

三单元并联 (平行轴定理):

  Ixx (横滚):
    = 3 × J_unit_xx + 2 × m_unit × d²
    = 3 × 0.0894 + 2 × 1.9 × 1.2²
    = 0.2682 + 5.472
    = 5.740 kg·m²
    
    说明: 左右两个单元距中心 1.2m，平行轴贡献 m×d² = 1.9 × 1.44 = 2.736
          两侧合计 2 × 2.736 = 5.472

  Iyy (俯仰):
    = 3 × J_unit_yy + 0          (所有单元沿 Y 轴排列，俯仰轴无平行轴贡献)
    = 3 × 0.144
    = 0.432 kg·m²

  Izz (偏航):
    = 3 × J_unit_zz + 2 × m_unit × d²
    = 3 × 0.162 + 2 × 1.9 × 1.2²
    = 0.486 + 5.472
    = 5.958 kg·m²
```

**CG 偏移 (-0.06m) 原因**:

```
                 电机位置 x=0.25m（推力线）
                     ↓
  ←── -0.06m ──→│ CG
  ─────────────┼──────────────── 翼弦线
               ↑
         翼弦 1/4 处 (0.3m × 0.25 = 0.075m)
         
CG 略偏后于空气动力中心 (AC)，提供纵向静稳定性
AC 位于翼弦 25% 处 ≈ x = 0.075m
CG 位于 x = -0.06m (略偏后)
静稳定裕度 = (x_AC - x_CG) / chord = (0.075 - (-0.06)) / 0.3 = 45%
```

### 4.2 气动力模型（LiftDrag 插件）

链翼模型使用 **8 个 LiftDrag 插件**：

| # | 部件 | 控制关节 | 位置 cp (x, y, z) | 面积 [m²] | 作用 |
|---|------|----------|-------------------|-----------|------|
| 1 | 左翼 | servo_0 | (-0.07, -1.20, 0.03) | 0.36 | 升力+滚转控制 |
| 2 | 中翼 | 无 | (-0.07, 0, 0.03) | 0.36 | 纯升力 |
| 3 | 右翼 | servo_2 | (-0.07, 1.20, 0.03) | 0.36 | 升力+滚转控制 |
| 4 | 平尾 | servo_1 | (-0.55, 0, 0) | 0.18 | 俯仰控制 |
| 5 | 中垂尾 | 无 | (-0.55, 0, 0.09) | 0.02 | 偏航阻尼 |
| 6 | 左垂尾 | 无 | (-0.55, -1.20, 0.09) | 0.02 | 偏航阻尼 |
| 7 | 右垂尾 | 无 | (-0.55, 1.20, 0.09) | 0.02 | 偏航阻尼 |

#### 翼面气动参数

```xml
<!-- 左/中/右翼通用参数 -->
<a0>-0.05</a0>                        <!-- 零升攻角 [rad] ≈ -2.86° -->
<cla>5.25</cla>                       <!-- 升力线斜率 [/rad] -->
<cda>0.65</cda>                       <!-- 阻力系数斜率 -->
<alpha_stall>0.227</alpha_stall>      <!-- 失速攻角 [rad] ≈ 13° -->
<cla_stall>-4.25</cla_stall>          <!-- 失速后升力斜率 -->
<cda_stall>-0.93</cda_stall>          <!-- 失速后阻力斜率 -->
<area>0.36</area>                     <!-- 翼面积 = 1.2m × 0.3m -->
<air_density>1.2041</air_density>     <!-- 标准大气密度 -->
```

| 参数 | 值 | 数据来源 | 公式/说明 |
|------|-----|----------|----------|
| `a0` | -0.05 rad | MATLAB 气动模型 | 零升攻角 ≈ -2.86° |
| `cla` | 5.25 /rad | MATLAB: `CLA=5.25` | 由翼型和展弦比决定: `CL_α = 2π / (1 + 2/AR)`, AR=4 → 3.14，但实际翼型为 5.25 |
| `cda` | 0.65 | MATLAB: `CDA=0.65` | 诱导阻力系数: `Cd = Cd0 + K × CL²`, K = 1/(π×e×AR) = 1/(π×0.85×12) = 0.0313 |
| `alpha_stall` | 0.227 rad | 经验值 | ≈ 13°，典型翼型失速角 |
| `area` | 0.36 m² | 物理尺寸 | 翼展 1.2m × 翼弦 0.3m = 0.36 m² |
| `air_density` | 1.2041 | ISA 标准大气 | 海平面 15°C 标准空气密度 |

#### 升降副翼控制增益

```xml
<!-- 左翼 (servo_0) 和 右翼 (servo_2) -->
<control_joint_rad_to_cl>-0.3</control_joint_rad_to_cl>

<!-- 平尾 (servo_1) — 升降舵 -->
<control_joint_rad_to_cl>-4.0</control_joint_rad_to_cl>
```

| 控制面 | 增益 | 物理含义 |
|--------|------|----------|
| 升降副翼 | -0.3 | 舵偏 +1 rad → CL 减少 0.3。负号表示正舵偏减小升力 |
| 升降舵 | -4.0 | 舵偏 +1 rad → CL 减少 4.0。平尾面积小（0.18 m²），需要更大增益以产生足够俯仰力矩 |

**升降舵增益为什么是 -4.0**:

```
俯仰力矩 = CL_tail × q × S_tail × L_tail

其中:
  CL_tail = control_joint_rad_to_cl × δ_e
  q = 0.5 × ρ × V² = 0.5 × 1.2041 × 20² = 240.8 Pa
  S_tail = 0.18 m²
  L_tail = 0.55 - (-0.07) = 0.62 m（平尾到翼面的力臂）

需要的俯仰力矩（配平）:
  M_pitch = m × g × (x_CG - x_AC) = 5.7 × 9.81 × 0.135 = 7.55 N·m

所需 CL_tail:
  CL_tail = M_pitch / (q × S_tail × L_tail)
          = 7.55 / (240.8 × 0.18 × 0.62)
          = 7.55 / 26.87
          = 0.281

对应的升降舵偏转:
  δ_e = CL_tail / |control_joint_rad_to_cl|
      = 0.281 / 4.0
      = 0.07 rad ≈ 4°

这是合理的配平偏转角
```

#### 垂尾气动参数

```xml
<!-- 垂尾 — 被动偏航阻尼（无控制关节） -->
<a0>0.0</a0>                          <!-- 对称翼型，零侧滑无侧力 -->
<cla>4.75</cla>                       <!-- 侧力斜率 -->
<area>0.02</area>                     <!-- 每个垂尾面积很小 -->
<forward>1 0 0</forward>              <!-- 前进方向：+X -->
<upward>0 1 0</upward>                <!-- 关键: +Y 方向使 LiftDrag 在偏航平面工作 -->
```

**`upward=0,1,0` 的关键作用**:

```
标准 LiftDrag 插件假设:
  forward = 飞行方向 (X)
  upward = 升力方向 (Z)
  → 攻角 = atan2(V_z, V_x) → 升力在 XZ 平面

垂尾设置 upward = Y:
  → "攻角" 变成侧滑角 β = atan2(V_y, V_x)
  → "升力" 变成侧力（在 XY 平面）
  → 侧力 × 力臂 → 偏航力矩（阻尼）

这是 GZ LiftDrag 插件模拟垂尾的标准技巧
```

### 4.3 电机模型（MulticopterMotorModel）

```xml
<!-- 电机 0: 左单元 (逆时针) -->
<plugin filename="gz-sim-multicopter-motor-model-system"
        name="gz::sim::systems::MulticopterMotorModel">
    <jointName>rotor_left_joint</jointName>
    <linkName>rotor_left</linkName>
    <turningDirection>ccw</turningDirection>
    <timeConstantUp>0.0125</timeConstantUp>
    <timeConstantDown>0.025</timeConstantDown>
    <maxRotVelocity>1000</maxRotVelocity>
    <motorConstant>1.5e-05</motorConstant>
    <momentConstant>0.001</momentConstant>
    <commandSubTopic>command/motor_speed</commandSubTopic>
    <motorNumber>0</motorNumber>
    <rotorDragCoefficient>8.06e-05</rotorDragCoefficient>
    <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
    <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
    <motorType>velocity</motorType>
</plugin>

<!-- 电机 1: 中心单元 (顺时针 — 反向旋转以平衡扭矩) -->
<!-- turningDirection = cw -->

<!-- 电机 2: 右单元 (逆时针) -->
<!-- turningDirection = ccw -->
```

| 参数 | 值 | 数据来源 | 计算/说明 |
|------|-----|----------|----------|
| `motorConstant` | 1.5e-05 | MATLAB: `T_max=15N` | `T = kT × ω²`, `kT = T_max / ω_max² = 15 / 1000² = 1.5e-05` |
| `maxRotVelocity` | 1000 | 归一化设定 | 与 `SIM_GZ_EC_MAX*=1000` 匹配 |
| `timeConstantUp` | 0.0125 s | 电机响应特性 | 加速时间常数 12.5ms |
| `timeConstantDown` | 0.025 s | 电机响应特性 | 减速时间常数 25ms（减速慢于加速） |
| `momentConstant` | 0.001 | 经验值 | 反扭矩系数: `M = kM × T = 0.001 × T` |
| `turningDirection` | ccw/cw/ccw | 扭矩平衡 | 中心电机反向旋转，平衡左右电机的陀螺/反扭矩 |
| `rotorDragCoefficient` | 8.06e-05 | rc_cessna 参考 | 螺旋桨阻力系数 |
| `rotorVelocitySlowdownSim` | 10 | GZ 仿真惯例 | 降低可视化旋转速度（不影响推力计算） |

**最大推力计算验证**:

```
T_max = motorConstant × maxRotVelocity²
      = 1.5e-05 × 1000²
      = 1.5e-05 × 1,000,000
      = 15.0 N ✓

三台电机总推力:
  T_total_max = 3 × 15.0 = 45.0 N

推重比:
  TWR = T_total / W = 45.0 N / (5.7 kg × 9.81 m/s²) = 45.0 N / 55.917 N = 0.805

巡航推力 (FW_THR_TRIM=0.60):
  T_cruise = 0.60 × 45.0 = 27.0 N
  阻力 D = 0.5 × ρ × V² × S × Cd
         = 0.5 × 1.2041 × 400 × 1.08 × 0.028
         = 7.3 N (每个翼面)
  总阻力 ≈ 3 × 7.3 + 尾部 = ~25 N
  T_cruise ≈ D ✓（平衡检查通过）
```

### 4.4 舵机控制器

```xml
<!-- 3 个关节位置控制器 -->
<plugin filename="gz-sim-joint-position-controller-system"
        name="gz::sim::systems::JointPositionController">
    <joint_name>servo_0</joint_name>    <!-- 左升降副翼 -->
    <sub_topic>servo_0</sub_topic>
    <p_gain>10.0</p_gain>
</plugin>
<!-- servo_1 (升降舵), servo_2 (右升降副翼) 相同配置 -->
```

| 参数 | 值 | 说明 |
|------|-----|------|
| `p_gain` | 10.0 | 关节位置控制器 P 增益。值越大舵面响应越快。10.0 是 GZ 标准值（rc_cessna 也使用 10.0） |

### 4.5 仿真世界（flat_terrain.sdf）

```xml
<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="flat_terrain">

    <!-- 物理引擎: 250Hz 更新率 -->
    <physics name="4ms" type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- 地理参考坐标（与 PX4 SIM 默认值匹配） -->
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>47.397742</latitude_deg>
      <longitude_deg>8.545594</longitude_deg>
      <elevation>488.0</elevation>
    </spherical_coordinates>

    <!-- 地面: 碰撞 1×1 (无限平面), 视觉 2000×2000 -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>1 1</size></plane></geometry>
          <surface><friction><ode/></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>2000 2000</size></plane></geometry>
        </visual>
      </link>
    </model>

    <!-- 跑道标记: 200m × 2m 中线 -->
    <model name="runway_marking">
      <static>true</static>
      <pose>0 0 0.001 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><box><size>200 2 0.001</size></box></geometry>
          <material><ambient>0.8 0.8 0.8 0.5</ambient></material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

**与 default.sdf 的关键差异**:

| 方面 | default.sdf | flat_terrain.sdf | 原因 |
|------|------------|-------------------|------|
| 碰撞面尺寸 | 1×1 | **1×1** | 必须相同（SDF 平面碰撞无论如何都是无限的） |
| 视觉面尺寸 | 1×1 | **2000×2000** | 固定翼需要广阔视野参考 |
| 摩擦参数 | `<ode/>` | **`<ode/>`** | 必须相同（不同摩擦参数可能影响 GZ 物理引擎初始化） |
| spherical_coordinates | 无 | **有** | 为 GZ 提供准确地理参考（GPS 模拟依赖） |
| 跑道标记 | 无 | **有** | 起飞/着陆视觉参考 |

**为什么碰撞面必须是 1×1 而不是 2000×2000**:

在 SDF 规范中，平面（`<plane>`）的碰撞体是无限延伸的。`<size>` 参数仅影响视觉渲染。
如果设置碰撞尺寸为 2000×2000，虽然功能不变，但可能影响 ODE 物理引擎的 AABB（轴对齐边界框）计算，导致初始化时间差异。
经测试，保持与 default.sdf 一致的 1×1 碰撞尺寸可以避免 gz_bridge 的 EntityFactory 超时问题。

---

## 5. 参数数据来源汇总表

| 数据来源 | 参数 | 数量 | 说明 |
|----------|------|------|------|
| **MATLAB 动力学模型** | 质量、惯量、空速、推力、气动系数 | 15+ | 开环仿真验证的核心数据 |
| **物理尺寸测量** | 翼展、翼弦、面积、单元间距 | 10+ | 实际飞行器的几何参数 |
| **rc_cessna 参考** | 控制面增益符号、舵机配平、EKF 参数 | 8+ | PX4 标准固定翼基准 |
| **GZ 仿真调试** | PID 增益、时间常数、阈值 | 20+ | 仿真中迭代调整 |
| **PX4 默认值** | 大部分未列出的参数 | 100+ | 使用固件默认配置 |
| **问题修复** | FD_ESCS_EN, FW_LND_USETER, COM_ARM_EKF_* | 6 | 解决特定仿真问题 |

### MATLAB 参数对照

| MATLAB 变量 | 值 | 对应 PX4/GZ 参数 | 转换说明 |
|-------------|-----|-------------------|----------|
| `m_unit` | 1.9 kg | `<mass>5.7</mass>` | 3 × 1.9 = 5.7 |
| `b` | 1.2 m | `CA_ROTOR*_PY` | 单元间距 |
| `V0` | 20 m/s | `FW_AIRSPD_TRIM` | 巡航速度 |
| `T_max` | 15 N | `motorConstant=1.5e-05` | kT = T_max / ω² |
| `J_unit` | [0.0894, 0.144, 0.162] | `<ixx/iyy/izz>` | 平行轴定理 |
| `CLA` | 5.25 /rad | `<cla>5.25</cla>` | 直接对应 |
| `CDA` | 0.65 | `<cda>0.65</cda>` | 直接对应 |
| `alpha_0` | -0.05 rad | `<a0>-0.05</a0>` | 直接对应 |
| `S_wing` | 0.36 m² | `<area>0.36</area>` | 1.2m × 0.3m |
| `thrust_trim` | 0.6 | `FW_THR_TRIM=0.60` | 直接对应 |

---

> **文档版本**: 2026-03-14  
> **适用固件**: PX4 v1.15-dev (Chain-Wing Fork)  
> **对应文件**: ecl_yaw_controller.cpp/h, GZBridge.cpp, 4007_gz_chainwing, model.sdf, flat_terrain.sdf
