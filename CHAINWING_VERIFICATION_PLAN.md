# ChainWing 验证测试计划

> **版本**: v3.0  
> **日期**: 2026-03-26  
> **目标**: 系统性验证通信、基础飞控、偏航增强、从机铰链修正  
> **平台**: Gazebo SITL (主要) / Pixhawk 2.4.8 (FMU-V3) SIH (辅助)  
> **工具**: QGC 参数修改 + 自主飞行任务 + Flight Review 回看分析  
> **v2.0 新增**: §12-§15 Gazebo SITL 专项验证指南（通信 + 控制律 + 操作流程 + 数据分析）  
> **v3.0 新增**: §17 自动化参数扫描验证 + chainwing_master 模块 + 双模铰链角

---

## 目录

- [§1 验证总体策略](#1-验证总体策略)
- [§2 测试环境准备](#2-测试环境准备)
- [§3 第一轮：SIH 通信验证](#3-第一轮sih-通信验证)
- [§4 第二轮：基线飞行（无 Yaw P 增强）](#4-第二轮基线飞行无-yaw-p-增强)
- [§5 第三轮：Yaw P 增强 + D 控制评估](#5-第三轮yaw-p-增强--d-控制评估)
- [§6 第四轮：从机铰链修正验证](#6-第四轮从机铰链修正验证)
- [§7 Flight Review 分析指标清单](#7-flight-review-分析指标清单)
- [§8 QGC 参数修改速查表](#8-qgc-参数修改速查表)
- [§9 方法论评估与改进建议](#9-方法论评估与改进建议)
- [§10 风险与应急预案](#10-风险与应急预案)
- [§11 预期结果与判定标准](#11-预期结果与判定标准)
- [§12 GZ SITL 通信验证详细指南](#12-gz-sitl-通信验证详细指南) ← **v2.0 新增**
- [§13 GZ SITL 控制律验证详解](#13-gz-sitl-控制律验证详解) ← **v2.0 新增**
- [§14 GZ SITL 完整操作流程](#14-gz-sitl-完整操作流程) ← **v2.0 新增**
- [§15 GZ SITL 飞行数据深度分析](#15-gz-sitl-飞行数据深度分析) ← **v2.0 新增**
- [§17 自动化参数扫描验证](#17-自动化参数扫描验证) ← **v3.0 新增**
- [§16 版本历史](#16-版本历史)

---

## §1 验证总体策略

### 1.1 四轮递进测试

```
┌──────────────────────────────────────────────────────────────┐
│   第一轮: SIH 通信验证                                        │
│   ├─ chainwing_slave 模块启动确认                              │
│   ├─ uORB 消息流验证                                          │
│   ├─ PWM trim 叠加输出验证                                    │
│   └─ MAVLink DEBUG_FLOAT_ARRAY 收发                           │
│                                                              │
│   第二轮: 基线飞行 (FW_YR_P = 0.05, CW_SLV_EN = 0)           │
│   ├─ 纯 PX4 默认偏航控制                                      │
│   ├─ 验证无增强时的飞行可行性                                   │
│   └─ 建立对比基准                                              │
│                                                              │
│   第三轮: Yaw P 增强 (FW_YR_P = 0.3~0.6 + FW_YR_D 评估)      │
│   ├─ 渐进增加 FW_YR_P                                         │
│   ├─ 观察超调→引入 FW_YR_D                                     │
│   └─ 寻找最优 P/D 组合                                         │
│                                                              │
│   第四轮: 从机修正 (CW_SLV_EN = 1, CW_SLV_PWM_EN = 1)        │
│   ├─ 在最优 Yaw 参数基础上启用                                  │
│   ├─ 验证 trim 叠加效果                                        │
│   └─ 评估是否改善 or 恶化飞行品质                               │
└──────────────────────────────────────────────────────────────┘
```

### 1.2 核心原则

| 原则 | 说明 |
|------|------|
| **单变量控制** | 每轮只改一组参数，确保因果关系清晰 |
| **渐进递增** | 从保守值开始，逐步增加 |
| **可复现** | 每次测试飞同一任务航线 |
| **有记录** | 每次飞行保存 .ulg + Flight Review 链接 |
| **安全第一** | SIH 验证通过后再上实机 |

---

## §2 测试环境准备

### 2.1 Gazebo SITL 环境

```bash
# 编译 SITL
cd /path/to/PX4-Autopilot
make px4_sitl gz_chainwing_3body

# 启动后在 pxh> 终端验证
pxh> chainwing_slave status
# 应显示: running, 50 Hz, hinge estimates

pxh> listener chainwing_hinge_status -n 3
# 应显示: hinge_angle_left/right, trim_left/right, data_valid=true
```

### 2.2 SIH 在环环境（需 Pixhawk 2.4.8）

```bash
# 编译 FMU-V3 固件
make px4_fmu-v3_default

# 刷固件后在 QGC 中：
# 1. 选择机架: 1103_chainwing_sih.hil
# 2. 重启飞控
# 3. 验证参数: SYS_HITL=2, CW_SLV_EN=1
```

### 2.3 标准测试任务航线

**在 QGC 中规划一个矩形航线**（每次测试保持一致）：

```
      起飞点
        │
  ┌─────┼─────┐
  │     │     │
  │  WP2│  WP3│  ← 100m × 200m 矩形
  │     │     │     高度: 50m AGL
  │  WP1│  WP4│     巡航速度: 20 m/s
  │     │     │
  └─────┼─────┘
        │
      降落点

航点:
  WP1: 起飞后直线 200m
  WP2: 左转 90° (测试滚转+偏航协调)
  WP3: 直线 100m
  WP4: 左转 90° (第二次转弯)
  WP5: 返回降落
```

**为什么选矩形**：
- 直线段：观察稳态偏航保持
- 转弯段：观察协调转弯 + 偏航耦合
- 4 个转弯提供足够样本

---

## §3 第一轮：SIH 通信验证

### 3.1 验证目标

| 编号 | 验证项 | 通过标准 |
|------|--------|---------|
| T1.1 | chainwing_slave 模块自动启动 | `chainwing_slave status` 显示 running |
| T1.2 | chainwing_hinge_status 发布 | `listener` 收到消息，data_valid=true |
| T1.3 | PWM trim 叠加生效 | actuator_servos.control[0,2] 包含 trim |
| T1.4 | 铰链角估计正常 | hinge_angle 在合理范围 (< ±0.3 rad) |
| T1.5 | MAVLink 通信（可选） | CW_HINGE id=42 消息在 MAVLink Inspector 可见 |

### 3.2 验证步骤

**步骤 1：模块状态检查**

```
# QGC MAVLink 控制台 (Analyze > MAVLink Console)
pxh> chainwing_slave status

# 预期输出:
# chainwing_slave
#   Running: YES
#   Rate: 50 Hz
#   Enable: 1
#   PWM overlay: ACTIVE        ← CW_SLV_PWM_EN=1
#   Ref initialized: true
#   Hinge angle L: 0.002 rad  R: -0.001 rad
#   Trim L: 0.003  R: -0.002
```

**步骤 2：uORB 消息流验证**

```
pxh> listener chainwing_hinge_status -n 5

# 预期输出:
# TOPIC: chainwing_hinge_status
#   timestamp: 1234567890
#   hinge_angle_left: 0.002
#   hinge_angle_right: -0.001
#   hinge_rate_left: 0.01
#   hinge_rate_right: -0.01
#   trim_left: 0.003
#   trim_right: -0.002
#   data_valid: true

pxh> listener actuator_servos -n 3

# 验证 control[0] 和 control[2] 是否包含 trim 偏移
# 对比 actuator_servos_trim（控制分配器原始输出）
```

**步骤 3：MAVLink 通信验证（可选）**

```
# 在 QGC 中：
# 1. 设置 CW_SLV_COMM_EN = 1
# 2. 打开 Analyze > MAVLink Inspector
# 3. 搜索 DEBUG_FLOAT_ARRAY
# 4. 应看到 id=42, name="CW_HINGE"

# 注意: 需要先配置 MAVLink 实例
# SIH 下如用 USB 连接则无需额外 mavlink start
```

### 3.3 SIH 模式的重要限制

⚠️ **SIH 是单刚体模型，没有铰链物理**

```
SIH 物理引擎:
  ✅ 6DOF 刚体动力学（姿态、位置、速度）
  ✅ 舵面气动力矩
  ✅ 电机推力
  ❌ 铰链运动（没有多体动力学）
  ❌ 铰链弹性/阻尼

结论:
  - SIH 下 hinge_angle ≈ 0（因为无真实铰链变形）
  - trim 值极小（仅由滤波器噪声产生）
  - 这是正常的！SIH 主要验证：
    1. 模块启动和通信链路
    2. PWM 叠加代码不崩溃
    3. 基础飞控在无铰链干扰下工作
```

---

## §4 第二轮：基线飞行（无 Yaw P 增强）

### 4.1 目的

验证 chainwing 气动布局在**最基础控制律**下的飞行可行性。这是对比基准。

### 4.2 QGC 参数设置

```
# 偏航控制率参数 — 使用 PX4 默认值
FW_YR_P    = 0.05    ← PX4 默认值
# 注: SITL 机架文件中设为 0.6（为仿真特别调高），
#     但基线测试刻意使用 PX4 默认 0.05 作为"最保守起点"，
#     以建立无增强的对比基准，再逐步增加到 0.6。
FW_YR_I    = 0.1     ← PX4 默认
FW_YR_FF   = 0.3     ← PX4 默认
FW_YR_D    = 0.0     ← PX4 默认（无 D 项）
FW_YR_IMAX = 0.2     ← PX4 默认

# 偏航稳定增益
FW_YAW_STAB_SC = 0.0  ← 关闭航向保持（纯协调转弯模式）

# 从机控制
CW_SLV_EN  = 0       ← 关闭铰链修正（纯基线测试）
```

### 4.3 物理分析：为什么默认值可能不够

```
当前 chainwing 配置:
  - 3 个电机（左/中/右）间距 1.2m
  - 无垂直尾翼（非传统布局）
  - 偏航控制依赖: 差动推力 + 差动阻力

标准固定翼（有垂尾）:
  - FW_YR_P = 0.05 足够（垂尾提供被动方向稳定性）

ChainWing（无垂尾）:
  - FW_YR_P = 0.05 可能不足
  - 差动推力响应慢于垂尾气动力矩
  - 可能出现:
    ✗ 转弯时侧滑角增大
    ✗ 直线段航向缓慢漂移
    ✗ 风中横向稳定性不足
```

### 4.4 Flight Review 重点关注指标

| 指标 | 位置 | 关注什么 |
|------|------|---------|
| **Yaw Tracking** | Attitude > Yaw | setpoint vs actual 偏差 |
| **Sideslip** | 如有空速探头: β角 | 转弯时是否协调 |
| **Roll-Yaw 耦合** | Attitude > Roll+Yaw | 滚转时偏航是否跟随 |
| **Motor Output** | Actuator Controls > Motor 0/1/2 | 差动推力幅度 |
| **Heading Error** | 自主模式下 heading 偏差 | 直线段漂移量 |

### 4.5 判定标准

| 项目 | 通过 | 勉强 | 失败 |
|------|------|------|------|
| 直线航向保持 | < 5° 偏差 | 5-15° | > 15° |
| 转弯协调性 | β < 5° | 5-10° | > 10° |
| 能完成矩形航线 | 全程稳定 | 有抖动但完成 | 无法完成 |
| 着陆对准 | < 10° 偏差 | 10-20° | > 20° |

---

## §5 第三轮：Yaw P 增强 + D 控制评估

### 5.1 目的

1. 验证增加 FW_YR_P 是否改善偏航控制
2. 评估超调情况
3. 确定是否需要 FW_YR_D（微分项抑制超调）

### 5.2 渐进测试方案

```
                   FW_YR_P 渐进路线图
                   
    默认           轻度增强        中度增强        机架文件值
    ─────────>────────────>────────────>────────────>
    0.05           0.15            0.3            0.6
    
    每档飞一圈矩形航线，在 Flight Review 上比较
```

**测试 3A：FW_YR_P = 0.15**（3× 默认，保守起步）

```
FW_YR_P    = 0.15
FW_YR_D    = 0.0     ← 先不加 D
FW_YR_FF   = 0.3     ← 保持默认
FW_YR_I    = 0.1
FW_YAW_STAB_SC = 0.0 ← 暂不加航向保持
CW_SLV_EN  = 0       ← 继续关闭从机修正
```

**测试 3B：FW_YR_P = 0.3**（6× 默认）

```
FW_YR_P    = 0.3
# 其余同 3A
```

**测试 3C：FW_YR_P = 0.6**（当前 SITL 值，12× 默认）

```
FW_YR_P    = 0.6
# 其余同 3A
```

### 5.3 超调分析与 D 控制引入

**如果在 Flight Review 中观察到偏航超调：**

```
典型超调表现（在 Attitude > Yaw 图中）:
  
  设定值 ─────────────────────────────
                    ╭─╮
  实际值 ──────────╯  ╰──────────────
                  超调 ↑
  
  超调量 = (峰值 - 设定值) / (设定值变化量) × 100%
  
  判定:
    < 5%: 无需 D 控制
    5-15%: 可加小量 D
    15-30%: 建议加 D
    > 30%: 必须加 D 或降低 P
```

**引入 FW_YR_D（测试 3D）：**

```
# 在超调最明显的 FW_YR_P 值基础上

# 第一步: 小量 D
FW_YR_D = 0.003     ← 从极小值开始（PX4 建议范围 0-0.05）

# 第二步: 如果效果不明显，增加到
FW_YR_D = 0.01

# 第三步: 如果还不够
FW_YR_D = 0.02

# ⚠️ 不要超过 0.05！D 过大会放大传感器噪声
```

### 5.4 D 控制物理解释

```
偏航角速率控制器完整公式:

  τ_yaw = FW_YR_P × e_r        ← 比例: 纠正当前偏差
        + FW_YR_I × ∫e_r dt    ← 积分: 消除稳态误差
        + FW_YR_D × d(e_r)/dt  ← 微分: 抑制快速变化（超调阻尼）
        + FW_YR_FF × r_sp      ← 前馈: 快速跟踪设定值

  其中: e_r = r_sp - r_actual (偏航角速率误差)
  
  注: PX4 中 D 项实际计算为:
      D_output = -FW_YR_D × angular_acceleration_z
      使用角加速度而非速率微分，减少噪声放大
```

### 5.5 航向保持增益（FW_YAW_STAB_SC）评估

**在找到最优 FW_YR_P/D 后，可选择性启用：**

```
# 测试 3E: 加入航向保持
FW_YAW_STAB_SC = 1.0  ← SITL 验证值
# 或
FW_YAW_STAB_SC = 2.0  ← 硬件机架文件值

# 航向保持的作用:
#   在稳态直线飞行中，额外施加偏航力矩纠正航向偏差
#   公式: yaw_rate += FW_YAW_STAB_SC × heading_error × (V/V_trim)²
#   
#   注意: 增益随空速²缩放，低速时自动减弱（防止着陆失控）
```

### 5.6 Decision Tree（决策树）

```
飞行 3A (FW_YR_P=0.15)
  ├─ 偏航控制明显改善? 
  │   ├─ YES → 有超调?
  │   │   ├─ YES → 加 FW_YR_D=0.003, 飞 3D
  │   │   └─ NO  → 继续增加到 0.3, 飞 3B
  │   └─ NO  → 继续增加到 0.3, 飞 3B
  │
飞行 3B (FW_YR_P=0.3)
  ├─ 偏航控制满意?
  │   ├─ YES → 有超调?
  │   │   ├─ YES → 加 FW_YR_D=0.005~0.01
  │   │   └─ NO  → 记录最优参数，进入第四轮
  │   └─ NO  → 继续增加到 0.6, 飞 3C
  │
飞行 3C (FW_YR_P=0.6)
  ├─ 偏航控制满意?
  │   ├─ YES → 有超调?
  │   │   ├─ YES → 必须加 D: FW_YR_D=0.01~0.02
  │   │   └─ NO  → 使用此值
  │   └─ NO  → 检查 FW_YR_FF，可能需要同步增加
```

---

## §6 第四轮：从机铰链修正验证

### 6.1 目的

在最优偏航参数基础上，验证铰链修正（CW_SLV_EN=1）的效果。

### 6.2 重要前提

```
⚠️ SIH 模式下的限制:
  SIH = 单刚体模型 → 没有铰链物理
  → hinge_angle ≈ 0（无真实铰链变形）
  → trim ≈ 0（修正量极小）
  → 无法验证"铰链修正是否有效"
  
  只能验证:
  ✅ 修正代码不导致飞行恶化（不引入有害振荡）
  ✅ PWM 叠加代码不崩溃
  ✅ 模块间通信正常
  
  真正的铰链修正效果验证需要:
  ☐ Gazebo SITL（有完整铰链物理）
  ☐ 实机飞行（有真实铰链变形）
```

### 6.3 测试方案

**测试 4A：启用从机修正（SIH 安全验证）**

```
# 在第三轮最优参数基础上
CW_SLV_EN     = 1    ← 启用铰链修正
CW_SLV_PWM_EN = 1    ← 启用 PWM 叠加

# 保守 PD 参数
CW_SLV_KP = 1.5      ← 当前值
CW_SLV_KD = 0.2      ← 当前值
CW_SLV_TRIM_MAX = 0.3  ← 30% 限幅
```

**测试 4B：Gazebo SITL 铰链修正验证（推荐）**

```bash
# Gazebo SITL 有完整铰链物理，是验证修正效果的最佳环境
make px4_sitl gz_chainwing_3body

# 在 pxh> 中测试:
# 1. 飞矩形航线，观察 hinge_angle 变化
# 2. 比较 CW_SLV_EN=0 vs CW_SLV_EN=1 的:
#    - 滚转追踪精度
#    - 舵面活动量
#    - 铰链角振幅
```

### 6.4 判定标准（Gazebo SITL）

| 指标 | 改善 | 中性 | 恶化 |
|------|------|------|------|
| 滚转追踪 RMSE | 降低 > 10% | 变化 < 10% | 升高 > 10% |
| 铰链角峰值 | 降低 | 不变 | 升高 |
| 舵面活动量 | 轻微增加（正常） | 不变 | 持续高频振荡 |
| 飞行轨迹 | 更平滑 | 不变 | 偏离增大 |

### 6.5 铰链修正可能的问题

```
可能的问题          原因                     解决方案
─────────────────────────────────────────────────────────
高频振荡           KP 过大 or 延迟            降低 CW_SLV_KP (1.5→0.5)
舵面饱和           TRIM_MAX 过大              降低 CW_SLV_TRIM_MAX (0.3→0.1)
漂移               互补滤波器漂移              检查 _roll_ref 初始化
反向修正           trim 方向错误               检查 servo channel 映射
无效果             SIH 无铰链物理              正常！需 Gazebo 验证
```

---

## §7 Flight Review 分析指标清单

### 7.1 使用方法

```
1. 飞行完成后，在 QGC 下载 .ulg 日志
   - 路径: QGC > Analyze > Log Download
   
2. 上传到 https://review.px4.io/
   - 拖放 .ulg 文件
   - 等待分析完成
   
3. 保存每次飞行的 URL 链接
   - 格式: https://review.px4.io/plot_app/...
```

### 7.2 每次飞行必查图表

| 图表 | 路径 | 关注点 |
|------|------|--------|
| **Roll** | Attitude Tracking > Roll | setpoint vs actual，超调量，settling time |
| **Pitch** | Attitude Tracking > Pitch | 俯仰稳定性 |
| **Yaw** | Attitude Tracking > Yaw | ⚡ **最重要**: 航向跟踪精度 |
| **Yaw Rate** | Rate Tracking > Yaw Rate | 角速率跟踪，超调/振荡 |
| **Actuator Controls** | Actuator Controls | 舵面活动量，是否饱和 |
| **Motor Outputs** | Actuator Outputs | 差动推力幅度 |
| **Airspeed** | Sensor Data > Airspeed | 空速稳定性 |
| **GPS Track** | Position > GPS Track | 航迹偏离 |

### 7.3 关键数值指标

```
从 Flight Review 的 "Status" 页面读取:

1. Roll 步阶响应时间    → 目标: < 0.5s
2. Pitch 步阶响应时间   → 目标: < 0.5s
3. Yaw 步阶响应时间     → 如有: < 2s
   注: 自主飞行可能无步阶事件（显示 "0"）
   → 这是正常的！自主模式用的是平滑设定值

4. 手工评估:
   - 直线段偏航标准差     → 目标: < 3°
   - 转弯时最大偏航超调   → 目标: < 10°
   - 转弯后恢复时间       → 目标: < 3s
```

### 7.4 对比分析方法

```
在 Flight Review 中打开两个飞行日志并排对比:

https://review.px4.io/plot_app/compare?log1=XXX&log2=YYY

对比组合:
  Round 2 vs Round 3A  → FW_YR_P 0.05 vs 0.15 的效果
  Round 3A vs Round 3B → FW_YR_P 0.15 vs 0.3 的效果  
  Round 3 最优 vs Round 4 → 从机修正 OFF vs ON 的效果
```

---

## §8 QGC 参数修改速查表

### 8.1 操作步骤

```
QGC > Vehicle Setup > Parameters > 搜索参数名

修改后:
  ✅ 参数立即生效（无需重启）
  ✅ 但建议修改后重启以确保保存
  ✅ 每次修改后截图记录（或导出参数文件）
```

### 8.2 各轮次参数表

**第二轮（基线）**

| 参数 | 值 | 说明 |
|------|-----|------|
| FW_YR_P | 0.05 | PX4 默认 |
| FW_YR_I | 0.1 | PX4 默认 |
| FW_YR_D | 0.0 | 无 D 项 |
| FW_YR_FF | 0.3 | PX4 默认 |
| FW_YR_IMAX | 0.2 | PX4 默认 |
| FW_YAW_STAB_SC | 0.0 | 关闭航向保持 |
| CW_SLV_EN | 0 | 关闭从机修正 |

**第三轮（Yaw P 增强）**

| 参数 | 3A | 3B | 3C | 3D (加D) |
|------|-----|-----|-----|----------|
| FW_YR_P | 0.15 | 0.3 | 0.6 | 最优 P |
| FW_YR_D | 0.0 | 0.0 | 0.0 | 0.005~0.02 |
| FW_YR_I | 0.1 | 0.1 | 0.1 | 0.1 |
| FW_YR_FF | 0.3 | 0.3 | 0.3 | 0.3 |
| FW_YAW_STAB_SC | 0.0 | 0.0 | 0.0 | 0.0 |
| CW_SLV_EN | 0 | 0 | 0 | 0 |

**第四轮（从机修正）**

| 参数 | 4A (SIH) | 4B (Gazebo) | 说明 |
|------|----------|-------------|------|
| FW_YR_P | 最优值 | 最优值 | |
| FW_YR_D | 最优值 | 最优值 | |
| CW_SLV_EN | 1 | 1 | |
| CW_SLV_PWM_EN | 1 | 0 | SIH 用硬件 PWM 叠加; Gazebo 用 GZMixingInterfaceServo |
| CW_SLV_KP | 1.5 | 1.5 | |
| CW_SLV_KD | 0.2 | 0.2 | |
| CW_SLV_TRIM_MAX | 0.3 | 0.3 | |

> **注**: CW_SLV_PWM_EN 区别原因：
> - **SIH (PWM_EN=1)**: SIH 运行在 Pixhawk 实体硬件上，使用硬件 PWM 输出。需要 PWM overlay 直接修改 actuator_servos。
> - **Gazebo (PWM_EN=0)**: Gazebo 使用仿真舵面接口 GZMixingInterfaceServo 读取 chainwing_hinge_status 并叠加 trim。不需要 PWM overlay。
> - 两种路径最终效果相同：elevon 输出 = 控制分配值 + trim 修正值。

### 8.3 参数导出/导入

```
# QGC 支持参数文件导入/导出:
# Vehicle Setup > Parameters > Tools (右上角) > Save to file
# 每轮测试前导出一份，便于回滚

建议命名:
  params_round2_baseline.params
  params_round3A_yrp015.params
  params_round3B_yrp030.params
  params_round3C_yrp060.params
  params_round3D_yrp_with_d.params
  params_round4_slave_on.params
```

---

## §9 方法论评估与改进建议

### 9.1 当前方案评估

**你的方案**:
> QGC 参数修改 + 自主飞行 + Flight Review 回看

**评估：✅ 基本合理，但有改进空间**

| 优点 | 限制 |
|------|------|
| ✅ Flight Review 是标准 PX4 分析工具 | ⚠️ 只有事后分析，无实时监控 |
| ✅ QGC 参数修改方便快捷 | ⚠️ 自主飞行无步阶激励，响应分析受限 |
| ✅ 自主飞行可复现 | ⚠️ 航线相同但风况不同 |
| ✅ .ulg 文件可永久保存 | ⚠️ Flight Review 不显示自定义 topic |

### 9.2 改进建议

#### 建议 1：增加 Stabilized 模式手动激励（⭐⭐⭐ 强烈推荐）

```
问题: 自主飞行使用平滑设定值，Flight Review 的
      "Step Response" 分析需要阶跃输入才能计算

解决: 每轮测试增加一段 Stabilized 模式手动飞行

流程:
  1. 自主飞行矩形航线（获取自主模式数据）
  2. 切换 Stabilized 模式
  3. 快速打满偏航杆 → 放开 → 等稳定（≈步阶响应）
  4. 重复 3 次
  5. 切回自主降落

这样 Flight Review 能计算出:
  ✅ Yaw step response time
  ✅ Overshoot percentage
  ✅ Settling time
```

#### 建议 2：使用 PlotJuggler 做深度分析（⭐⭐ 推荐）

```
Flight Review 不能显示自定义 topic (chainwing_hinge_status)。
PlotJuggler 可以:

安装:
  sudo apt install ros-*-plotjuggler-ros  # 如有 ROS
  # 或独立版: https://github.com/facontidavide/PlotJuggler

使用:
  1. pip install pyulog
  2. ulog2csv your_flight.ulg      # 转换为 CSV
  3. 在 PlotJuggler 中加载 CSV
  4. 可绘制:
     - chainwing_hinge_status.hinge_angle_left
     - chainwing_hinge_status.trim_left
     - 与 attitude.roll 叠加对比
```

#### 建议 3：先在 Gazebo SITL 完成所有参数调优（⭐⭐⭐ 强烈推荐）

```
为什么:
  - Gazebo 有完整铰链物理（SIH 没有）
  - 可无限重复（不消耗电池/不冒坠机风险）
  - 参数调优完成后再上 SIH/实机验证
  
建议流程:
  Gazebo SITL: 完成第 2-4 轮所有参数调优
       ↓
  SIH: 验证通信链路 + 代码安全性
       ↓
  实机: 使用 Gazebo 调好的参数
```

#### 建议 4：记录风速和天气条件（⭐ 实机时必须）

```
每次飞行记录:
  - 地面风速（手持风速仪或气象站）
  - 风向
  - 温度
  - 阵风等级

原因: 偏航性能受风影响极大，
      相同参数在有风/无风下表现完全不同
```

#### 建议 5：增加安全边界测试（⭐⭐ 推荐）

```
在找到最优参数后，额外做:

1. 鲁棒性测试: 最优 P × 1.5 → 是否还稳定?
   （留有余量，实际飞行条件可能更恶劣）

2. 故障模式测试 (SIH/Gazebo):
   - CW_SLV_EN 飞行中 0→1 切换 → 是否平稳过渡?
   - CW_SLV_EN 飞行中 1→0 切换 → 是否安全退出?
```

### 9.3 改进后的完整测试流程

```
┌─────────────────────────────────────────────────┐
│  阶段 0: Gazebo SITL 参数调优 (室内)             │
│  ├─ 所有 4 轮测试在 Gazebo 中完成                 │
│  ├─ 确定最优 FW_YR_P / FW_YR_D                  │
│  ├─ 确定 CW_SLV_EN 效果                         │
│  └─ 导出参数文件                                  │
│                                                  │
│  阶段 1: SIH 通信验证 (桌面)                      │
│  ├─ 模块启动、uORB、PWM 叠加验证                   │
│  ├─ 使用 Gazebo 调好的参数                        │
│  └─ 确认 SIH 下飞行正常                          │
│                                                  │
│  阶段 2: 实机地面测试 (跑道)                       │
│  ├─ 电机+舵面响应检查                             │
│  ├─ RC 手动控制验证                               │
│  └─ 传感器校准确认                                │
│                                                  │
│  阶段 3: 实机空中测试 (飞行场)                     │
│  ├─ 手动飞行验证                                  │
│  ├─ 自主飞行矩形航线                              │
│  ├─ Stabilized 模式手动激励                        │
│  └─ Flight Review 分析                            │
└─────────────────────────────────────────────────┘
```

---

## §10 风险与应急预案

### 10.1 SIH 测试风险

| 风险 | 概率 | 影响 | 预案 |
|------|------|------|------|
| 模块未启动 | 低 | 无修正 | 手动 `chainwing_slave start` |
| SIH 物理不收敛 | 低 | 仿真崩溃 | 重启飞控 |
| 参数保存失败 | 低 | 参数丢失 | 修改后重启确认 |

### 10.2 Gazebo SITL 测试风险

| 风险 | 概率 | 影响 | 预案 |
|------|------|------|------|
| FW_YR_P 过大导致振荡 | 中 | 仿真坠机 | 降低 P 值，重新启动 |
| CW_SLV 导致舵面饱和 | 低 | 控制异常 | 降低 TRIM_MAX |
| 锁步仿真时间偏差 | 低 | 日志时间线不准 | 使用仿真时间分析 |

### 10.3 实机测试风险（将来）

| 风险 | 概率 | 影响 | 预案 |
|------|------|------|------|
| 偏航失控 | 中 | 坠机 | 手动 RTL，降低 P |
| 铰链修正反向 | 低 | 加剧振荡 | QGC 中 CW_SLV_EN=0 |
| 电机差动不足 | 中 | 偏航响应弱 | 增加 FW_YR_FF |
| 通信延迟 | 低 | 修正滞后 | 降低 KP |

---

## §11 预期结果与判定标准

### 11.1 各轮预期结果

**第二轮（基线）预期**:
```
预期: 能飞，但偏航控制偏弱
  - 直线段可能有缓慢航向漂移
  - 转弯协调性一般
  - 这建立了"不加增强时的水平"
```

**第三轮（Yaw P 增强）预期**:
```
FW_YR_P = 0.15: 轻微改善，可能不够
FW_YR_P = 0.3:  明显改善，可能有轻微超调
FW_YR_P = 0.6:  强劲改善，但可能有明显超调

加 FW_YR_D 后: 超调减少 30-50%
```

**第四轮（从机修正）预期**:
```
SIH:    几乎无效果（无铰链物理），确认"不害人"
Gazebo: 滚转追踪改善 10-20%（转弯时更明显）
        铰链角峰值降低
        舵面活动量略增
```

### 11.2 最终成功标准

```
✅ 基线飞行可行（证明气动布局可行）
✅ 找到最优 FW_YR_P（偏航响应好、超调可接受）
✅ FW_YR_D 效果验证（是否需要，需要多少）
✅ 从机修正不恶化飞行品质（安全性确认）
✅ 每次飞行有 .ulg + Flight Review 记录
✅ 最终参数文件导出保存
```

---

## 附录 A：快速参考卡

### A.1 PXH 控制台快速命令

```
# 模块状态
chainwing_slave status

# 监听铰链状态（3条消息后自动停止）
listener chainwing_hinge_status -n 3

# 监听舵面输出
listener actuator_servos -n 3

# 参数实时修改
param set FW_YR_P 0.3
param set CW_SLV_EN 1

# 参数保存
param save

# 参数重置为默认
param reset FW_YR_P
```

### A.2 Flight Review 关键 URL

```
上传: https://review.px4.io/upload
分析: https://review.px4.io/plot_app/browse
对比: https://review.px4.io/plot_app/compare?log1=XXX&log2=YYY
```

### A.3 测试结果记录模板

```
日期: ____
轮次: ____
参数: FW_YR_P=____ FW_YR_D=____ CW_SLV_EN=____
风速: ____  风向: ____
Flight Review URL: ____
.ulg 文件: ____

观察:
  偏航跟踪: □优 □良 □差
  超调:     □无 □轻 □重
  振荡:     □无 □轻 □重
  总评:     □通过 □需改进 □失败

备注: ____
```

---

## §12 GZ SITL 通信验证详细指南

> **本节专注 Gazebo SITL**。与 SIH（§3）不同，Gazebo 有完整铰链物理，能真正验证通信+修正效果。

### 12.1 GZ SITL 通信架构总览

```
┌──────────────────────── PX4 进程 ────────────────────────────┐
│                                                              │
│  ┌──────────────┐    vehicle_angular_velocity    ┌─────────┐ │
│  │   EKF2       │ ─────────────────────────────→ │ Chain-  │ │
│  │  (250 Hz)    │    vehicle_attitude             │ wing    │ │
│  │              │ ─────────────────────────────→ │ Slave   │ │
│  └──────────────┘                                │ (50 Hz) │ │
│                                                  │         │ │
│  ┌──────────────┐    actuator_servos             │         │ │
│  │  Control     │ ─────────────────────────────→ │ (PWM    │ │
│  │  Allocator   │       (250 Hz)                 │  mode   │ │
│  │  (250 Hz)    │                                │  only)  │ │
│  └──────────────┘                                └────┬────┘ │
│         │                                             │      │
│         │ actuator_servos                             │      │
│         │  (原始,无trim)            chainwing_hinge_status    │
│         ↓                                             ↓      │
│  ┌──────────────────────────────────────────────────────────┐ │
│  │             GZMixingInterfaceServo (250 Hz)              │ │
│  │                                                          │ │
│  │  output = (actuator_servos[i] - 500) / 500.0             │ │
│  │                                                          │ │
│  │  if hinge_valid:                                         │ │
│  │    servo_0 += trim_left     ← 左Elevon铰链修正            │ │
│  │    servo_2 += trim_right    ← 右Elevon铰链修正            │ │
│  │    clamp(-1.0, 1.0)                                      │ │
│  │                                                          │ │
│  │  Publish to Gazebo: /model/chainwing_3body_0/servo_{0,1,2}│ │
│  └──────────────────────────────────────────────────────────┘ │
└──────────────────────────────────────────────────────────────┘
                              │
                    Gazebo Transport
                              │
                              ↓
         ┌──────────────────────────────────────┐
         │   Gazebo 物理引擎 (1000 Hz)           │
         │                                      │
         │   servo_0 → 左Elevon LiftDrag        │
         │   servo_1 → 中央Elevator LiftDrag     │
         │   servo_2 → 右Elevon LiftDrag        │
         │                                      │
         │   hinge_left:  k=200, c=12, ±15°     │
         │   hinge_right: k=200, c=12, ±15°     │
         │                                      │
         │   → IMU 传感器数据 → gz_bridge → PX4  │
         └──────────────────────────────────────┘
```

### 12.2 关键通信路径验证

**路径 A：铰链估计（输入侧）**

```
验证目标: IMU 数据 → ChainwingSlave → 铰链角估计

步骤:
1. 启动 SITL
   make px4_sitl gz_chainwing_3body

2. 在 pxh> 终端检查模块状态
   pxh> chainwing_slave status

   预期输出:
   Chain-wing slave controller
     Enabled: YES
     PWM trim overlay: DISABLED (sim)     ← GZ用GZMixingInterfaceServo,不需要PWM overlay
     Communication: DISABLED
     Reference initialized: YES
     Hinge angles: left=0.xxx deg, right=-0.xxx deg
     Hinge rates:  left=0.xxx, right=-0.xxx rad/s
     PD gains: Kp=1.50, Kd=0.20, max_trim=0.30

   ⚠️ 关键确认:
   - PWM trim overlay: DISABLED (sim)  ← 这是正确的！GZ不用PWM overlay
   - Reference initialized: YES         ← 互补滤波器已初始化
   - Hinge angles 不为0                  ← 有真实铰链变形

3. 监听铰链状态
   pxh> listener chainwing_hinge_status -n 5

   预期:
   - hinge_angle_left/right: 小值 (±0.01~0.05 rad 静态, 转弯时更大)
   - hinge_rate_left/right: 随运动变化
   - trim_left/right: = KP * angle + KD * rate
   - data_valid: true
```

**路径 B：Trim 叠加（输出侧）— GZ 专用路径**

```
验证目标: chainwing_hinge_status → GZMixingInterfaceServo → Gazebo servo topic

代码位置: src/modules/simulation/gz_bridge/GZMixingInterfaceServo.cpp:62-86

关键代码（已验证）:
  Line 63: bool hinge_valid = _hinge_status_sub.copy(&hinge_status) && hinge_status.data_valid;
  Line 77: if (i == 0) output += (double)hinge_status.trim_left;
  Line 79: if (i == 2) output += (double)hinge_status.trim_right;

验证方法:
  pxh> listener actuator_servos -n 3     # 控制分配器原始输出
  
  # 对比: Gazebo 端的实际舵面输入
  # 终端2:
  # ⚠️ 注意: SITL 以 -i 0 启动, model name = chainwing_3body_0 (有 _0 后缀!)
  # 先用 gz topic -l | grep servo 确认实际话题名
  gz topic -e /model/chainwing_3body_0/servo_0  # 左Elevon (应 = 原始 + trim_left)
  gz topic -e /model/chainwing_3body_0/servo_1  # 中央Elevator (应 = 原始, 无修正)
  gz topic -e /model/chainwing_3body_0/servo_2  # 右Elevon (应 = 原始 + trim_right)
  
  如果 servo_0 ≠ actuator_servos.control[0] (差值 ≈ trim_left)，
  则通信路径正确 ✅
```

**路径 C：MAVLink 通信验证（可选,用于多实例）**

```
验证目标: DEBUG_FLOAT_ARRAY 在 SITL 多实例间传输

步骤:
1. 启用通信
   pxh> param set CW_SLV_COMM_EN 1

2. 启动 MAVLink custom 实例
   pxh> mavlink start -x -u 24550 -o 24551 -r 4000 -m custom
   pxh> mavlink stream -u 24550 -s DEBUG_FLOAT_ARRAY -r 10

   ⚠️ 必须用 -m custom！不要用 -m onboard！
   原因: onboard 模式包含 ODOMETRY@30Hz，
         ODOMETRY.hpp:140 硬编码 estimator_type=8
         → mavlink_receiver.cpp:1429 拒绝 → 60次/秒警告刷屏

3. 监听 debug_array
   pxh> listener debug_array -n 3
   
   预期: id=42, name="CW_HINGE", data[0-6] 有值
```

### 12.3 GZ SITL vs SIH 通信差异速查

| 维度 | Gazebo SITL | SIH |
|------|-------------|-----|
| **铰链物理** | ✅ 完整（k=200, c=12, ±15°）| ❌ 无（单刚体） |
| **铰链角** | 真实值（转弯时可达 ±5°）| ≈ 0（噪声级别）|
| **trim 输出** | 有实际修正量（±0.05~0.15）| ≈ 0 |
| **Trim 叠加路径** | GZMixingInterfaceServo | CW_SLV_PWM_EN=1 |
| **CW_SLV_PWM_EN** | = 0（关闭）| = 1（启用）|
| **验证价值** | ⭐⭐⭐ 完整验证 | ⭐ 仅通信链路 |

---

## §13 GZ SITL 控制律验证详解

### 13.1 偏航控制完整链路（5 层追踪）

```
       ┌─────────── 第 1 层: 任务/手动输入 ──────────────┐
       │                                                 │
       │  自主模式: navigator → heading_setpoint          │
       │  手动模式: RC yaw stick → FW_Y_RMAX × stick     │
       │                                                 │
       └──────────────────┬──────────────────────────────┘
                          ↓
       ┌─────────── 第 2 层: 姿态控制器 ─────────────────┐
       │  文件: FixedwingAttitudeControl.cpp              │
       │                                                 │
       │  协调转弯: ω_yaw = tan(φ)·cos(θ)·g / V         │
       │    → ecl_yaw_controller.cpp:88                  │
       │                                                 │
       │  航向保持:                                       │
       │    heading_error = ψ_sp - ψ_actual              │
       │    V_ratio = V / max(V_trim, 1.0)               │
       │    K_scaled = FW_YAW_STAB_SC × V_ratio²         │
       │    ω_correction = heading_error × K_scaled       │
       │    → ecl_yaw_controller.cpp:109-120             │
       │                                                 │
       │  输出: yaw_rate_setpoint (rad/s)                 │
       └──────────────────┬──────────────────────────────┘
                          ↓
       ┌─────────── 第 3 层: 角速率控制器 ───────────────┐
       │  文件: rate_control.cpp:78                       │
       │                                                 │
       │  τ_yaw = FW_YR_P × e_r                          │
       │        + I_state                                 │
       │        - FW_YR_D × angular_accel_z               │
       │        + FW_YR_FF × r_sp                         │
       │                                                 │
       │  其中: e_r = r_sp - r_actual                     │
       │                                                 │
       │  空速缩放: V_scale = V_trim / max(V, V_min)     │
       │  τ_yaw_scaled = τ_yaw × V_scale²                │
       │    → FixedwingRateControl.cpp:372                │
       │                                                 │
       │  滚转-偏航耦合前馈:                               │
       │  τ_yaw += FW_RLL_TO_YAW_FF × τ_roll             │
       │    → FixedwingRateControl.cpp:412                │
       │                                                 │
       │  输出: vehicle_torque_setpoint.xyz[2]            │
       └──────────────────┬──────────────────────────────┘
                          ↓
       ┌─────────── 第 4 层: 控制分配 ───────────────────┐
       │  文件: ActuatorEffectivenessControlSurfaces.cpp  │
       │                                                 │
       │  效率矩阵:                                       │
       │  ┌──────────┬───────┬───────┬───────┐           │
       │  │ 通道     │ Roll  │ Pitch │ Yaw   │           │
       │  ├──────────┼───────┼───────┼───────┤           │
       │  │ CS0(LEv) │ +0.5  │ +0.5  │  0.0  │           │
       │  │ CS1(Ele) │  0.0  │ +1.0  │  0.0  │           │
       │  │ CS2(REv) │ -0.5  │ +0.5  │  0.0  │           │
       │  └──────────┴───────┴───────┴───────┘           │
       │                                                 │
       │  注: Yaw 列全为 0 → 偏航完全靠差动推力！          │
       │  S0 = 0.5×τ_roll + 0.5×τ_pitch                  │
       │  S1 = τ_pitch                                    │
       │  S2 = -0.5×τ_roll + 0.5×τ_pitch                 │
       └──────────────────┬──────────────────────────────┘
                          ↓
       ┌─────────── 第 5 层: 铰链修正叠加 ───────────────┐
       │  文件: GZMixingInterfaceServo.cpp:76-86          │
       │                                                 │
       │  servo_0_final = servo_0_raw + trim_left         │
       │  servo_1_final = servo_1_raw (无修正)             │
       │  servo_2_final = servo_2_raw + trim_right        │
       │                                                 │
       │  其中: trim = CW_SLV_KP × θ_hinge               │
       │             + CW_SLV_KD × θ̇_hinge               │
       │  → ChainwingSlave.cpp:224-234                    │
       └─────────────────────────────────────────────────┘
```

### 13.2 四轮测试的控制律参数矩阵（GZ SITL 专用）

> ⚠️ **重要**: GZ SITL 机架 `4008_gz_chainwing_3body` 的默认参数与 PX4 默认值不同！
> 以下标注的"机架默认"是指 SITL 机架文件中的设定值。

| 参数 | PX4 默认 | 机架默认 | 第2轮(基线) | 第3轮A | 第3轮B | 第3轮C | 第3轮D |
|------|---------|---------|-----------|-------|-------|-------|-------|
| **FW_YR_P** | 0.05 | **0.6** | 0.05 | 0.15 | 0.3 | 0.6 | 最优P |
| **FW_YR_I** | 0.1 | **0.5** | 0.1 | 0.1 | 0.1 | 0.5 | 0.5 |
| **FW_YR_D** | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 | 0.005~0.02 |
| **FW_YR_FF** | 0.3 | **0.5** | 0.3 | 0.3 | 0.3 | 0.5 | 0.5 |
| **FW_YAW_STAB_SC** | 0.0 | **1.0** | 0.0 | 0.0 | 0.0 | 1.0 | 1.0 |
| **FW_RR_P** | 0.05 | **0.3** | 0.3 | 0.3 | 0.3 | 0.3 | 0.3 |
| **FW_PR_P** | 0.08 | **0.9** | 0.9 | 0.9 | 0.9 | 0.9 | 0.9 |
| **CW_SLV_EN** | 1 | 1 | **0** | 0 | 0 | 0 | 0 |
| **CW_SLV_KP** | 1.5 | 1.5 | - | - | - | - | - |
| **CW_SLV_KD** | 0.2 | 0.2 | - | - | - | - | - |

> **策略**: 第 2 轮用 PX4 默认值建立基线 → 第 3 轮逐步增大 → 第 3C 轮使用机架文件值

### 13.3 每个偏航参数的物理含义与调参建议

**FW_YR_P（偏航角速率比例增益）— 代码: rate_control.cpp:78**

```
作用: τ_P = FW_YR_P × (yaw_rate_sp - yaw_rate_actual)

物理含义:
  - 偏航角速率误差 1 rad/s 时，产生多大的差动推力力矩
  - PX4默认 0.05：假设有垂尾，1 rad/s误差只需5%推力差
  - ChainWing 设 0.6：无垂尾，需要12×差动推力补偿

调参原则:
  太小 → 偏航响应迟缓，转弯侧滑大，直线漂移
  太大 → 偏航振荡，电机反复加减速（听到嗡嗡声）
  
Gazebo 观察:
  - 打开 pxh> listener vehicle_torque_setpoint -r 2
  - 观察 xyz[2] (偏航力矩)，应在 ±0.3 范围内
  - 如果持续 > 0.5 → P 可能过大
```

**FW_YR_I（偏航角速率积分增益）— 代码: rate_control.cpp:107-111**

```
作用: 消除稳态偏航速率偏差（如侧风导致的持续偏航）

物理含义:
  - 不为0时：长时间偏航误差会累积，逐渐增大修正量
  - 机架默认 0.5（比PX4默认0.1高5×）→ 快速消除稳态偏差
  
注意:
  - I 项有 IMAX 限制（FW_YR_IMAX=0.2），防止饱和
  - rate_control.cpp:107 的 i_factor 在大误差时自动减弱 I 增益:
    i_factor = max(0, 1 - (error/400°)²)
    这防止了大误差时积分饱和的"windup"问题
    
调参建议:
  第 2 轮保持 0.1（保守），第 3C 轮改为 0.5（机架值）
```

**FW_YR_D（偏航角速率微分增益）— 代码: rate_control.cpp:78**

```
作用: τ_D = -FW_YR_D × angular_acceleration_z

物理含义:
  - 注意负号！抑制偏航角加速度（不是角速率微分）
  - 当偏航加速过快时（如 P 项过激），D 项产生反向力矩
  - 效果 = 偏航运动的"阻尼器"

代码细节 (rate_control.cpp:78):
  torque = P*error + I_state - D*angular_accel + FF*rate_sp
                               ↑ 负号在这里

为什么 PX4 默认 D=0?
  - 角加速度信号噪声大（陀螺仪二阶导数）
  - 对于有垂尾的飞机，被动气动阻尼足够
  - ChainWing 无垂尾，如果 P 过大导致超调，D 项有意义

调参建议:
  - 如果 FW_YR_P=0.3 时有超调 > 15% → 先试 FW_YR_D=0.005
  - 如果 FW_YR_P=0.6 时有超调 > 25% → 试 FW_YR_D=0.01~0.02
  - ⚠️ 不要超过 0.05！否则噪声放大导致高频抖动
  - Flight Review 判断: Yaw Rate 图中是否有高频毛刺
```

**FW_YR_FF（偏航角速率前馈）— 代码: rate_control.cpp:78**

```
作用: τ_FF = FW_YR_FF × yaw_rate_setpoint

物理含义:
  - 不等误差反馈，直接根据设定值产生力矩
  - 提高跟踪速度，减少延迟
  - PX4默认 0.3, 机架设 0.5 → 更快的偏航响应

调参建议:
  - FF 与 P 配合：P 负责纠偏，FF 负责跟踪
  - 如果 P 增大后偏航响应够快但超调，不应再增 FF
  - 如果 P 不大但需要更快响应 → 增 FF 比增 P 更安全
```

**FW_YAW_STAB_SC（航向保持增益）— 代码: ecl_yaw_controller.cpp:109-120**

```
作用: ω_correction = FW_YAW_STAB_SC × heading_error × (V/V_trim)²

物理含义:
  - 在直线飞行时，额外施加偏航力矩保持航向
  - 与协调转弯公式叠加（不替代）
  - 空速²缩放: 低速时自动减弱（防止着陆时过度修正）
  
数值示例 (V=20m/s, V_trim=20m/s):
  heading_error = 5° = 0.087 rad
  FW_YAW_STAB_SC = 1.0
  V_ratio = 20/20 = 1.0
  ω_correction = 1.0 × 0.087 × 1.0² = 0.087 rad/s
  
  heading_error = 5° = 0.087 rad
  FW_YAW_STAB_SC = 1.0
  V = 15m/s (着陆)
  V_ratio = 15/20 = 0.75
  V_ratio² = 0.75² = 0.5625
  ω_correction = 1.0 × 0.087 × 0.5625 = 0.049 rad/s ← 低速时自动减弱

GZ 验证:
  第 2 轮: FW_YAW_STAB_SC=0 → 观察直线段航向漂移
  第 3C 轮: FW_YAW_STAB_SC=1.0 → 观察漂移是否改善
```

### 13.4 铰链修正控制律详解（第 4 轮专用）

```
┌──── ChainwingSlave::Run() [50 Hz] ────────────────────────┐
│                                                            │
│  1. 更新铰链角估计 (updateHingeEstimate)                    │
│     ├─ 读取 IMU: roll_rate = angular_vel.xyz[0]            │
│     ├─ 低通滤波: α = dt / (dt + 1/(2π·LP_FREQ))           │
│     │   rate_filtered = (1-α)·rate_old + α·rate_new        │
│     ├─ 积分+衰减: angle = e^(-dt/τ) · (angle + rate·dt)   │
│     │   τ = 2.0s (互补滤波时间常数)                         │
│     └─ 姿态校正: angle = 0.98·angle + 0.02·roll_error     │
│                                                            │
│  2. 计算 trim (computeTrim)                                 │
│     trim = KP × angle + KD × rate                          │
│     trim = clamp(trim, -TRIM_MAX, +TRIM_MAX)               │
│                                                            │
│  3. 发布 hinge_status → GZMixingInterfaceServo              │
│                                                            │
│  数值示例 (5° = 0.087 rad 铰链偏转):                        │
│     trim = 1.5 × 0.087 + 0.2 × 0.05                       │
│          = 0.131 + 0.01 = 0.141 (14.1%)                    │
│                                                            │
│     转换为舵面偏转:                                         │
│     左Elevon: servo_0 += 0.141 → ≈ 0.141 × 30° = 4.2°     │
│     右Elevon: servo_2 += trim_right (对称反向)              │
└────────────────────────────────────────────────────────────┘

潜在问题分析:

  问题1: 互补滤波器漂移
    原因: 单IMU无法直接测量铰链角，依赖积分
    症状: 静态飞行中 hinge_angle 缓慢增大
    观察: listener chainwing_hinge_status -n 10
    判断: 直线段 angle > 0.1 rad (5.7°) 且持续增长 → 有漂移
    解决: 降低 cf_alpha (0.02→0.05，更信任姿态)

  问题2: 修正方向反转
    原因: 铰链变形方向假设错误
    症状: 启用修正后滚转偏差更大（CW_SLV_EN=1 比 =0 更差）
    判断: Flight Review 对比 Roll RMSE
    解决: 交换 trim_left 和 trim_right（代码修改）

  问题3: 高频振荡
    原因: KP 过大，采样延迟导致相位裕度不足
    症状: Actuator Controls 图中 servo_0/servo_2 有 >5Hz 振荡
    判断: 频谱分析（PlotJuggler FFT 功能）
    解决: 降低 KP (1.5→0.5) 或增加 LP_FREQ 滤波
```

### 13.5 差动推力偏航控制的物理限制

```
ChainWing 偏航控制 = 差动推力（无垂尾！）

三个电机位置:
  Motor 0: Y = -1.2m (左)
  Motor 1: Y =  0.0m (中,无偏航贡献)
  Motor 2: Y = +1.2m (右)
  
最大偏航力矩 = (T_max - T_min) × 1.2m
  假设 T_max = 15N, T_min = 5N
  → τ_yaw_max = 10 × 1.2 = 12 N·m
  
vs 有垂尾飞机:
  典型垂尾: τ_yaw ≈ 0.5 × ρ × V² × S_vt × l_vt × C_Lα × δ_r
  在 20m/s: τ_yaw ≈ 25~50 N·m

结论: ChainWing 差动推力力矩 < 有垂尾的 1/3
  → 需要更大的 FW_YR_P 补偿
  → FW_YR_P = 0.6 (12× 默认) 是合理的
  → 但差动推力响应时间 > 舵面响应（电机加减速惯性）
  → 这是超调的物理根源
```

---

## §14 GZ SITL 完整操作流程

### 14.1 准备工作

```bash
# ═══════════════════════════════════════════════════════════
# 第 0 步: 确认环境
# ═══════════════════════════════════════════════════════════

# 1. 确认 QGC 已安装并启动
#    QGC 会自动连接 SITL（UDP 14550）

# 2. 确认 Gazebo 环境
which gz  # 应返回 gz 路径

# 3. 进入 PX4 目录
cd /path/to/PX4-Autopilot
```

### 14.2 第 2 轮：基线飞行操作流程

```bash
# ═══════════════════════════════════════════════════════════
# 第 2 轮: 基线飞行 — 验证无 Yaw 增强时的飞行可行性
# ═══════════════════════════════════════════════════════════

# 1. 启动 SITL
make px4_sitl gz_chainwing_3body

# 2. 等待 "Ready for takeoff" 或 QGC 显示连接
#    pxh> 会出现

# 3. 在 pxh> 终端修改参数（覆盖机架默认值）
param set FW_YR_P 0.05        # PX4 默认（机架文件=0.6）
param set FW_YR_I 0.1         # PX4 默认（机架文件=0.5）
param set FW_YR_FF 0.3        # PX4 默认（机架文件=0.5）
param set FW_YAW_STAB_SC 0.0  # 关闭航向保持（机架文件=1.0）
param set CW_SLV_EN 0         # 关闭铰链修正

# 4. 验证参数已生效
param show FW_YR_P             # 应显示 0.0500
param show CW_SLV_EN           # 应显示 0

# 5. 在 QGC 中规划矩形航线
#    Plan View > Add Waypoint
#    WP1: 起飞位置北 200m
#    WP2: WP1 西 100m
#    WP3: WP2 南 200m
#    WP4: WP3 东 100m（回起飞位置上方）
#    高度: 50m, 速度: 20m/s
#    
#    设置: Takeoff → Waypoints → Land

# 6. 上传任务
#    QGC: Plan > Upload (右上角)

# 7. 解锁并起飞
#    QGC: Fly View > Slide to Arm > Confirm Takeoff
#    或 pxh>:
commander takeoff

# 8. 观察飞行
#    QGC 地图: 关注航迹偏离
#    实时参数: 随时可在 pxh> 调整

# 9. 飞行中监控（可选,另开终端窗口）
listener vehicle_attitude -r 2   # 实时姿态
listener vehicle_angular_velocity -r 2  # 实时角速率

# 10. 着陆后下载日志
#     QGC > Analyze > Log Download > 选最新的 .ulg
#     或: 日志保存在 build/px4_sitl_default/rootfs/log/

# 11. 上传到 Flight Review
#     打开 https://review.px4.io/upload
#     拖放 .ulg 文件
#     保存 URL

# 12. 记录结果
#     用 §A.3 模板填写
```

### 14.3 第 3 轮：Yaw P 增强操作流程

```bash
# ═══════════════════════════════════════════════════════════
# 第 3A 轮: FW_YR_P = 0.15
# ═══════════════════════════════════════════════════════════

# 不需要重启 SITL！直接在 pxh> 修改参数：
param set FW_YR_P 0.15

# 飞同一航线 → 着陆 → 保存日志 → 上传 Flight Review

# ═══════════════════════════════════════════════════════════
# 第 3B 轮: FW_YR_P = 0.3
# ═══════════════════════════════════════════════════════════

param set FW_YR_P 0.3

# 飞同一航线 → 着陆 → 保存日志 → 上传 Flight Review

# ═══════════════════════════════════════════════════════════
# 第 3C 轮: FW_YR_P = 0.6 + 机架文件完整参数
# ═══════════════════════════════════════════════════════════

param set FW_YR_P 0.6
param set FW_YR_I 0.5          # 恢复机架文件值
param set FW_YR_FF 0.5         # 恢复机架文件值
param set FW_YAW_STAB_SC 1.0   # 启用航向保持

# 飞同一航线 → 着陆 → 保存日志 → 上传 Flight Review

# ═══════════════════════════════════════════════════════════
# 第 3D 轮: 加 D 控制（仅在超调 > 15% 时）
# ═══════════════════════════════════════════════════════════

# 保持 3C 的参数，额外加 D
param set FW_YR_D 0.005        # 从小值开始

# 飞同一航线 → 在 Flight Review 检查超调是否减少
# 如果效果不明显:
param set FW_YR_D 0.01

# 如果仍不够:
param set FW_YR_D 0.02

# ⚠️ 判断 D 是否过大: 看 Yaw Rate 图是否有高频毛刺
```

### 14.4 第 4 轮：铰链修正操作流程

```bash
# ═══════════════════════════════════════════════════════════
# 第 4 轮: 启用铰链修正（在第 3 轮最优参数基础上）
# ═══════════════════════════════════════════════════════════

# 保持第 3 轮最优偏航参数不变！

# 1. 启用铰链修正
param set CW_SLV_EN 1

# 2. 确认模块状态
chainwing_slave status
# 预期: Enabled: YES, Reference initialized: YES

# 3. 确认 CW_SLV_PWM_EN = 0（GZ 不需要 PWM overlay）
param show CW_SLV_PWM_EN      # 应为 0

# 4. 飞矩形航线（CW_SLV_EN=1）
# 特别关注:
#   - 转弯时 hinge_angle 变化
#   - 转弯后 hinge_angle 恢复速度
#   - 滚转跟踪精度是否改善

# 5. 着陆后，立即做对比测试
param set CW_SLV_EN 0          # 关闭修正
# 再飞一次完全相同的航线

# 6. 两次日志上传 Flight Review，用对比功能:
#    https://review.px4.io/plot_app/compare?log1=XXX&log2=YYY

# 7. 如果修正导致问题:
param set CW_SLV_KP 0.5        # 降低增益
# 或
param set CW_SLV_TRIM_MAX 0.1  # 限制修正幅度
# 再测试

# 8. PD 参数调优（如需要）:
#    KP 过大 → 高频振荡 → 降低 KP
#    KP 过小 → 无效果 → 增加 KP
#    KD 过大 → 对噪声敏感 → 降低 KD
#    KD 过小 → 铰链角超调 → 增加 KD
```

### 14.5 Stabilized 模式手动激励操作（强烈推荐！）

```
⭐⭐⭐ 重要: 每轮增加一段 Stabilized 手动飞行，
     用于 Flight Review 步阶响应分析！

步骤:
  1. 完成自主航线后，在空中切换模式:
     QGC > Fly View > 飞行模式选择器 > Stabilized
     
  2. 用遥控器（或QGC虚拟摇杆）做偏航激励:
     a. 快速打满右偏航杆 → 保持2秒 → 回中
     b. 等稳定 3 秒
     c. 快速打满左偏航杆 → 保持2秒 → 回中
     d. 等稳定 3 秒
     e. 重复 a-d 共 3 次
     
  3. 做滚转激励:
     a. 快速打满右滚杆 → 保持1秒 → 回中
     b. 等稳定 2 秒
     c. 快速打满左滚杆 → 保持1秒 → 回中
     d. 等稳定 2 秒
     
  4. 切回 Mission 模式降落:
     QGC > Fly View > Mission

为什么这很重要:
  Flight Review 的 "Step Response" 分析需要阶跃输入！
  自主飞行使用平滑设定值 → 步阶响应显示 "0"
  Stabilized 手动操作能产生明确的阶跃信号
  → 可计算出: 响应时间、超调量、调节时间
```

---

## §15 GZ SITL 飞行数据深度分析

### 15.1 Flight Review 详细分析流程

**每次飞行必须检查的 8 项指标**:

```
1. Attitude > Yaw ⚡ 最重要
   ┌────────────────────────────────────────────┐
   │  蓝线: yaw setpoint (设定值)                │
   │  红线: yaw actual (实际值)                  │
   │                                            │
   │  检查项:                                    │
   │  a. 直线段: 红蓝重合? 差距 < 5°?            │
   │  b. 转弯段: 红线超调幅度?                    │
   │  c. 转弯后: 恢复到重合的时间?                │
   │  d. 有无持续振荡?                           │
   └────────────────────────────────────────────┘

2. Rate Tracking > Yaw Rate
   ┌────────────────────────────────────────────┐
   │  检查项:                                    │
   │  a. 跟踪延迟（红线落后蓝线多少ms?）         │
   │  b. 高频噪声（如果有毛刺 → D 过大）         │
   │  c. 饱和（如果红线被截断 → P 或 FF 过大）    │
   └────────────────────────────────────────────┘

3. Attitude > Roll
   ┌────────────────────────────────────────────┐
   │  检查项:                                    │
   │  a. 第4轮 vs 第3轮: 滚转跟踪是否更紧密?     │
   │  b. 转弯时滚转超调是否减少?                  │
   │  c. 有无铰链修正引入的异常振荡?              │
   └────────────────────────────────────────────┘

4. Actuator Controls > Motors
   ┌────────────────────────────────────────────┐
   │  检查项:                                    │
   │  a. Motor 0 和 Motor 2 差异（差动推力）      │
   │  b. 差动推力幅度是否合理 (< 30%)?            │
   │  c. 持续 > 50% 差异 → 推力不足，需检查       │
   └────────────────────────────────────────────┘

5. Actuator Controls > Servos
   ┌────────────────────────────────────────────┐
   │  检查项:                                    │
   │  a. servo_0 和 servo_2 是否含 trim 偏移?     │
   │  b. 第4轮: servo_0/2 活动量 vs 第3轮比较     │
   │  c. 是否有持续饱和 (±1.0)?                   │
   │  d. 高频振荡（> 5Hz）→ KP 过大               │
   └────────────────────────────────────────────┘

6. GPS Track (Position)
   ┌────────────────────────────────────────────┐
   │  检查项:                                    │
   │  a. 矩形航线是否方正? 还是圆角过大?          │
   │  b. 直线段偏离量                             │
   │  c. 第3轮 vs 第2轮: 航迹是否更精确?          │
   └────────────────────────────────────────────┘

7. Airspeed
   ┌────────────────────────────────────────────┐
   │  检查项:                                    │
   │  a. 巡航速度是否稳定在 20 m/s?               │
   │  b. 转弯时空速波动（影响偏航增益缩放）       │
   │  c. 着陆段空速不应低于 15 m/s                │
   └────────────────────────────────────────────┘

8. Step Response (仅 Stabilized 模式段有效)
   ┌────────────────────────────────────────────┐
   │  Status 页面 > Step Response                │
   │                                            │
   │  a. Yaw step response time: 目标 < 2s      │
   │  b. Roll step response time: 目标 < 0.5s   │
   │  c. Pitch step response time: 目标 < 0.5s  │
   │                                            │
   │  如果显示 "0" → 该段无阶跃事件（正常）      │
   └────────────────────────────────────────────┘
```

### 15.2 PlotJuggler 深度分析（chainwing_hinge_status 专用）

```
Flight Review 无法显示自定义 topic！
chainwing_hinge_status 只能用 PlotJuggler 或 pyulog 分析。

方法 1: pyulog 命令行
  pip install pyulog
  ulog2csv your_flight.ulg
  
  生成的 CSV 文件中找:
  your_flight_chainwing_hinge_status_0.csv
  
  列: timestamp, hinge_angle_left, hinge_angle_right,
      hinge_rate_left, hinge_rate_right,
      trim_left, trim_right, data_valid

方法 2: PlotJuggler GUI
  安装: https://github.com/facontidavide/PlotJuggler
  或: sudo snap install plotjuggler
  
  使用:
  1. File > Load data > 选 .ulg 文件（需 PX4 ULog 插件）
     或加载 CSV 文件
  2. 拖拽变量到绘图区
  3. 推荐叠加:
     - hinge_angle_left + vehicle_attitude.roll
     - trim_left + actuator_controls_0[0] (servo0)
     - hinge_rate_left 的频谱（右键 > FFT）
```

### 15.3 GZ SITL 特有的实时监控方法

```
Gazebo SITL 比 SIH/实机多一个优势: Gazebo 端有独立观察手段

方法 1: Gazebo GUI
  - 3D 视图直观观察铰链变形
  - 如果看到翼尖明显翘起/下垂 → 铰链物理正常
  - Inspector 面板: 查看铰链角实时值

方法 2: Gazebo topic 监听
  # 终端（PX4外另开）
  gz topic -l                                      # 列出所有 topic
  # ⚠️ SITL model name = chainwing_3body_0 (带实例后缀 _0)
  gz topic -e /model/chainwing_3body_0/servo_0       # 左Elevon 实际输出
  gz topic -e /model/chainwing_3body_0/joint_state    # 铰链关节状态
  
方法 3: PX4 pxh> 实时监听
  # 每2秒打印一次铰链状态
  listener chainwing_hinge_status -r 2
  
  # 实时姿态
  listener vehicle_attitude -r 2
  
  # 实时舵面输出
  listener actuator_servos -r 2
  
  # 偏航力矩输出
  listener vehicle_torque_setpoint -r 2
```

### 15.4 数据对比分析模板

```
═══════════════════════════════════════════════════════════════
            四轮测试结果对比总表
═══════════════════════════════════════════════════════════════

                    第2轮    第3A轮   第3B轮   第3C轮   第4轮
                    基线     P=0.15  P=0.3   P=0.6   +修正
─────────────────────────────────────────────────────────────
FW_YR_P            0.05     0.15    0.3     0.6     0.6
FW_YR_D            0.0      0.0     0.0     0.0     0.0
FW_YAW_STAB_SC     0.0      0.0     0.0     1.0     1.0
CW_SLV_EN          0        0       0       0       1
─────────────────────────────────────────────────────────────
直线偏航偏差(°)     ___      ___     ___     ___     ___
转弯偏航超调(°)     ___      ___     ___     ___     ___
转弯恢复时间(s)     ___      ___     ___     ___     ___
Roll RMSE(°)       ___      ___     ___     ___     ___
GPS 航迹偏差(m)     ___      ___     ___     ___     ___
步阶响应时间(s)     ___      ___     ___     ___     ___
Flight Review URL   ___      ___     ___     ___     ___
═══════════════════════════════════════════════════════════════

结论:
  最优偏航参数: FW_YR_P = ___, FW_YR_D = ___
  航向保持: FW_YAW_STAB_SC = ___
  铰链修正效果: □改善 ___% / □无效果 / □恶化 ___%
  建议: ___
═══════════════════════════════════════════════════════════════
```

### 15.5 常见问题诊断表

| 现象 | 可能原因 | Flight Review 判断 | 解决方案 |
|------|---------|-------------------|---------|
| 直线段缓慢偏航漂移 | FW_YR_P 太小 or FW_YAW_STAB_SC=0 | Yaw: setpoint-actual 持续偏差 | 增加 FW_YR_P 或启用航向保持 |
| 转弯后偏航大幅超调 | FW_YR_P 太大 | Yaw: 尖锐的过冲脉冲 | 降低 FW_YR_P 或加 FW_YR_D |
| 高频偏航抖动 | FW_YR_D 太大 or KP 太大 | Yaw Rate: 高频毛刺 | 降低 FW_YR_D 或 CW_SLV_KP |
| 差动推力饱和 | 偏航力矩需求 > 电机能力 | Motors: Motor0 和 Motor2 持续在极值 | 降低 FW_YR_P，增加 FW_YR_FF |
| 铰链修正导致振荡 | CW_SLV_KP 过大 | Servos: servo_0/2 有周期振荡 | 降低 CW_SLV_KP (1.5→0.5) |
| 铰链修正无效果 | 铰链角估计漂移 | hinge_angle 持续单方向增大 | 检查互补滤波 cf_alpha |
| 着陆时偏航失控 | 低速下差动推力不足 | 着陆段 Yaw 大幅偏离 | 增加 FW_YR_FF 着陆前预减速 |
| servo 饱和 | trim + 控制分配超出[-1,1] | Servos: 值被截断在 ±1.0 | 降低 CW_SLV_TRIM_MAX |

---

## §17 自动化参数扫描验证 **[v3.0 新增]**

### 17.1 概述

手动逐轮参数调试效率低、易遗漏。本项目提供了 **自动化参数扫描工具**，可一键运行 22 轮仿真、自动设参、自动飞行、自动保存日志，并生成对比分析图。

**工具位置**: `scripts/` 目录

| 文件 | 功能 |
|------|------|
| `param_sweep_sitl.sh` | Bash 编排脚本 — 遍历参数组合 → 启动 SITL → 设参 → 飞行 → 保存日志 |
| `analyze_param_sweep.py` | Python 分析脚本 — 读取 .ulg → 提取数据 → 计算指标 → 生成图表 |
| `sweep_config.json` | 22 轮扫描配置 — 定义每轮参数值和阶段分组 |
| `README.md` | 脚本使用详细说明 |

### 17.2 前置条件

```bash
# 1. 安装系统依赖
sudo apt install jq

# 2. 安装 Python 分析依赖
pip3 install pyulog matplotlib numpy

# 3. 编译 PX4 SITL（首次）
cd PX4_test
DONT_RUN=1 make px4_sitl_default gz_chainwing_3body
```

### 17.3 运行扫描

#### 17.3.1 全量扫描（22 轮，约 66 分钟）

```bash
bash scripts/param_sweep_sitl.sh
```

脚本自动执行：
1. 读取 `sweep_config.json` 中的参数配置
2. 对每轮：启动 Gazebo SITL → 等待 PX4 启动 → 设置参数 → 起飞 → 盘旋 90 秒 → 着陆 → 保存日志
3. 日志保存到 `sweep_logs/` 目录，文件名包含轮次和参数描述

#### 17.3.2 单轮/单阶段运行

```bash
# 只运行第 5 轮
bash scripts/param_sweep_sitl.sh --run 5

# 只运行 Yaw P 阶段（第 2-4 轮）
bash scripts/param_sweep_sitl.sh --phase B_yaw_P

# 只运行铰链修正阶段（第 20-22 轮）
bash scripts/param_sweep_sitl.sh --phase I_hinge
```

### 17.4 扫描配置（22 轮 × 9 阶段）

| 阶段 | 名称 | 轮次 | 扫描参数 | 扫描值 | 验证目标 |
|------|------|------|----------|--------|----------|
| A | baseline | 1 | — | PX4 默认值 | 基线对照 |
| B | yaw_P | 2-4 | `FW_YR_P` | 0.15 → 0.3 → 0.6 | 偏航阻尼 |
| C | yaw_I | 5-6 | `FW_YR_I` | 0.2 → 0.5 | 偏航稳态误差 |
| D | yaw_FF | 7-8 | `FW_YR_FF` | 0.5 → 0.7 | 偏航前馈响应 |
| E | yaw_D | 9-11 | `FW_YR_D` | 0.005 → 0.01 → 0.02 | 偏航微分阻尼 |
| F | heading_hold | 12-13 | `FW_YAW_STAB_SC` | 1.0 → 2.0 | 航向保持增益 |
| G | roll_P | 14-16 | `FW_RR_P` | 0.15 → 0.3 → 0.5 | 滚转阻尼 |
| H | pitch_P | 17-19 | `FW_PR_P` | 0.2 → 0.5 → 0.9 | 俯仰响应 |
| I | hinge | 20-22 | `CW_SLV_KP`/`KD` | 1.0/0.1 → 1.5/0.2 → 2.0/0.3 | 铰链修正效果 |

**设计原则**: 每阶段仅变化一个参数（或一组相关参数），前序阶段的最优值锁定后递推。

### 17.5 分析日志

#### 17.5.1 生成图表

```bash
# 生成 PNG 对比图
python3 scripts/analyze_param_sweep.py sweep_logs/

# 指定输出目录
python3 scripts/analyze_param_sweep.py sweep_logs/ --output analysis_results/

# 同时生成 PDF 报告
python3 scripts/analyze_param_sweep.py sweep_logs/ --pdf
```

#### 17.5.2 输出文件

| 文件 | 内容 |
|------|------|
| `01_attitude_comparison.png` | 多轮 Roll/Pitch/Yaw 姿态角时间序列 |
| `02a_yaw_rate_tracking.png` | 各轮偏航角速率跟踪精度 |
| `02b_roll_rate_tracking.png` | 各轮滚转角速率跟踪精度 |
| `02c_pitch_rate_tracking.png` | 各轮俯仰角速率跟踪精度 |
| `03_servo_output.png` | 舵面输出对比 |
| `04_hinge_correction.png` | 铰链修正效果对比（角度 + 修正量） |
| `05_metrics_comparison.png` | 9 项性能指标柱状图对比（3×3 布局） |
| `06_metrics_table.png` | 全轮次性能汇总表格 |
| `metrics_summary.csv` | CSV 格式汇总（可导入 Excel） |

### 17.6 性能判定标准

| 指标 | 优秀 | 合格 | 不合格 |
|------|------|------|--------|
| Yaw Rate RMS | < 5 °/s | < 10 °/s | > 15 °/s |
| Roll Rate RMS | < 3 °/s | < 8 °/s | > 12 °/s |
| Pitch Rate RMS | < 3 °/s | < 8 °/s | > 12 °/s |
| Roll 偏差 RMS | < 2° | < 5° | > 8° |
| Pitch 偏差 RMS | < 1° | < 3° | > 5° |
| Yaw 偏差 RMS | < 3° | < 8° | > 15° |
| Hinge Angle RMS | < 2° | < 5° | > 10° |
| Max Hinge Angle | < 5° | < 10° | > 15° |

### 17.7 典型工作流

```
1. 全量扫描 → 找到每阶段最优参数
   bash scripts/param_sweep_sitl.sh

2. 分析对比 → 确认改善幅度
   python3 scripts/analyze_param_sweep.py sweep_logs/

3. 针对性精调 → 在最优附近细化
   编辑 sweep_config.json，收窄范围
   bash scripts/param_sweep_sitl.sh --phase B_yaw_P

4. 最终确认 → 锁定参数写入机架文件
   将最优值更新到 4008_gz_chainwing_3body / 2150_chainwing
```

### 17.8 与手动验证的关系

| 方面 | 自动化扫描 (§17) | 手动验证 (§3-§6) |
|------|-------------------|-------------------|
| 适用阶段 | 参数粗调/快速对比 | 最终确认/边界验证 |
| 精度 | 中（固定飞行模式） | 高（自定义机动） |
| 效率 | 22 轮 ≈ 66 分钟 | 22 轮 ≈ 数天 |
| 输出 | PNG 图表 + CSV | Flight Review 链接 |
| 覆盖度 | 全参数扫描 | 关注特定场景 |

**推荐**: 先用自动化扫描找到大致最优区间 → 再用手动验证精调边界条件。

---

## §16 版本历史

| 版本 | 日期 | 变更 |
|------|------|------|
| v1.0 | 2026-03-24 | 初始版本: 四轮递进测试计划（SIH+Gazebo） |
| v2.0 | 2026-03-25 | 新增 §12-§15: GZ SITL 专项验证指南（通信架构图、控制律详解、操作流程、深度分析） |
| v3.0 | 2026-03-26 | 新增 §17: 自动化参数扫描验证（22轮×9阶段脚本 + 分析工具 + 判定标准）；同步 chainwing_master 模块和双模铰链角更新 |
