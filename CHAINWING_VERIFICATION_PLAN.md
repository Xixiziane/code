# ChainWing 验证测试计划

> **版本**: v1.0  
> **日期**: 2026-03-24  
> **目标**: 系统性验证通信、基础飞控、偏航增强、从机铰链修正  
> **平台**: Pixhawk 2.4.8 (FMU-V3) + SIH 在环仿真 / Gazebo SITL  
> **工具**: QGC 参数修改 + 自主飞行任务 + Flight Review 回看分析

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
