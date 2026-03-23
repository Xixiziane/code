# 链翼 FMU-V3 硬件部署评估与实施建议

> 版本：v1.0 | 日期：2026-03-23 | 目标飞控：px4_fmu-v3 (STM32F427)

---

## 目录

1. [总体评估结论](#1-总体评估结论)
2. [当前项目状态盘点](#2-当前项目状态盘点)
3. [FMU-V3 硬件约束分析](#3-fmu-v3-硬件约束分析)
4. [核心技术差距分析](#4-核心技术差距分析)
5. [工作量与难度评估](#5-工作量与难度评估)
6. [阶段一：编译上板（1-2天）](#6-阶段一编译上板12天)
7. [阶段二：PWM 舵面修正（2-3天）](#7-阶段二pwm-舵面修正23天)
8. [阶段三：SIH 硬件在环（1-2天）](#8-阶段三sih-硬件在环12天)
9. [阶段四：真机实飞（3-5天）](#9-阶段四真机实飞35天)
10. [完整代码修改清单](#10-完整代码修改清单)
11. [测试验证计划](#11-测试验证计划)
12. [风险评估与建议](#12-风险评估与建议)
13. [FAQ](#13-faq)

---

## 1. 总体评估结论

### 可行性：✅ 完全可行

| 维度 | 评估 |
|------|------|
| **总工作量** | **约 7-12 个工作日**（1-2周） |
| **新增代码量** | **约 61 行 C++**（必须） + 20 行（可选文档） |
| **修改文件数** | **6 个文件**（必须） + 2 个（可选文档） |
| **技术难度** | **中等偏低** ⭐⭐⭐☆☆ |
| **最大风险** | Flash 空间不足（需验证） |
| **最大挑战** | PWM trim 叠加机制（替代 GZMixingInterfaceServo） |

### 关键发现

| 项目 | 状态 | 说明 |
|------|------|------|
| fmu-v3 板级配置 | ✅ 存在 | `boards/px4/fmu-v3/` 完整 |
| chainwing_slave 在 fmu-v3 上编译 | ❌ 未启用 | 需加 `CONFIG_MODULES_CHAINWING_SLAVE=y` |
| 硬件机架文件 | ⚠️ 不完整 | `2150_chainwing` 缺少 CW_SLV_* 参数 |
| SIH 机架文件 | ⚠️ 不完整 | `1103_chainwing_sih.hil` 缺少 CW_SLV_* 参数 |
| PWM trim 叠加 | ❌ 不存在 | GZMixingInterfaceServo 是 Gazebo 专用 |
| PWM 驱动 | ✅ 已有 | `CONFIG_DRIVERS_PWM_OUT=y` 已在 fmu-v3 启用 |
| UART 通信 | ✅ 已有 | MAVLink DEBUG_FLOAT_ARRAY 代码已在 ChainwingSlave.cpp |
| IMU 铰链估计 | ✅ 已有 | 互补滤波器代码无需修改，硬件 IMU 数据格式一致 |

---

## 2. 当前项目状态盘点

### 2.1 已完成的可直接复用的代码

| 模块 | 文件 | 硬件可用度 | 说明 |
|------|------|-----------|------|
| 铰链估计算法 | ChainwingSlave.cpp:131-205 | ✅ 100% | 纯 uORB + IMU，与平台无关 |
| PD trim 控制律 | ChainwingSlave.cpp:207-220 | ✅ 100% | 纯数学计算 |
| 参数系统 | chainwing_slave_params.c | ✅ 100% | PX4 标准参数接口 |
| MAVLink 通信 | ChainwingSlave.cpp:222-267 | ✅ 100% | 已实现 DEBUG_FLOAT_ARRAY |
| uORB 消息定义 | ChainwingHingeStatus.msg | ✅ 100% | 与平台无关 |
| 日志记录 | logged_topics.cpp:57 | ✅ 100% | 已注册 optional topic |

### 2.2 仅限仿真的代码（需替代）

| 模块 | 文件 | 问题 | 解决方案 |
|------|------|------|---------|
| Trim 叠加 | GZMixingInterfaceServo.cpp | Gazebo 专用 | 新增 PWM trim 叠加 |
| 铰链物理 | model.sdf | Gazebo SDF | SIH 无铰链；真机用物理铰链 |

### 2.3 需要补充的配置

| 文件 | 缺失内容 |
|------|---------|
| boards/px4/fmu-v3/default.px4board | `CONFIG_MODULES_CHAINWING_SLAVE=y` |
| 2150_chainwing | CW_SLV_* 参数 + `chainwing_slave start` |
| 1103_chainwing_sih.hil | CW_SLV_* 参数 + `chainwing_slave start` |

---

## 3. FMU-V3 硬件约束分析

### 3.1 处理器与存储

```
┌───────────────────────────────────────┐
│  STM32F427VIT6 (Cortex-M4F, 180MHz)  │
├───────────┬───────────────────────────┤
│  Flash    │  2048 KB (2 MB)           │
│  可用Flash │  2032 KB (去掉 bootloader)│
│  SRAM     │  256 KB                   │
│  TCM      │  64 KB (快速内存)         │
│  FPU      │  ✅ 硬件浮点单元          │
├───────────┼───────────────────────────┤
│  约束标记  │  CONFIG_BOARD_CONSTRAINED │
│           │  _MEMORY = y              │
└───────────┴───────────────────────────┘
```

### 3.2 chainwing_slave 模块资源占用估算

```
ChainwingSlave.cpp (377行) + ChainwingSlave.hpp (~170行)
├── 代码段 (.text)      ≈ 8-12 KB
├── 只读数据 (.rodata)   ≈ 1-2 KB (字符串+参数定义)
├── 初始化数据 (.data)   ≈ 0.5 KB
├── BSS (.bss)          ≈ 1-2 KB (类成员变量)
├── 栈空间              ≈ 2 KB (任务栈)
└── 合计                ≈ 15-20 KB Flash + 4 KB RAM
```

**结论：** 对 2032KB Flash 来说仅增加 ~1%，完全可接受。

### 3.3 可用 UART 端口

| 端口 | 设备名 | 默认用途 | 可用性 |
|------|--------|---------|--------|
| UART1 | /dev/ttyS1 | TEL1 (数传) | 已占用 |
| UART2 | /dev/ttyS2 | TEL2 (空闲) | ✅ **推荐用于从机通信** |
| UART3 | /dev/ttyS3 | GPS | 已占用 |
| UART6 | /dev/ttyS6 | TEL4 (空闲) | ✅ 备用 |

### 3.4 PWM 输出通道

```
FMU-V3 PWM 输出:
├── MAIN OUT 1-8:  通过 IO 协处理器 (PX4IO)
│   ├── MAIN 1-3: 电机 (Motor 0/1/2)
│   └── MAIN 4-6: 舵面 (Servo 0/1/2) ← trim 叠加在这里
└── AUX OUT 1-6:  直接 FMU PWM
    └── 可用于额外功能
```

---

## 4. 核心技术差距分析

### 4.1 最关键的差距：PWM Trim 叠加

**问题：**
```
SITL 控制链路:
  control_allocator → actuator_servos → GZMixingInterfaceServo → [+trim] → Gazebo
                                         ↑ 这里做 trim 叠加
                                         ↑ 但这个模块只在仿真中存在！

硬件控制链路:
  control_allocator → actuator_servos → PWMOut → PWM信号 → 舵机
                                        ↑ 这里没有 trim 叠加！
```

**解决方案（3 种可选）：**

| 方案 | 原理 | 代码量 | 难度 | 推荐 |
|------|------|--------|------|------|
| A. 修改 actuator_servos | 在控制分配后、PWM 输出前叠加 | ~30 行 | ⭐⭐ | ⭐⭐⭐⭐ |
| B. 修改 PWMOut | 在 PWM 驱动中叠加 | ~40 行 | ⭐⭐⭐ | ⭐⭐⭐ |
| C. 修改 ChainwingSlave | 直接发布修正后的 actuator 值 | ~50 行 | ⭐⭐ | ⭐⭐⭐⭐⭐ |

**推荐方案 C：在 ChainwingSlave 中直接修改 actuator_servos**

```
优点：
1. 修改集中在自定义模块内，不动 PX4 原生代码
2. SITL 和硬件可以用同一套代码
3. 降低与 PX4 上游合并冲突的风险
```

### 4.2 方案 C 实现原理

```
修改后的控制链路：

control_allocator → actuator_servos (原始值)
                         ↓
                  ChainwingSlave 订阅 → 读取 servo[0], servo[2]
                         ↓ 叠加 trim
                  发布 actuator_servos_trim (修正后)
                         ↓
                    PWMOut 订阅 → 输出 PWM
```

**核心代码（~50行新增）：**

```cpp
// 在 ChainwingSlave::Run() 的 publish 之后添加：

// 方案 C：直接修改 actuator_servos 实现硬件 trim 叠加
// 仅在非仿真环境下启用（仿真走 GZMixingInterfaceServo）
#if !defined(CONFIG_MODULES_SIMULATION_GZ_BRIDGE)

actuator_servos_s servos{};
if (_actuator_servos_sub.copy(&servos)) {
    // 叠加 trim 到左右 elevon
    servos.control[0] += trim_left;   // Servo 0: Left elevon
    servos.control[2] += trim_right;  // Servo 2: Right elevon

    // 限幅
    servos.control[0] = math::constrain(servos.control[0], -1.0f, 1.0f);
    servos.control[2] = math::constrain(servos.control[2], -1.0f, 1.0f);

    // 更新时间戳并重新发布
    servos.timestamp = hrt_absolute_time();
    _actuator_servos_pub.publish(servos);
}

#endif
```

**⚠️ 重要注意：** 这需要验证 PWMOut 是否会从修改后的 topic 读取。
实际实现中，更安全的方式是通过 `actuator_outputs` 而非直接改 servos。
下面阶段二会给出完整实现方案。

---

## 5. 工作量与难度评估

### 5.1 各阶段工作量

| 阶段 | 内容 | 代码量 | 人天 | 难度 | 前提 |
|------|------|--------|------|------|------|
| **一** | 编译上板 | ~5 行配置 | 1-2天 | ⭐ | 有 fmu-v3 硬件 |
| **二** | PWM trim 叠加 | ~50 行 C++ | 2-3天 | ⭐⭐⭐ | 阶段一完成 |
| **三** | SIH 硬件在环 | ~20 行配置 | 1-2天 | ⭐⭐ | 阶段一完成 |
| **四** | 真机实飞 | ~10 行配置 | 3-5天 | ⭐⭐⭐⭐ | 阶段二完成 |
| **合计** | | **~61 行** | **7-12天** | | |

### 5.2 难度评分详解

```
难度分解：
├── ⭐     编译配置（修改 .px4board + 机架文件）
├── ⭐⭐   参数配置（添加 CW_SLV_* 参数到机架文件）
├── ⭐⭐⭐  PWM trim 叠加（需理解 PX4 actuator pipeline）
├── ⭐⭐⭐  UART MAVLink 通信调试（硬件连线+协议验证）
└── ⭐⭐⭐⭐ 真机首飞调试（PID 调参 + 安全检查 + 天气等）
```

### 5.3 与其他方案的对比

| 方案 | 代码量 | 时间 | 风险 |
|------|--------|------|------|
| **本项目方案（方案 C）** | ~85 行 | 7-12天 | 低 |
| 自定义 MAVLink 消息 | ~300 行 | 15-20天 | 中 |
| 修改 PX4 原生 PWMOut | ~60 行 | 5-8天 | 高（影响升级） |
| 外部微控制器方案 | ~500 行 | 20-30天 | 高（额外硬件） |

---

## 6. 阶段一：编译上板（1-2天）

### 6.1 步骤 1：启用 chainwing_slave 模块

**文件：** `boards/px4/fmu-v3/default.px4board`

```diff
# 在文件末尾或 MODULES 区域添加：
+CONFIG_MODULES_CHAINWING_SLAVE=y
```

### 6.2 步骤 2：编译固件

```bash
# 编译 fmu-v3 固件
make px4_fmu-v3_default

# 检查固件大小
ls -la build/px4_fmu-v3_default/px4_fmu-v3_default.px4
# 确保 < 2032 KB
```

### 6.3 步骤 3：烧录并验证

```bash
# 烧录
make px4_fmu-v3_default upload

# 在 NSH console 验证模块可用
nsh> chainwing_slave status
# 应显示 "not running" (因为还没启动)

nsh> chainwing_slave start
nsh> chainwing_slave status
# 应显示 running + 参数值
```

### 6.4 预期问题与解决

| 问题 | 可能性 | 解决方案 |
|------|--------|---------|
| Flash 空间不足 | 低（模块仅 ~20KB） | 禁用不需要的模块 |
| 编译错误 | 极低（代码已在 SITL 验证） | 检查平台宏定义 |
| 参数未加载 | 中（需要机架文件） | 见阶段三/四 |

---

## 7. 阶段二：PWM 舵面修正（2-3天）

### 7.1 核心问题

当前 trim 叠加在 `GZMixingInterfaceServo.cpp` 中，这个文件仅在 Gazebo 仿真中存在。
硬件上需要一个新的机制来实现相同功能。

### 7.2 推荐实现方案

**在 ChainwingSlave.cpp 中添加 actuator_servos 修改功能：**

**原理：**
1. ChainwingSlave 订阅 `actuator_servos`（控制分配器的输出）
2. 读取原始 servo 值
3. 叠加 trim 修正
4. 重新发布到 `actuator_servos`（PWMOut 会读取最新值）

### 7.3 需要修改的文件

**文件 1：ChainwingSlave.hpp** — 添加新的订阅/发布器

```cpp
// 新增头文件
#include <uORB/topics/actuator_servos.h>

// 在类成员中新增：
// === 硬件 PWM trim 叠加 ===
uORB::Subscription _actuator_servos_sub{ORB_ID(actuator_servos)};
uORB::Publication<actuator_servos_s> _actuator_servos_pub{ORB_ID(actuator_servos)};

// 新增参数：PWM trim 叠加使能（硬件模式下为 1）
DEFINE_PARAMETERS(
    // ... 已有参数 ...
    (ParamInt<px4::params::CW_SLV_PWM_EN>) _param_pwm_trim_enable
)
```

**文件 2：chainwing_slave_params.c** — 添加 PWM trim 使能参数

```c
/**
 * Enable PWM trim overlay for hardware deployment.
 *
 * When enabled, ChainwingSlave directly modifies actuator_servos
 * to apply hinge trim corrections. This is the hardware replacement
 * for GZMixingInterfaceServo (simulation-only).
 *
 * Set to 1 for hardware/SIH, 0 for Gazebo SITL (uses GZMixingInterfaceServo).
 *
 * @boolean
 * @group Chain-Wing Slave
 */
PARAM_DEFINE_INT32(CW_SLV_PWM_EN, 0);
```

**文件 3：ChainwingSlave.cpp** — 在 Run() 中添加 PWM trim 叠加

```cpp
// 在现有的 _hinge_status_pub.publish(status) 之后添加：

// === 硬件 PWM Trim 叠加 ===
if (_param_pwm_trim_enable.get() != 0 && _ref_initialized) {
    actuator_servos_s servos{};

    if (_actuator_servos_sub.copy(&servos)) {
        // 叠加 trim 到左右 elevon（与 GZMixingInterfaceServo 逻辑一致）
        servos.control[0] += trim_left;   // Servo 0: Left elevon
        servos.control[2] += trim_right;  // Servo 2: Right elevon

        // 限幅到 [-1, 1]
        servos.control[0] = math::constrain(servos.control[0], -1.0f, 1.0f);
        servos.control[2] = math::constrain(servos.control[2], -1.0f, 1.0f);

        // 更新时间戳并发布
        servos.timestamp = hrt_absolute_time();
        _actuator_servos_pub.publish(servos);
    }
}
```

### 7.4 验证方法

```bash
# 在 SIH 模式下测试 (不需要真机)
nsh> param set CW_SLV_EN 1
nsh> param set CW_SLV_PWM_EN 1
nsh> chainwing_slave start

# 检查 actuator_servos 是否包含 trim
nsh> listener actuator_servos -n 5
# 对比启用/禁用 CW_SLV_PWM_EN 的输出差异
```

### 7.5 ⚠️ 重要注意

1. **SITL 模式下应保持 CW_SLV_PWM_EN=0**，因为仿真走 GZMixingInterfaceServo
2. **actuator_servos 的重发布可能引起竞争条件**，需确保 ChainwingSlave 运行频率 ≥ PWMOut
3. 如果 `_actuator_servos_sub.update()` 返回 false（无新数据），则跳过——不会覆盖有效值

---

## 8. 阶段三：SIH 硬件在环（1-2天）

### 8.1 SIH 模式说明

```
SIH (Simulation-In-Hardware) = 硬件板上运行物理引擎
优点：测试真实硬件 I/O + 参数系统 + 启动流程
限制：SIH 物理引擎是单刚体，不模拟铰链
结果：铰链角始终为 0，但可验证代码流程正确性
```

### 8.2 修改 SIH 机架文件

**文件：** `ROMFS/px4fmu_common/init.d/airframes/1103_chainwing_sih.hil`

```diff
# 在文件末尾（约 line 130）添加：

+# ============================
+# Chain-Wing 从机控制器参数
+# ============================
+param set-default CW_SLV_EN 1
+param set-default CW_SLV_KP 1.5
+param set-default CW_SLV_KD 0.2
+param set-default CW_SLV_TRIM_MAX 0.3
+param set-default CW_SLV_COMM_EN 0
+param set-default CW_SLV_PWM_EN 1
+
+# 启动从机模块
+chainwing_slave start
```

### 8.3 SIH 验证步骤

```bash
# 1. 编译并烧录
make px4_fmu-v3_default upload

# 2. 设置机架
nsh> param set SYS_AUTOSTART 1103
nsh> reboot

# 3. 验证模块运行
nsh> chainwing_slave status
# 应显示 running, KP=1.5, KD=0.2

# 4. 检查 trim 输出
nsh> listener chainwing_hinge_status
# 应显示 hinge_angle_left ≈ 0, trim_left ≈ 0 (SIH无铰链物理)

# 5. 验证 PWM 叠加
nsh> listener actuator_servos -n 3
# 对比 CW_SLV_PWM_EN=0 和 =1 的差异
```

---

## 9. 阶段四：真机实飞（3-5天）

### 9.1 修改硬件机架文件

**文件：** `ROMFS/px4fmu_common/init.d/airframes/2150_chainwing`

```diff
# 在文件末尾添加：

+# ============================
+# Chain-Wing 从机控制器参数
+# ============================
+param set-default CW_SLV_EN 1
+param set-default CW_SLV_KP 1.5
+param set-default CW_SLV_KD 0.2
+param set-default CW_SLV_TRIM_MAX 0.3
+param set-default CW_SLV_COMM_EN 0
+param set-default CW_SLV_PWM_EN 1
+
+# 启动从机模块
+chainwing_slave start
```

### 9.2 真机安全检查清单

```
首飞前必须完成：
□ 1. 地面测试：手动转动铰链 → listener 显示角度变化
□ 2. 地面测试：trim 方向正确（铰链右偏 → 左 elevon 补偿）
□ 3. 地面测试：trim 幅度在安全范围（<= 0.3）
□ 4. 地面测试：CW_SLV_EN=0 → trim=0（安全关断）
□ 5. 遥控器杀开关配置（紧急禁用）
□ 6. 低速滑行测试
□ 7. 短距离低空飞行
□ 8. 逐步增加飞行包线
```

### 9.3 PID 调参建议

```
首飞参数（保守值）：
  CW_SLV_KP = 0.5      # 从低开始
  CW_SLV_KD = 0.1      #
  CW_SLV_TRIM_MAX = 0.15  # 限制最大修正量

逐步增加：
  第2飞：KP = 1.0, TRIM_MAX = 0.2
  第3飞：KP = 1.5, TRIM_MAX = 0.3 (目标值)
```

### 9.4 三机分布式架构（可选进阶）

```
如果使用 3 台 Pixhawk（每个机翼单元一台）：

主机 (Center Pixhawk):
  - 运行标准 FW 控制栈
  - UART2 连接到左从机
  - UART6 连接到右从机
  - mavlink start -x -d /dev/ttyS2 -b 921600 -m custom
  - mavlink stream -d /dev/ttyS2 -s DEBUG_FLOAT_ARRAY -r 10

从机 (Left/Right Pixhawk):
  - 运行 chainwing_slave
  - CW_SLV_COMM_EN = 1
  - mavlink start -d /dev/ttyS2 -b 921600 -m custom
```

---

## 10. 完整代码修改清单

### 10.1 必须修改的文件（最小集）

| # | 文件 | 修改类型 | 行数 | 阶段 |
|---|------|---------|------|------|
| 1 | boards/px4/fmu-v3/default.px4board | 添加 1 行 | +1 | 一 |
| 2 | chainwing_slave_params.c | 添加 1 个参数 | +15 | 二 |
| 3 | ChainwingSlave.hpp | 添加订阅器+参数声明 | +5 | 二 |
| 4 | ChainwingSlave.cpp | 添加 PWM trim 逻辑 | +20 | 二 |
| 5 | 1103_chainwing_sih.hil | 添加 CW_SLV_* 参数 | +10 | 三 |
| 6 | 2150_chainwing | 添加 CW_SLV_* 参数 | +10 | 四 |

**合计：6 个文件，约 61 行新增代码**

### 10.2 可选修改

| # | 文件 | 修改内容 | 阶段 |
|---|------|---------|------|
| 7 | CHAINWING_PARAMETER_REFERENCE.md | 添加 CW_SLV_PWM_EN 说明 | 二 |
| 8 | CHAINWING_COMMUNICATION_GUIDE.md | 添加硬件部署章节 | 四 |

---

## 11. 测试验证计划

### 11.1 阶段验证矩阵

| 测试项 | 阶段一 | 阶段二 | 阶段三 | 阶段四 |
|--------|--------|--------|--------|--------|
| 编译通过 | ✅ | ✅ | ✅ | ✅ |
| 固件大小 < 2032KB | ✅ | ✅ | ✅ | ✅ |
| 模块可启动 | ✅ | ✅ | ✅ | ✅ |
| 参数可读写 | ✅ | ✅ | ✅ | ✅ |
| hinge_status 发布 | | ✅ | ✅ | ✅ |
| PWM trim 叠加 | | ✅ | ✅ | ✅ |
| SIH 全流程 | | | ✅ | |
| 地面铰链测试 | | | | ✅ |
| 低速滑行 | | | | ✅ |
| 首飞验证 | | | | ✅ |

### 11.2 关键验证命令

```bash
# 编译检查
make px4_fmu-v3_default 2>&1 | tail -5

# 固件大小检查
arm-none-eabi-size build/px4_fmu-v3_default/px4_fmu-v3_default.elf

# 运行时检查
nsh> chainwing_slave status
nsh> listener chainwing_hinge_status
nsh> listener actuator_servos
nsh> param show CW_*

# 日志检查（飞行后）
# 下载 .ulg 文件到 logs.px4.io 查看 chainwing_hinge_status
```

---

## 12. 风险评估与建议

### 12.1 风险矩阵

| 风险 | 可能性 | 影响 | 缓解措施 |
|------|--------|------|---------|
| Flash 空间不足 | 低 | 高 | 禁用不需要的模块 |
| actuator_servos 竞争条件 | 中 | 中 | 确保 50Hz 足够快 |
| PWM 输出抖动 | 低 | 中 | 增加低通滤波 |
| 铰链估计漂移（真机） | 中 | 中 | 调整互补滤波器参数 |
| UART 通信延迟（三机） | 中 | 低 | 10Hz 足够，增加超时保护 |
| 首飞坠机 | 低 | 极高 | 保守参数 + 渐进测试 |

### 12.2 关键建议

1. **先做阶段一和三**（SIH）—— 零飞行风险，验证软件全流程
2. **阶段二的 actuator_servos 修改需仔细测试** —— 这是唯一有风险的代码修改
3. **首飞保守参数** —— KP=0.5, TRIM_MAX=0.15，逐步增大
4. **保留 CW_SLV_EN=0 作为安全开关** —— 任何异常立即禁用
5. **单 Pixhawk 方案先行** —— 三机分布式方案是进阶选项
6. **真机的铰链传感器** —— IMU 互补滤波器可能在真实振动环境下需要调参

### 12.3 推荐开发顺序

```
第 1 天：阶段一（编译上板，验证模块可运行）
第 2-3 天：阶段二（PWM trim 代码，SITL 对比验证）
第 4 天：阶段三（SIH 全流程验证）
第 5-7 天：阶段四 前期（地面测试，铰链响应验证）
第 8-12 天：阶段四 后期（渐进飞行测试）
```

---

## 13. FAQ

### Q1: fmu-v3 的 Flash 够用吗？

**A:** 够用。chainwing_slave 模块约 15-20KB，fmu-v3 有 2032KB Flash。
即使当前固件用了 1800KB，剩余 200+KB 也完全足够。

### Q2: 为什么不直接修改 PWMOut 驱动？

**A:** 修改 PWMOut 属于修改 PX4 原生代码，会增加后续合并上游更新的冲突风险。
方案 C（在 ChainwingSlave 内部修改 actuator_servos）将所有自定义逻辑封装在自定义模块中。

### Q3: SIH 模式能测试铰链修正吗？

**A:** 不能测试铰链物理（SIH 是单刚体），但可以验证：
- 模块启动和参数加载
- PWM trim 叠加机制
- MAVLink 通信
- 日志记录

### Q4: 真机需要铰链角度传感器吗？

**A:** 当前代码使用 IMU 互补滤波器估计铰链角，不需要额外传感器。
但真机的振动环境可能影响估计精度，可选方案：
- 调整互补滤波器时间常数（_tau = 2.0s）
- 添加编码器/电位器传感器（进阶方案，需额外代码）

### Q5: 三机分布式和单机方案怎么选？

**A:**
- **首选单 Pixhawk 方案**：简单可靠，所有代码在一个进程，通过 uORB 通信
- **三机方案**：适合未来扩展，每个机翼有独立飞控，通过 UART MAVLink 通信
- **建议路径**：单机先飞通 → 三机后续升级

### Q6: CW_SLV_PWM_EN 和 GZMixingInterfaceServo 会冲突吗？

**A:** 不会。
- SITL：CW_SLV_PWM_EN=0，trim 通过 GZMixingInterfaceServo 叠加
- 硬件：CW_SLV_PWM_EN=1，GZMixingInterfaceServo 不存在（不编译）
- 两条路径互斥，由参数控制

---

*文档结束 — 如有疑问，参考 CHAINWING_HIL_REALFLIGHT_GUIDE.md 获取更详细的架构设计*
