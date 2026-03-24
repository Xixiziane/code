# 链翼 FMU-V3 硬件部署评估与实施指南

> 版本：v2.0 | 日期：2026-03-24 | 目标飞控：**Pixhawk 2.4.8** (px4_fmu-v3 / STM32F427)
>
> ✅ **v2.0 更新**：代码已实现！PWM Trim 叠加（方案 C）+ 板级配置 + 机架文件全部完成。
> 新增 §A「硬件/实机完整控制流程」详细阐述三种部署架构的信息流。

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
14. [**附录 A：硬件/实机完整控制流程**](#a-硬件实机完整控制流程)
    - [A.1 三种部署架构总览](#a1-三种部署架构总览)
    - [A.2 架构一：单 Pixhawk 实机控制流程](#a2-架构一单-pixhawk-实机控制流程)
    - [A.3 架构二：SIH 硬件在环控制流程](#a3-架构二sih-硬件在环控制流程)
    - [A.4 架构三：三 Pixhawk 分布式控制流程](#a4-架构三三-pixhawk-分布式控制流程)
    - [A.5 各飞控的电机/舵机输出分配](#a5-各飞控的电机舵机输出分配)
    - [A.6 姿态设定值输入方式](#a6-姿态设定值输入方式)
    - [A.7 PWM Trim 叠加实现细节](#a7-pwm-trim-叠加实现细节)
    - [A.8 控制频率与时序分析](#a8-控制频率与时序分析)
    - [A.9 安全机制](#a9-安全机制)
15. [版本历史](#15-版本历史)

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
| fmu-v3 板级配置 | ✅ 完成 | `boards/px4/fmu-v3/` + `CONFIG_MODULES_CHAINWING_SLAVE=y` |
| chainwing_slave 在 fmu-v3 上编译 | ✅ 已启用 | 已添加到 default.px4board |
| 硬件机架文件 | ✅ 完成 | `2150_chainwing` 含 CW_SLV_* + chainwing_slave start |
| SIH 机架文件 | ✅ 完成 | `1103_chainwing_sih.hil` 含 CW_SLV_* + chainwing_slave start |
| PWM trim 叠加 | ✅ 已实现 | 方案 C: ChainwingSlave 直接修改 actuator_servos |
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

## 3. Pixhawk 2.4.8 硬件约束分析

> 📷 参考：`飞控.jpg` — Pixhawk 2.4.8 接口引脚图

### 3.1 处理器与存储

```
┌───────────────────────────────────────┐
│  Pixhawk 2.4.8 (双 MCU 架构)         │
├───────────────────────────────────────┤
│  主处理器 FMU:                        │
│  STM32F427VIT6 (Cortex-M4F, 180MHz)  │
├───────────┬───────────────────────────┤
│  Flash    │  2048 KB (2 MB)           │
│  可用Flash │  2032 KB (去掉 bootloader)│
│  SRAM     │  256 KB                   │
│  TCM      │  64 KB (快速内存)         │
│  FPU      │  ✅ 硬件浮点单元          │
├───────────┼───────────────────────────┤
│  IO协处理器 PX4IO:                     │
│  STM32F100 (Cortex-M3, 24MHz)        │
│  用途：MAIN OUT 1-8 PWM输出           │
│  SYS_USE_IO = 1 (默认启用)            │
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

### 3.3 Pixhawk 2.4.8 接口布局（对照 飞控.jpg）

```
┌─────────────────────────────────────────────────────────────┐
│                    Pixhawk 2.4.8 顶视图                      │
│                                                             │
│  ┌─────┐  ┌──────┐  ┌──────┐  ┌────┐  ┌──────┐  ┌──────┐  │
│  │DSM  │  │TELEM2│  │TELEM1│  │SPKT│  │SERIAL│  │BUZZER│  │
│  │     │  │+SD卡 │  │      │  │/DSM│  │  /5  │  │      │  │
│  └─────┘  └──────┘  └──────┘  └────┘  └──────┘  └──────┘  │
│                                                             │
│  ┌─────┐  ┌──────┐  ┌─────┐  ┌─────┐  ┌──────┐  ┌──────┐  │
│  │USB  │  │ADC   │  │ SPI │  │ I2C │  │POWER │  │ GPS  │  │
│  │     │  │3.3V  │  │     │  │     │  │      │  │      │  │
│  └─────┘  └──────┘  └─────┘  └─────┘  └──────┘  └──────┘  │
│                                                             │
│  ┌────────────────────────┐  ┌────────────────────────────┐ │
│  │  MAIN OUT 1-8 (PX4IO) │  │  AUX OUT 1-6 (FMU直接)    │ │
│  └────────────────────────┘  └────────────────────────────┘ │
│                    ┌──────┐                                  │
│                    │ CAN  │                                  │
│                    └──────┘                                  │
└─────────────────────────────────────────────────────────────┘
```

### 3.4 可用 UART 端口（基于 Pixhawk 2.4.8 实际硬件）

| 物理接口 | 设备名 | USART | 默认用途 | 流控 | 可用性 |
|---------|--------|-------|---------|------|--------|
| **TELEM1** | /dev/ttyS1 | USART2 | 数传/GCS | RTS/CTS | ⚠️ 已占用（QGC） |
| **TELEM2** | /dev/ttyS2 | USART3 | 空闲 | RTS/CTS | ✅ **推荐用于从机通信** |
| **GPS** | /dev/ttyS3 | USART1 | GPS 模块 | — | ❌ 已占用 |
| **PX4IO** | /dev/ttyS4 | USART6 | IO协处理器 | — | ❌ 系统内部 |
| **SERIAL5** | /dev/ttyS6 | UART4 | 空闲 | — | ✅ 备用（无流控） |

> 📌 **TELEM2 推荐理由**：
> - 硬件流控 (RTS/CTS) 保证高速率传输可靠性
> - 有 DMA 支持 (DMA1 Stream4 Ch7 TX)
> - 6 针 DF13 接口，方便接线
> - 位于飞控板侧面，便于走线

### 3.5 TELEM2 接口引脚定义（6 针 DF13）

```
TELEM2 连接器 (DF13-6P)：
┌─────────────────────────────────┐
│  Pin 1: +5V   (供电给从机)      │
│  Pin 2: TX    → 从机 RX         │
│  Pin 3: RX    ← 从机 TX         │
│  Pin 4: CTS   (可选流控)        │
│  Pin 5: RTS   (可选流控)        │
│  Pin 6: GND   (共地)            │
└─────────────────────────────────┘

与从机 Pixhawk 连线：
┌──────────────────┐         ┌──────────────────┐
│  主机 TELEM2     │         │  从机 TELEM2     │
│  Pin 1: +5V   ───┤─ (不连) ├── Pin 1: +5V    │
│  Pin 2: TX    ───┤────────→├── Pin 3: RX     │
│  Pin 3: RX    ───┤←────────├── Pin 2: TX     │
│  Pin 4: CTS   ───┤─ (可选) ├── Pin 5: RTS    │
│  Pin 5: RTS   ───┤─ (可选) ├── Pin 4: CTS    │
│  Pin 6: GND   ───┤────────→├── Pin 6: GND    │
└──────────────────┘         └──────────────────┘

⚠️ 注意：TX↔RX 交叉连接！不要连 +5V（各自独立供电）
```

### 3.6 PWM 输出通道（Pixhawk 2.4.8 特有双 MCU 架构）

```
Pixhawk 2.4.8 PWM 输出（双 MCU）:

MAIN OUT 1-8 (通过 PX4IO 协处理器):
├── MAIN 1: Motor 0 — 左机翼电机
├── MAIN 2: Motor 1 — 中央机翼电机
├── MAIN 3: Motor 2 — 右机翼电机
├── MAIN 4: Servo 0 — 左 Elevon     ← ⭐ trim 叠加在这里
├── MAIN 5: Servo 1 — 中央 Elevator
├── MAIN 6: Servo 2 — 右 Elevon     ← ⭐ trim 叠加在这里
├── MAIN 7: (空闲)
└── MAIN 8: (空闲)

AUX OUT 1-6 (FMU 直接 PWM, 6 通道):
├── AUX 1-6: 备用（可用于辅助功能）
└── 注意：如果 SYS_USE_IO=0 则 AUX 变成主输出

⚠️ 对于 chainwing 项目：
   SYS_USE_IO = 1（使用 PX4IO 输出 MAIN PWM）
   trim 叠加作用于 MAIN 4 和 MAIN 6（Servo 0 和 Servo 2）
```

### 3.7 Pixhawk 2.4.8 单机部署接线总览

```
                          ┌────────────────────────────────┐
                          │     Pixhawk 2.4.8 (主控)       │
                          │                                │
  QGC/数传 ←── TELEM1    │  TELEM1: /dev/ttyS1 (USART2)  │
                          │  TELEM2: /dev/ttyS2 (USART3)  │── TELEM2 → 从机(可选)
                          │  GPS:    /dev/ttyS3 (USART1)  │── GPS → GPS模块
                          │                                │
  左电机  ←── MAIN 1     │  MAIN OUT 1-8 (via PX4IO):    │
  中电机  ←── MAIN 2     │   1-3: 电机                    │
  右电机  ←── MAIN 3     │   4-6: 舵面                    │
  左Elevon←── MAIN 4     │   4: 左Elevon (Servo 0) ⭐trim │
  Elevator←── MAIN 5     │   5: Elevator (Servo 1)        │
  右Elevon←── MAIN 6     │   6: 右Elevon (Servo 2) ⭐trim │
                          │                                │
                          │  POWER: 电源模块               │
                          │  USB:   调试/烧录              │
                          └────────────────────────────────┘
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

### 9.4 三机分布式架构（基于 3 台 Pixhawk 2.4.8）

```
三台 Pixhawk 2.4.8 接线方案：

┌──────────────────────────────────────────────────────────────┐
│                     3 × Pixhawk 2.4.8 部署                   │
│                                                              │
│  ┌─────────────────┐                  ┌─────────────────┐   │
│  │ 左从机 Pixhawk   │                  │ 右从机 Pixhawk   │   │
│  │                  │                  │                  │   │
│  │ TELEM2 ─────────┤──── UART ────────├─── TELEM2       │   │
│  │                  │    TX↔RX交叉     │                  │   │
│  │ MAIN 4: 左Elevon│                  │ MAIN 4: 右Elevon│   │
│  │ MAIN 1: 左电机  │                  │ MAIN 1: 右电机  │   │
│  └──────┬───────────┘                  └──────┬───────────┘   │
│         │                                     │              │
│         │ TELEM2 (/dev/ttyS2)                 │              │
│         │ TX↔RX交叉                           │              │
│         │                                     │              │
│         ▼                                     ▼              │
│  ┌──────────────────────────────────────────────────────┐    │
│  │              主机 Pixhawk 2.4.8 (中央)                │    │
│  │                                                      │    │
│  │  TELEM2 (/dev/ttyS2) → 连接左从机                    │    │
│  │  SERIAL5 (/dev/ttyS6) → 连接右从机（备选：I2C转UART）│    │
│  │  TELEM1 (/dev/ttyS1) → QGC/数传                      │    │
│  │  MAIN 5: 中央 Elevator                                │    │
│  │  MAIN 2: 中央电机                                     │    │
│  └──────────────────────────────────────────────────────┘    │
└──────────────────────────────────────────────────────────────┘

主机 MAVLink 配置：
  mavlink start -x -d /dev/ttyS2 -b 921600 -m custom
  mavlink stream -d /dev/ttyS2 -s DEBUG_FLOAT_ARRAY -r 10
  # 如果连接右从机到 SERIAL5:
  mavlink start -x -d /dev/ttyS6 -b 921600 -m custom
  mavlink stream -d /dev/ttyS6 -s DEBUG_FLOAT_ARRAY -r 10

从机 MAVLink 配置（左/右相同）：
  mavlink start -d /dev/ttyS2 -b 921600 -m custom
  CW_SLV_COMM_EN = 1

⚠️ 注意事项：
  1. 必须使用 -m custom（不是 -m onboard）避免 ODOMETRY 刷屏
  2. SERIAL5 (/dev/ttyS6) 无硬件流控，长线缆建议降速到 115200
  3. 三台 Pixhawk 各自独立供电，仅连 TX/RX/GND
  4. TELEM2 的 Pin 1 (+5V) 不要互连！
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

---

## 附录 A：硬件/实机完整控制流程 {#a-硬件实机完整控制流程}

> 本附录详细描述链翼 UAV 在三种硬件部署架构下的完整控制流程，包括：
> 预期姿态的输入、各飞控负责的信息流、以及各飞控的电机/舵机输出流。
> 所有信息均基于已实现的代码，可直接对照源文件验证。

---

### A.1 三种部署架构总览 {#a1-三种部署架构总览}

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    链翼 UAV 三种部署架构对比                              │
├──────────────┬──────────────────┬──────────────────┬────────────────────┤
│              │ 架构一：单Pixhawk  │ 架构二：SIH      │ 架构三：三Pixhawk   │
│              │ 实机飞行          │ 硬件在环          │ 分布式实机          │
├──────────────┼──────────────────┼──────────────────┼────────────────────┤
│ 飞控数量      │ 1台              │ 1台              │ 3台                │
│ 机架文件      │ 2150_chainwing   │ 1103_chainwing   │ 2150_chainwing ×3  │
│              │                  │ _sih.hil         │                    │
│ 物理传感器    │ ✅ 真实IMU/GPS   │ ❌ SIH模拟       │ ✅ 各自独立IMU/GPS  │
│ PWM 输出     │ ✅ MAIN 1-6     │ ✅ HIL虚拟PWM    │ ✅ 各自 MAIN OUT    │
│ 铰链物理      │ ✅ 真实铰链      │ ❌ 无铰链模型     │ ✅ 真实铰链         │
│ 通信方式      │ uORB (进程内)    │ uORB (进程内)    │ UART MAVLink       │
│ CW_SLV_PWM_EN│ 1               │ 1               │ 1 (各从机)         │
│ CW_SLV_COMM_EN│ 0              │ 0               │ 1 (各从机)         │
│ 适用阶段      │ 首飞验证         │ 地面桌面测试      │ 量产/大规模部署     │
└──────────────┴──────────────────┴──────────────────┴────────────────────┘
```

---

### A.2 架构一：单 Pixhawk 实机控制流程 {#a2-架构一单-pixhawk-实机控制流程}

> **推荐首飞方案**：一台 Pixhawk 2.4.8 控制全部 3 个机翼单元。

#### A.2.1 系统总览框图

```
                        ┌──────────────────────────────────────────────────┐
                        │            Pixhawk 2.4.8 (FMU-V3)               │
                        │                                                  │
  ┌──────┐   GPS/MAG    │  ┌──────────┐    ┌───────────┐    ┌──────────┐  │   MAIN OUT
  │ GPS  │──────────────→│  │  EKF2    │───→│ Navigator │───→│ FW_Pos   │  │   ┌──────┐
  └──────┘              │  │(位置估计) │    │(任务规划) │    │(TECS/NPFG│  │   │MAIN 1│→ 电机(左)
                        │  └──────────┘    └───────────┘    │ 位置控制) │  │   │MAIN 2│→ 电机(中)
  ┌──────┐   加速度/角速│  ┌──────────┐                     └─────┬─────┘  │   │MAIN 3│→ 电机(右)
  │ IMU  │──────────────→│  │  姿态    │         ┌──────────────────┘       │   │MAIN 4│→ 左Elevon
  │MPU6000│             │  │  估计器   │         ↓                         │   │MAIN 5│→ 中Elevator
  └──────┘              │  └──────────┘  ┌──────────────┐  ┌───────────┐  │   │MAIN 6│→ 右Elevon
                        │        │       │ FW_Att/Rate  │  │ Control   │  │   └──────┘
  ┌──────┐   空速       │        ↓       │ (姿态/角速率 │→│ Allocator │  │
  │空速管│──────────────→│  vehicle_     │  PID 控制)   │  │ (效率矩阵 │  │
  └──────┘              │  attitude     └──────────────┘  │  →舵面)    │  │
                        │        │                        └─────┬──────┘  │
  ┌──────┐   RC信号     │        │                              ↓         │
  │ 遥控 │──────────────→│        │        ┌─────────────────────────────┐ │
  └──────┘              │        │        │   actuator_servos (uORB)    │ │
                        │        ↓        │   control[0]=左Elevon       │ │
                        │  ┌──────────┐   │   control[1]=中Elevator     │ │
                        │  │ChainWing │   │   control[2]=右Elevon       │ │
                        │  │ Slave    │   └──────────┬──────────────────┘ │
                        │  │(铰链估计 │              ↓                    │
                        │  │ +PD trim)│→ 叠加trim → actuator_servos(修正) │
                        │  └──────────┘       ↓                          │
                        │                ┌──────────┐  ┌──────────────┐  │
                        │                │FuncServos│→│    PWMOut     │  │
                        │                │(读取servo)│  │(输出PWM信号) │  │
                        │                └──────────┘  └──────────────┘  │
                        └──────────────────────────────────────────────────┘
```

#### A.2.2 完整信息流追踪（从遥控输入到舵面输出）

**第 1 层：姿态设定值输入**

```
遥控器输入 (PPM/SBUS)
    ↓
manual_control_setpoint (uORB, ~50Hz)
    ├── roll_stick     → FW_R_TC → att_sp.roll_body    (目标滚转角, rad)
    ├── pitch_stick    → FW_P_TC → att_sp.pitch_body   (目标俯仰角, rad)
    ├── yaw_stick      →           yaw_rate_cmd        (偏航角速率, rad/s)
    └── throttle_stick →           att_sp.thrust_body   (油门 [0,1])

自主飞行模式：
Navigator → position_setpoint_triplet (航点目标)
    ↓
FW_Pos_Control (NPFG 横向 + TECS 纵向)
    ↓
vehicle_attitude_setpoint (uORB, ~50Hz)
    ├── roll_body      (目标滚转角, 由NPFG导航律计算)
    ├── pitch_body     (目标俯仰角, 由TECS能量管理计算)
    ├── yaw_body       (目标航向角)
    └── thrust_body[0] (纵向推力, 由TECS计算)
```

**第 2 层：姿态与角速率控制**

```
vehicle_attitude_setpoint
    ↓
FW_Att_Control (50Hz)
    │   误差 = 设定值 - 当前姿态
    │   角速率设定值 = 误差 × (1/时间常数)
    │
    │   Roll:  ω_roll_sp  = (φ_sp - φ) / FW_R_TC     (FW_R_TC=0.5s)
    │   Pitch: ω_pitch_sp = (θ_sp - θ) / FW_P_TC     (FW_P_TC=0.5s)
    ↓
vehicle_rates_setpoint (uORB)
    ↓
FW_Rate_Control (250Hz)
    │
    │   横滚力矩: τ_roll  = FW_RR_P × e_p + FW_RR_I × ∫e_p + FW_RR_FF × ω_roll_sp
    │   俯仰力矩: τ_pitch = FW_PR_P × e_q + FW_PR_I × ∫e_q + FW_PR_FF × ω_pitch_sp
    │   偏航力矩: τ_yaw   = FW_YR_P × e_r + FW_YR_I × ∫e_r + FW_YR_FF × ω_yaw_sp
    │
    │   空速缩放: τ_scaled = τ × (FW_AIRSPD_TRIM / airspeed_true)²
    ↓
vehicle_torque_setpoint (uORB)
    ├── xyz[0] = τ_roll   (归一化横滚力矩)
    ├── xyz[1] = τ_pitch  (归一化俯仰力矩)
    └── xyz[2] = τ_yaw    (归一化偏航力矩)
```

**第 3 层：控制分配（力矩 → 舵面偏转）**

```
vehicle_torque_setpoint + vehicle_thrust_setpoint
    ↓
ControlAllocator (250Hz)
    │
    │   效率矩阵 B（从机架文件配置）：
    │   ┌──────────────┬──────────┬──────────┬──────────┐
    │   │              │ τ_roll   │ τ_pitch  │ τ_yaw    │
    │   ├──────────────┼──────────┼──────────┼──────────┤
    │   │ Servo 0 (左) │ +0.5     │ +0.5     │  0.0     │
    │   │ Servo 1 (中) │  0.0     │ +1.0     │  0.0     │
    │   │ Servo 2 (右) │ -0.5     │ +0.5     │  0.0     │
    │   └──────────────┴──────────┴──────────┴──────────┘
    │
    │   伪逆求解:
    │   Servo_0 = +0.5 × τ_roll + 0.5 × τ_pitch   (左 Elevon)
    │   Servo_1 =                  1.0 × τ_pitch   (中 Elevator)
    │   Servo_2 = -0.5 × τ_roll + 0.5 × τ_pitch   (右 Elevon)
    ↓
actuator_servos (uORB, 250Hz)
    ├── control[0] = Servo_0  (左 Elevon, 归一化 [-1, +1])
    ├── control[1] = Servo_1  (中 Elevator, 归一化 [-1, +1])
    └── control[2] = Servo_2  (右 Elevon, 归一化 [-1, +1])
```

**第 4 层：铰链修正 — PWM Trim 叠加（方案 C）**

```
IMU angular_velocity (xyz[0] = roll_rate)
    ↓
ChainwingSlave::updateHingeEstimate() (50Hz)
    │
    │   铰链角估计（互补滤波器）：
    │   1. 低通滤波: rate_filtered = (1-α)×rate_old + α×roll_rate
    │   2. 积分+衰减: angle = e^(-dt/τ) × (angle + rate×dt)     τ=2s
    │   3. 姿态修正:  angle = 0.98×angle + 0.02×(roll - roll_ref)
    │
    │   PD 控制律：
    │   trim = Kp × θ_hinge + Kd × θ̇_hinge
    │   trim = clamp(trim, -TRIM_MAX, +TRIM_MAX)
    │
    │   Kp = 1.5, Kd = 0.2, TRIM_MAX = 0.3
    ↓
ChainwingSlave::Run() — PWM Trim 叠加 (CW_SLV_PWM_EN=1)
    │
    │   读取 actuator_servos（来自 ControlAllocator）
    │   control[0] += trim_left    →  clamp to [-1, +1]
    │   control[2] += trim_right   →  clamp to [-1, +1]
    │   重新发布 actuator_servos
    ↓
actuator_servos (uORB, 修正后)
    ├── control[0] = Servo_0 + trim_left   (左 Elevon + 铰链修正)
    ├── control[1] = Servo_1               (中 Elevator, 不变)
    └── control[2] = Servo_2 + trim_right  (右 Elevon + 铰链修正)
```

**第 5 层：PWM 输出（信号 → 物理舵面）**

```
actuator_servos (修正后)
    ↓
FunctionServos::update() (在 MixingOutput 中运行)
    │   读取 actuator_servos.control[] 值
    ↓
MixingOutput::limitAndUpdateOutputs()
    │   归一化 [-1,1] → PWM 微秒 [1000, 2000]
    │   output_us = 1500 + control × 500
    ↓
PWMOut::updateOutputs()
    │   up_pwm_servo_set(channel, output_us)
    ↓
PX4IO 协处理器 (STM32F100)
    │   MAIN OUT 1-6 PWM 信号 (50Hz, 1000-2000μs)
    ↓
┌──────────────────────────────────────────────────────────┐
│ 物理执行器                                                │
│   MAIN 1 → Motor 左  (ESC → 无刷电机)                    │
│   MAIN 2 → Motor 中  (ESC → 无刷电机)                    │
│   MAIN 3 → Motor 右  (ESC → 无刷电机)                    │
│   MAIN 4 → Servo 左  (右 Elevon, 铰链修正已叠加)          │
│   MAIN 5 → Servo 中  (中 Elevator, 纯姿态控制)            │
│   MAIN 6 → Servo 右  (左 Elevon, 铰链修正已叠加)          │
└──────────────────────────────────────────────────────────┘
```

#### A.2.3 完整数据流汇总表

| 阶段 | uORB 话题 | 发布者 | 订阅者 | 频率 | 源文件 |
|------|-----------|--------|--------|------|--------|
| 传感器 | vehicle_angular_velocity | EKF2/传感器 | ChainwingSlave, FW_Rate | 250Hz | — |
| 传感器 | vehicle_attitude | EKF2 | ChainwingSlave, FW_Att | 250Hz | — |
| 导航 | position_setpoint_triplet | Navigator | FW_Pos | ~5Hz | — |
| 位置 | vehicle_attitude_setpoint | FW_Pos/Manual | FW_Att | 50Hz | — |
| 角速率 | vehicle_rates_setpoint | FW_Att | FW_Rate | 50Hz | — |
| 力矩 | vehicle_torque_setpoint | FW_Rate | ControlAllocator | 250Hz | — |
| 舵面(原始) | actuator_servos | ControlAllocator | ChainwingSlave | 250Hz | ControlAllocator.cpp:693 |
| **舵面(修正)** | **actuator_servos** | **ChainwingSlave** | **FunctionServos** | **50Hz** | **ChainwingSlave.cpp:Run()** |
| 铰链状态 | chainwing_hinge_status | ChainwingSlave | Logger | 50Hz | ChainwingSlave.cpp:122 |
| PWM | — | PWMOut | 物理舵机 | 50Hz | PWMOut.cpp:128 |

---

### A.3 架构二：SIH 硬件在环控制流程 {#a3-架构二sih-硬件在环控制流程}

> SIH（Simulation In Hardware）模式在 Pixhawk 硬件上运行内置物理引擎。
> 真实传感器被 SIH 虚拟传感器替代，PWM 输出被 HIL 执行器替代。

#### A.3.1 SIH 控制流程框图

```
┌─────────────────────────────────────────────────────────────────────┐
│                    Pixhawk 2.4.8 — SIH 模式                        │
│                    (SYS_HITL = 2)                                   │
│                                                                     │
│  ┌────────────────────────────────────────────────────────────────┐ │
│  │  SIH 物理引擎 (250Hz)                                         │ │
│  │  ┌──────────────────────┐                                     │ │
│  │  │ 单刚体动力学模型       │  虚拟传感器输出：                    │ │
│  │  │ M=1.0kg              │  → vehicle_angular_velocity (虚拟)   │ │
│  │  │ Ixx=1.02, Iyy=0.164 │  → vehicle_attitude (虚拟)           │ │
│  │  │ Izz=1.17 kg·m²      │  → vehicle_local_position (虚拟)     │ │
│  │  │ T_max=15N (3×5N)     │  → sensor_accel/gyro (虚拟)         │ │
│  │  └──────────────────────┘                                     │ │
│  │         ↑ 读取 actuator_servos + actuator_motors               │ │
│  └────────────────────────────────────────────────────────────────┘ │
│                                                                     │
│  ┌──────────┐     ┌──────────┐     ┌──────────┐     ┌──────────┐  │
│  │  EKF2    │────→│ FW_Pos   │────→│ FW_Att   │────→│ FW_Rate  │  │
│  │(读SIH虚拟│     │(位置控制)│     │(姿态控制)│     │(角速率)  │  │
│  │ 传感器)  │     └──────────┘     └──────────┘     └─────┬─────┘  │
│  └──────────┘                                             ↓        │
│                    ┌──────────────────────────────────────────────┐ │
│                    │  ControlAllocator → actuator_servos          │ │
│                    │                          ↓                   │ │
│                    │  ChainwingSlave (CW_SLV_PWM_EN=1)           │ │
│                    │       ↓ 叠加 trim                            │ │
│                    │  actuator_servos (修正) → SIH 物理引擎       │ │
│                    └──────────────────────────────────────────────┘ │
│                                                                     │
│  ⚠️ SIH 限制：                                                      │
│  - 单刚体模型，无铰链物理 → 铰链角始终≈0                              │
│  - trim 值≈0（无实际修正效果）                                        │
│  - 价值：验证模块启动、参数加载、PWM通路、日志记录                       │
└─────────────────────────────────────────────────────────────────────┘
```

#### A.3.2 SIH 模式信息流

```
SIH 虚拟传感器
    ↓
EKF2 (读取虚拟IMU/气压/磁力计/GPS)
    ↓
Navigator → FW_Pos → FW_Att → FW_Rate
    ↓
ControlAllocator
    ↓
actuator_servos (原始)
    ↓
ChainwingSlave (铰链角≈0 → trim≈0, 但通路完整)
    ↓
actuator_servos (修正后 ≈ 原始)
    ↓
SIH 物理引擎 (读取 actuator 值, 计算下一步状态)
    ↓
更新虚拟传感器 → 闭环
```

#### A.3.3 HIL 执行器映射

```
机架文件 1103_chainwing_sih.hil 定义：

HIL_ACT_FUNC1 = 201  →  Servo 0 (左 Elevon)
HIL_ACT_FUNC2 = 202  →  Servo 1 (中 Elevator)
HIL_ACT_FUNC3 = 203  →  Servo 2 (右 Elevon)
HIL_ACT_FUNC4 = 101  →  Motor 0 (左 电机)
HIL_ACT_FUNC5 = 102  →  Motor 1 (中 电机)
HIL_ACT_FUNC6 = 103  →  Motor 2 (右 电机)
```

---

### A.4 架构三：三 Pixhawk 分布式控制流程 {#a4-架构三三-pixhawk-分布式控制流程}

> 每个机翼单元有独立 Pixhawk。主机（中央）负责导航和姿态控制，
> 从机（左/右）负责执行主机命令并叠加铰链修正。

#### A.4.1 三机系统总览

```
┌────────────────────────────────────────────────────────────────────────────────┐
│                          三 Pixhawk 分布式架构                                  │
│                                                                                │
│     ┌──────────────────┐     UART/MAVLink      ┌──────────────────┐           │
│     │  左从机 Pixhawk   │◄═══════════════════►│  主机 Pixhawk     │           │
│     │  (TELEM2 连接)    │   DEBUG_FLOAT_ARRAY  │  (中央单元)       │           │
│     │                  │   id=42: 铰链状态 →   │                  │           │
│     │  CW_SLV_EN=1     │   id=43: ← 主机命令   │  CW_SLV_EN=0    │           │
│     │  CW_SLV_PWM_EN=1 │                      │  (不运行从机控制) │           │
│     │  CW_SLV_COMM_EN=1│                      │                  │           │
│     ├──────────────────┤                      ├──────────────────┤           │
│     │ MAIN OUT:        │                      │ MAIN OUT:        │           │
│     │  1: Motor 左     │                      │  1: Motor 中     │           │
│     │  4: Servo 左     │                      │  5: Servo 中     │           │
│     │  (含 trim 修正)  │                      │  (纯姿态控制)    │           │
│     └──────────────────┘                      └────────┬─────────┘           │
│                                                        │                     │
│                                          UART/MAVLink  │  (SERIAL5 连接)     │
│                                                        ↓                     │
│                                               ┌──────────────────┐           │
│                                               │  右从机 Pixhawk   │           │
│                                               │  (SERIAL5 连接)   │           │
│                                               │                  │           │
│                                               │  CW_SLV_EN=1     │           │
│                                               │  CW_SLV_PWM_EN=1 │           │
│                                               │  CW_SLV_COMM_EN=1│           │
│                                               ├──────────────────┤           │
│                                               │ MAIN OUT:        │           │
│                                               │  1: Motor 右     │           │
│                                               │  4: Servo 右     │           │
│                                               │  (含 trim 修正)  │           │
│                                               └──────────────────┘           │
└────────────────────────────────────────────────────────────────────────────────┘
```

#### A.4.2 主机信息流（中央 Pixhawk）

```
传感器 → EKF2 → Navigator → FW_Pos → FW_Att → FW_Rate
    ↓
ControlAllocator
    ↓
actuator_servos.control[1] → MAIN 5 (中央 Elevator)
actuator_motors.control[1] → MAIN 2 (中央 Motor)

同时：主机 MAVLink 实例发送命令给从机
    ↓
mavlink start -d /dev/ttyS2 -b 921600 -r 4000 -m custom  # → 左从机
mavlink start -d /dev/ttyS6 -b 921600 -r 4000 -m custom  # → 右从机
mavlink stream -d /dev/ttyS2 -s DEBUG_FLOAT_ARRAY -r 10
mavlink stream -d /dev/ttyS6 -s DEBUG_FLOAT_ARRAY -r 10

发送内容 (DEBUG_FLOAT_ARRAY, id=43, "CW_CMD"):
    data[0] = vehicle_torque_setpoint.xyz[1]  (俯仰力矩, 归一化)
    data[1] = vehicle_thrust_setpoint.xyz[0]  (油门, 归一化)
    data[2] = vehicle_torque_setpoint.xyz[0]  (横滚力矩, 归一化)
```

#### A.4.3 从机信息流（左/右 Pixhawk）

```
从机自身传感器 → EKF2 → 本地姿态估计
    ↓
ChainwingSlave (50Hz)
    │
    ├─ updateHingeEstimate(): 用本地 IMU 估计铰链角
    │   铰链角 = 从机滚转 - 主机滚转 (如果主机发送了滚转命令)
    │
    ├─ computeTrim(): PD 控制律
    │   trim = 1.5 × θ_hinge + 0.2 × θ̇_hinge
    │
    ├─ processMasterCommands(): 接收主机命令 (via MAVLink)
    │   读取 DEBUG_FLOAT_ARRAY (id=43, "CW_CMD")
    │   → _master_pitch_cmd, _master_throttle, _master_roll_cmd
    │
    ├─ publishDebugArray(): 发送铰链状态给主机 (via MAVLink)
    │   DEBUG_FLOAT_ARRAY (id=42, "CW_HINGE")
    │   → 铰链角、铰链率、trim 值、有效标志
    │
    └─ PWM Trim 叠加 (CW_SLV_PWM_EN=1):
        读取本地 ControlAllocator 的 actuator_servos
        control[0] += trim_left  (或 control[2] += trim_right)
        → MAIN OUT: Servo + Motor
```

#### A.4.4 三机通信协议时序

```
时间 →
主机:  ──CMD──CMD──CMD──CMD──CMD──CMD──  (10Hz, DEBUG_FLOAT_ARRAY id=43)
        ↓      ↓      ↓      ↓
左从机: ──────STS──────STS──────STS───  (10Hz, DEBUG_FLOAT_ARRAY id=42)
              ↑      ↑      ↑
右从机: ─STS──────STS──────STS────────  (10Hz, DEBUG_FLOAT_ARRAY id=42)

CMD: 主机 → 从机 (pitch, throttle, roll 命令)
STS: 从机 → 主机 (铰链角, 铰链率, trim 值)

超时安全: 从机 500ms 未收到 CMD → _master_cmd_valid = false
         (ChainwingSlave.cpp:263, MASTER_CMD_TIMEOUT_US = 500000)
```

---

### A.5 各飞控的电机/舵机输出分配 {#a5-各飞控的电机舵机输出分配}

#### A.5.1 单 Pixhawk 方案 — PWM 通道分配

```
Pixhawk 2.4.8 MAIN OUT (经 PX4IO 协处理器):

┌─────────┬───────────────┬──────────────────────────────────────┐
│ 通道    │ 功能           │ 说明                                 │
├─────────┼───────────────┼──────────────────────────────────────┤
│ MAIN 1  │ Motor 0 (左)  │ ESC → 左翼无刷电机, 推力                │
│ MAIN 2  │ Motor 1 (中)  │ ESC → 中翼无刷电机, 推力 + 差速偏航     │
│ MAIN 3  │ Motor 2 (右)  │ ESC → 右翼无刷电机, 推力                │
│ MAIN 4  │ Servo 0 (左)  │ 右 Elevon → 左翼舵面, **含 trim 修正** │
│ MAIN 5  │ Servo 1 (中)  │ Elevator → 中翼升降舵, 纯俯仰控制      │
│ MAIN 6  │ Servo 2 (右)  │ 左 Elevon → 右翼舵面, **含 trim 修正** │
│ MAIN 7  │ — (空闲)      │ 可备用                                │
│ MAIN 8  │ — (空闲)      │ 可备用                                │
└─────────┴───────────────┴──────────────────────────────────────┘

PWM 值域: 1000μs (最小) ↔ 1500μs (中位) ↔ 2000μs (最大)
归一化:    -1.0          ↔  0.0          ↔ +1.0
```

#### A.5.2 三 Pixhawk 分布式方案 — 各飞控输出

```
┌─────────────────────────────────────────────────────────────────┐
│                  主机 Pixhawk (中央单元)                          │
│                                                                 │
│  MAIN 1: — (空闲, 左电机由左从机控制)                              │
│  MAIN 2: Motor 1 (中翼电机)                                     │
│  MAIN 3: — (空闲, 右电机由右从机控制)                              │
│  MAIN 4: — (空闲)                                               │
│  MAIN 5: Servo 1 (中翼升降舵, 纯姿态控制)                         │
│  MAIN 6: — (空闲)                                               │
│                                                                 │
│  TELEM2 → 左从机 UART (MAVLink custom mode)                     │
│  SERIAL5 → 右从机 UART (MAVLink custom mode)                    │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│                  左从机 Pixhawk (左翼单元)                        │
│                                                                 │
│  MAIN 1: Motor 0 (左翼电机, 油门来自主机命令)                      │
│  MAIN 4: Servo 0 (左翼 Elevon, 主机pitch + 铰链trim)             │
│                                                                 │
│  TELEM2 → 主机 UART                                             │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│                  右从机 Pixhawk (右翼单元)                        │
│                                                                 │
│  MAIN 1: Motor 2 (右翼电机, 油门来自主机命令)                      │
│  MAIN 4: Servo 2 (右翼 Elevon, 主机pitch + 铰链trim)             │
│                                                                 │
│  TELEM2 → 主机 UART                                             │
└─────────────────────────────────────────────────────────────────┘
```

---

### A.6 姿态设定值输入方式 {#a6-姿态设定值输入方式}

#### A.6.1 手动飞行（Stabilized 模式）

```
遥控器摇杆 → RC接收机 → Pixhawk RC IN
    ↓
RC_Update 模块
    ↓
manual_control_setpoint (uORB)
    │
    ├── roll:     左右摇杆 → 目标滚转角 (±FW_R_LIM, 默认±50°)
    ├── pitch:    前后摇杆 → 目标俯仰角 (FW_P_LIM_MIN ~ FW_P_LIM_MAX)
    ├── yaw:      偏航摇杆 → 偏航角速率 (±FW_Y_RMAX, 默认±15°/s)
    └── throttle: 油门摇杆 → 推力 (0 ~ FW_THR_MAX)
    ↓
flight_mode_manager → 直接转换为 vehicle_attitude_setpoint
```

#### A.6.2 自主飞行（Mission 模式）

```
QGroundControl 上传航点任务
    ↓
DataMan (存储航点到 SD 卡)
    ↓
Navigator (5Hz)
    │   计算目标航点、切换逻辑
    ↓
position_setpoint_triplet (当前 + 前一个 + 下一个航点)
    ↓
FW_Pos_Control (50Hz)
    │
    │   横向 (NPFG 导航律):
    │   roll_sp = NPFG(当前位置, 目标航线)  →  限幅 ±FW_R_LIM
    │
    │   纵向 (TECS 能量管理):
    │   pitch_sp = TECS(高度误差, 速度误差)
    │   throttle = TECS(能量误差)
    │
    ↓
vehicle_attitude_setpoint (uORB)
    ├── roll_body   (NPFG 计算的目标滚转)
    ├── pitch_body  (TECS 计算的目标俯仰)
    └── thrust_body (TECS 计算的推力)
```

#### A.6.3 Return-to-Launch / Loiter 模式

```
Commander 切换到 RTL/Loiter
    ↓
Navigator 计算返航/盘旋航点
    ↓
position_setpoint_triplet (type = LOITER)
    ↓
FW_Pos_Control
    │   横向: NPFG 计算盘旋滚转角 (根据 NAV_LOITER_RAD)
    │   纵向: TECS 维持目标高度
    ↓
vehicle_attitude_setpoint → 同上控制链路
```

---

### A.7 PWM Trim 叠加实现细节（方案 C） {#a7-pwm-trim-叠加实现细节}

#### A.7.1 实现代码（ChainwingSlave.cpp，已提交）

```cpp
// 在 ChainwingSlave::Run() 中, publish hinge status 之后:

// Hardware PWM trim overlay (Scheme C):
// Read actuator_servos from control_allocator, add trim to elevon channels,
// and re-publish so that PWMOut receives the trimmed values.
// This replaces GZMixingInterfaceServo for real hardware.
if (_param_pwm_enable.get() != 0 && _ref_initialized) {
    actuator_servos_s servos{};

    if (_actuator_servos_sub.copy(&servos)) {
        // Apply hinge trim correction to left and right elevon channels
        servos.control[0] = math::constrain(servos.control[0] + trim_left, -1.0f, 1.0f);
        servos.control[2] = math::constrain(servos.control[2] + trim_right, -1.0f, 1.0f);

        servos.timestamp = hrt_absolute_time();
        _actuator_servos_pub.publish(servos);
    }
}
```

#### A.7.2 数据流管道对比：仿真 vs 硬件

```
                    仿真 (SITL Gazebo)                    硬件 (Pixhawk)
                    ──────────────────                    ───────────────

ControlAllocator    ControlAllocator
    ↓                       ↓
actuator_servos     actuator_servos
    ↓                       ↓
FunctionServos      ChainwingSlave ← CW_SLV_PWM_EN=1
    ↓                  ↓ 叠加 trim
GZMixingInterface   actuator_servos (修正后)
    ↓ 叠加 trim             ↓
Gazebo servo topic  FunctionServos
    ↓                       ↓
Gazebo 物理引擎     PWMOut → up_pwm_servo_set()
                            ↓
                    PX4IO → MAIN OUT 1-6
                            ↓
                    物理舵机/电机
```

#### A.7.3 为什么选择方案 C

| 方案 | 描述 | 优点 | 缺点 | 选择 |
|------|------|------|------|------|
| A | 修改 FunctionServos | 与 CA 同步 | 修改 PX4 核心代码 | ❌ |
| B | 修改 PWMOut | 最底层 | 修改 PX4 核心代码 | ❌ |
| **C** | **ChainwingSlave 修改 actuator_servos** | **封装在自定义模块** | 50Hz vs CA 250Hz | **✅** |
| D | 新建中间 topic | 无冲突 | 需修改 FunctionServos | ❌ |

**方案 C 的频率差异说明：**
- ControlAllocator 以 ~250Hz 发布 actuator_servos
- ChainwingSlave 以 50Hz 重新发布（含 trim）
- 物理舵机 PWM 频率 = 50Hz（标准模拟舵机）
- 因此 50Hz trim 更新与舵机物理响应匹配
- trim 变化缓慢（铰链动力学 ~3Hz），50Hz 远超 Nyquist 要求

---

### A.8 控制频率与时序分析 {#a8-控制频率与时序分析}

```
模块执行频率:
┌──────────────────────┬──────────┬──────────────────────────────┐
│ 模块                 │ 频率     │ 触发方式                      │
├──────────────────────┼──────────┼──────────────────────────────┤
│ IMU 驱动             │ 1000 Hz  │ SPI 中断                      │
│ EKF2 (姿态估计)     │  250 Hz  │ IMU 数据回调                   │
│ FW_Rate_Control      │  250 Hz  │ angular_velocity 回调          │
│ ControlAllocator     │  250 Hz  │ torque_setpoint 回调           │
│ FW_Att_Control       │   50 Hz  │ attitude 回调                  │
│ FW_Pos_Control       │   50 Hz  │ attitude 回调                  │
│ **ChainwingSlave**   │ **50 Hz**│ **ScheduleOnInterval(20ms)**  │
│ MixingOutput/PWMOut  │  250 Hz  │ actuator_servos 回调           │
│ PX4IO PWM 输出       │   50 Hz  │ 硬件 PWM 定时器 (20ms)        │
│ Navigator            │    5 Hz  │ 定时器                        │
└──────────────────────┴──────────┴──────────────────────────────┘

时序: 一个完整控制周期 (20ms = 一个 PWM 周期)
┌─────────────────────────────────────────────────────────────────┐
│ 0ms                          10ms                         20ms │
│  │                            │                            │   │
│  ├─ CA 发布 actuator_servos   │                            │   │
│  │  (raw, 无 trim)            │                            │   │
│  ├─ 4ms: CA 再次发布          │                            │   │
│  ├─ 8ms: CA 再次发布          │                            │   │
│  │                            │                            │   │
│  ├─ ~10ms: ChainwingSlave Run │                            │   │
│  │  → 读 IMU → 计算 trim      │                            │   │
│  │  → 读 actuator_servos      │                            │   │
│  │  → 叠加 trim → 发布        │                            │   │
│  │                            │                            │   │
│  ├─ 12ms: CA 再次发布 (raw)   │                            │   │
│  ├─ 16ms: CA 再次发布 (raw)   │                            │   │
│  │                            │                            │   │
│  ├─ PWM 周期边界 → 最后写入值生效                           │   │
│  │  (取决于 MixingOutput 处理顺序)                          │   │
│  └──────────────────────────────────────────────────────────│   │
│                                                             │   │
│  结论: 50Hz 舵机取 20ms 内最后一次 up_pwm_servo_set() 的值   │   │
│        trim 修正在每个 PWM 周期内至少有 1 次被写入            │   │
└─────────────────────────────────────────────────────────────────┘
```

---

### A.9 安全机制 {#a9-安全机制}

#### A.9.1 参数安全

```
CW_SLV_EN = 0       → Run() 立即返回, 不修改任何舵面值
CW_SLV_PWM_EN = 0   → 不执行 actuator_servos 修改
CW_SLV_TRIM_MAX = 0.3  → trim 限幅在 ±30% 舵面行程内
                        (剩余 70% 给正常姿态控制)
```

#### A.9.2 数据有效性检查

```
ChainwingSlave::Run():
    1. _param_enable.get() == 0 → return (模块禁用)
    2. _param_pwm_enable.get() == 0 → 跳过 PWM 叠加
    3. _ref_initialized == false → 跳过 PWM 叠加 (参考未初始化)
    4. _actuator_servos_sub.copy() 失败 → 跳过 (无数据)
    5. math::constrain() → 确保输出 [-1, +1]

GZMixingInterfaceServo (仿真):
    1. _hinge_status_sub.copy() 失败 → hinge_valid = false → 跳过
    2. hinge_status.data_valid == false → 跳过
    3. output 限幅 [-1, +1]
```

#### A.9.3 MAVLink 通信超时 (三机方案)

```
从机 processMasterCommands():
    if (hrt_elapsed_time(&_last_master_cmd) > 500ms):
        _master_cmd_valid = false
        PX4_WARN("Master command timeout")

安全策略: 超时后从机继续以最后收到的命令运行
         (不是紧急停止, 避免突然失控)
```

#### A.9.4 首飞安全检查清单

```
□ CW_SLV_EN=0 时舵面行为正常（纯姿态控制）
□ CW_SLV_EN=1, CW_SLV_PWM_EN=1 时舵面有 trim 偏移
□ 手动模式下 trim 叠加不超过 ±30% 行程
□ trim 方向正确（左铰链上偏 → 左 elevon 下偏修正）
□ 遥控器可随时切换到 Manual 模式接管
□ 解锁前确认 chainwing_slave status 显示正常
□ 首飞使用保守参数：CW_SLV_KP=0.5, CW_SLV_KD=0.1
```

---

## 14. 版本历史 {#15-版本历史}

| 版本 | 日期 | 变更内容 |
|------|------|---------|
| v1.0 | 2026-03-23 | 初版：工作量评估 + 4阶段实施路线 |
| v1.1 | 2026-03-23 | 基于 Pixhawk 2.4.8 实物引脚图更新：§3 接口布局、TELEM2 接线、PWM 通道映射、三机接线方案 |
| **v2.0** | **2026-03-24** | **代码实现完成！** (1) 新增 CW_SLV_PWM_EN 参数, (2) ChainwingSlave.cpp 方案C 实现, (3) fmu-v3 板级配置, (4) 2150_chainwing + 1103_sih 机架完善, (5) 新增附录A: 三种架构完整控制流程详解（含姿态输入、信息流、输出映射、时序分析） |

*文档结束 — 配合源代码文件阅读：ChainwingSlave.cpp, ChainwingSlave.hpp, chainwing_slave_params.c*
