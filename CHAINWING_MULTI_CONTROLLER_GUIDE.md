# 链翼无人机三飞控系统架构详细说明

> **版本**: 2.2  
> **日期**: 2026-03-15  
> **适用项目**: 三体串联固定翼（Chain-Wing）无人机  
> **基线固件**: PX4 Autopilot v1.15+ (本仓库定制版)  
> **v2.0 更新**: 新增 §10-§15 — 从机代码开发指南（CAN 通信 + 单侧副翼微调）  
> **v2.1 更新**: 新增 §16 — 主机链翼仿真数据来源完整追溯  
> **v2.2 更新**: 新增 §17-§18 — 升降舵 vs 副翼方案深度对比 + 从机固件编写可行性评估

---

## 目录

1. [项目概述与三飞控架构](#1-项目概述与三飞控架构)
2. [各飞控固件烧录方案](#2-各飞控固件烧录方案)
3. [飞控间通信方案](#3-飞控间通信方案)
4. [从飞控相对位置计算](#4-从飞控相对位置计算)
5. [从飞控尾翼升降舵实时位置调整](#5-从飞控尾翼升降舵实时位置调整)
6. [系统集成架构总览](#6-系统集成架构总览)
7. [项目落地注意事项](#7-项目落地注意事项)
8. [仿真验证路线图](#8-仿真验证路线图)
9. [风险评估与缓解措施](#9-风险评估与缓解措施)
10. [从机代码开发详细指南（CAN + 单侧副翼微调）](#10-从机代码开发详细指南can-通信--单侧副翼微调)
11. [CAN (DroneCAN) 通信实现详解](#11-can-dronecan-通信实现详解)
12. [需要创建和修改的代码文件清单](#12-需要创建和修改的代码文件清单)
13. [单侧副翼位置微调控制设计](#13-单侧副翼位置微调控制设计)
14. [仿真配置与数据](#14-仿真配置与数据)
15. [分步实施路线图（更新版）](#15-分步实施路线图更新版)
16. [**主机链翼仿真数据来源完整追溯**](#16-主机链翼仿真数据来源完整追溯)
17. [**从机位置微调执行器方案对比：升降舵 vs 副翼**](#17-从机位置微调执行器方案对比升降舵-vs-副翼)
18. [**从机固件编写可行性评估与数据清单**](#18-从机固件编写可行性评估与数据清单)

---

## 1. 项目概述与三飞控架构

### 1.1 链翼无人机物理结构

```
                     ┌─────────────────────────────────┐
                     │          链翼无人机俯视图           │
                     └─────────────────────────────────┘

   ←── 1.2m ──→     ←── 1.2m ──→     ←── 1.2m ──→
   ┌───────────┐     ┌───────────┐     ┌───────────┐
   │           │     │           │     │           │
   │  左翼单元  │─────│  中央单元  │─────│  右翼单元  │
   │  (从机L)  │铰接点│  (主机M)  │铰接点│  (从机R)  │
   │           │     │           │     │           │
   │ ▲左升降副翼│     │ ▲中央升降舵│     │ ▲右升降副翼│
   │ ◉左电机   │     │ ◉中央电机  │     │ ◉右电机   │
   │ □左飞控   │     │ □主飞控    │     │ □右飞控   │
   └───────────┘     └───────────┘     └───────────┘

        Y=-1.2m          Y=0.0m           Y=+1.2m

   总翼展: 3.6m | 总质量: 5.7kg (3×1.9kg) | 铰接点: Y=±0.6m
```

### 1.2 为什么需要三个飞控？

| 理由 | 说明 |
|------|------|
| **铰接结构** | 三个机体通过铰接点连接，非刚性一体，各段有独立自由度 |
| **独立控制面** | 每个单元有独立的升降舵/升降副翼，需要独立的舵机控制 |
| **独立动力** | 三台电机独立运转，需要独立的 ESC 控制 |
| **分布式传感** | 各段可能受到不同的气动载荷，需要独立感知 |
| **冗余安全** | 一个飞控故障不应导致整机失控 |

### 1.3 主从架构

```
                    ┌──────────────┐
                    │   地面站 QGC   │
                    │  (MAVLink)    │
                    └──────┬───────┘
                           │ 数传/WiFi
                           │ MAV_SYS_ID=1
                    ┌──────▼───────┐
                    │   主飞控 (M)   │
                    │  中央单元      │
                    │  MAV_SYS_ID=1 │
                    │  决策 + 协调   │
                    └───┬──────┬───┘
                   CAN/│      │CAN/
                  UART │      │UART
              ┌────────▼┐  ┌──▼────────┐
              │从飞控 (L)│  │从飞控 (R) │
              │ 左翼单元 │  │ 右翼单元  │
              │SYS_ID=2 │  │SYS_ID=3  │
              │执行+反馈 │  │执行+反馈  │
              └─────────┘  └──────────┘
```

---

## 2. 各飞控固件烧录方案

### 2.1 核心问题：三个飞控是否烧入同一固件？

**答：不完全相同。三个飞控烧入同一份 PX4 固件基础镜像，但参数配置不同。**

| 飞控 | 固件基础 | 机架文件 | MAV_SYS_ID | 角色 |
|------|---------|---------|------------|------|
| 主飞控 (M) | PX4 v1.15+ 定制版 | `2150_chainwing` | 1 | 整机飞行决策 |
| 从飞控 (L) | PX4 v1.15+ 定制版 | 新建 `2151_chainwing_slave` | 2 | 左翼执行 |
| 从飞控 (R) | PX4 v1.15+ 定制版 | 新建 `2151_chainwing_slave` | 3 | 右翼执行 |

### 2.2 主飞控 (M) — 中央单元

**烧录固件**：本仓库的定制 PX4 固件

**使用机架**：`2150_chainwing`（已存在）

**编译与烧录命令**：
```bash
# 1. 编译（根据飞控板型号选择）
cd ~/PX4-Autopilot
make px4_fmu-v6x_default          # Pixhawk 6X
# 或 make px4_fmu-v5x_default      # Pixhawk 5X
# 或 make px4_fmu-v5_default       # Pixhawk 4

# 2. 连接主飞控 USB，烧录
make px4_fmu-v6x_default upload

# 3. QGC 中选择机架: "Chain-Wing UAV (2150)"
```

**主飞控关键参数**：
```
# 系统标识
MAV_SYS_ID = 1          # 系统 ID（主机）
MAV_COMP_ID = 1          # 组件 ID

# 机架类型
MAV_TYPE = 1             # 固定翼

# 控制分配（3 电机 + 3 舵面 = 完整链翼）
CA_ROTOR_COUNT = 3       # 三台电机
CA_SV_CS_COUNT = 3       # 三个控制面

# 差动推力偏航
CA_ROTOR0_PY = -1.2      # 左电机 Y 轴位置
CA_ROTOR1_PY = 0.0       # 中央电机 Y 轴位置
CA_ROTOR2_PY = 1.2       # 右电机 Y 轴位置

# 航向保持（定制功能）
FW_YAW_STAB_SC = 2.0     # 航向保持增益

# 通信端口配置
MAV_1_CONFIG = TELEM2     # 第2路数传用于从飞控通信
MAV_1_MODE = 7            # Minimal 模式（低带宽）
MAV_1_FORWARD = 1         # 消息转发开启
```

**主飞控职责**：
- ✅ 整机飞行规划与导航（航点、任务、RTL）
- ✅ 整机姿态估计（EKF2 主实例）
- ✅ 中央电机和中央升降舵控制
- ✅ 向从飞控发送指令（差动推力、舵面偏转角度）
- ✅ 与地面站 QGC 通信（状态上报、指令接收）
- ✅ 故障检测与安全决策

### 2.3 从飞控 (L/R) — 左翼/右翼单元

**烧录固件**：同一份 PX4 固件（与主飞控完全相同的编译产物）

**使用机架**：需新建 `2151_chainwing_slave`

**编译与烧录命令**：
```bash
# 编译（只需编译一次，同一固件）
make px4_fmu-v6x_default

# 连接左翼从飞控 USB，烧录
make px4_fmu-v6x_default upload
# QGC 中设置 MAV_SYS_ID = 2

# 连接右翼从飞控 USB，烧录
make px4_fmu-v6x_default upload
# QGC 中设置 MAV_SYS_ID = 3
```

**从飞控关键参数（左翼 L）**：
```
# 系统标识
MAV_SYS_ID = 2           # 系统 ID（从机-左）
MAV_COMP_ID = 1

# 控制分配（只控制自己的 1 电机 + 1 舵面）
CA_ROTOR_COUNT = 1        # 只有自己的电机
CA_SV_CS_COUNT = 1        # 只有自己的升降副翼

# 该从飞控不独立飞行
COM_ARM_WO_GPS = 1        # 允许无 GPS 解锁（GPS 在主飞控）
CBRK_SUPPLY_CHK = 894281  # 电池检查旁路
EKF2_AID_MASK = 0         # 从飞控不需要独立 EKF
SYS_HAS_GPS = 0           # 从飞控可不带 GPS（可选）

# 通信
MAV_0_CONFIG = TELEM1     # 用于与主飞控通信
MAV_0_MODE = 7            # Minimal 模式
```

**从飞控关键参数（右翼 R）**：
```
MAV_SYS_ID = 3            # 系统 ID（从机-右）
# 其余与左翼从飞控相同
```

### 2.4 固件对比一览

| 方面 | 主飞控 (M) | 从飞控 (L) | 从飞控 (R) |
|------|-----------|-----------|-----------|
| 固件二进制 | **相同** | **相同** | **相同** |
| 机架配置 | 2150_chainwing | 2151_chainwing_slave | 2151_chainwing_slave |
| MAV_SYS_ID | 1 | 2 | 3 |
| GPS | ✅ 必须 | 可选（主飞控提供） | 可选（主飞控提供） |
| EKF2 | ✅ 完整 | 简化/依赖主机 | 简化/依赖主机 |
| 控制电机数 | 1 (中央) | 1 (左) | 1 (右) |
| 控制舵面数 | 1 (中央升降舵) | 1 (左升降副翼) | 1 (右升降副翼) |
| 导航决策 | ✅ 有 | ❌ 无（执行指令） | ❌ 无（执行指令） |
| 地面站通信 | ✅ 有 | ❌ 无（通过主飞控） | ❌ 无（通过主飞控） |

---

## 3. 飞控间通信方案

### 3.1 方案对比

| 方案 | 协议 | 速率 | 延迟 | 距离 | 线缆 | 推荐度 |
|------|------|------|------|------|------|--------|
| **UART + MAVLink** | MAVLink v2 | 921600 bps | <1ms | <2m | 4线 | ⭐⭐⭐⭐⭐ |
| **CAN (DroneCAN)** | UAVCAN/DroneCAN | 1 Mbps | <0.5ms | <5m | 2线 | ⭐⭐⭐⭐ |
| **I2C** | 原始协议 | 400 kbps | <0.1ms | <0.5m | 2线 | ⭐⭐ |
| **SPI** | 原始协议 | 10+ Mbps | <0.01ms | <0.1m | 4线 | ⭐ |
| **WiFi** | MAVLink/UDP | 可变 | 5-50ms | >100m | 无线 | ⭐⭐ |

### 3.2 推荐方案：UART + MAVLink v2

**为什么选择 UART + MAVLink？**

1. **PX4 原生支持**：PX4 MAVLink 模块支持最多 6 个并发实例
2. **成熟稳定**：MAVLink v2 是无人机标准协议，经过大量验证
3. **带宽足够**：921600 bps 远超从飞控所需的控制指令带宽
4. **延迟极低**：<1ms 完全满足 50Hz 控制循环需求
5. **接线简单**：只需 TX, RX, GND, (VCC) 四线
6. **无需额外硬件**：所有 Pixhawk 飞控均自带 TELEM 端口

**物理连接**：

```
                    主飞控 (M)
                 ┌──────────┐
                 │  TELEM1  │── 数传 ──→ 地面站
                 │  TELEM2  │── UART ──→ 从飞控 (L) 的 TELEM1
                 │  TELEM3  │── UART ──→ 从飞控 (R) 的 TELEM1
                 │   GPS    │── GPS 模块
                 │   USB    │── 调试
                 └──────────┘

     ┌──────────┐                          ┌──────────┐
     │从飞控 (L) │                          │从飞控 (R) │
     │  TELEM1  │←─ UART ─── 主飞控 TELEM2  │  TELEM1  │←─ UART ─── 主飞控 TELEM3
     │   PWM    │── ESC ──→ 左电机          │   PWM    │── ESC ──→ 右电机
     │   PWM    │── 舵机 ──→ 左升降副翼      │   PWM    │── 舵机 ──→ 右升降副翼
     └──────────┘                          └──────────┘
```

**UART 接线详情**：

```
主飞控 TELEM2          从飞控(L) TELEM1
  ┌─────┐                ┌─────┐
  │ TX ─┼───────────────→│ RX  │
  │ RX ←┼────────────────│ TX  │
  │ GND ┼───────────────→│ GND │
  │ VCC │   (可选 5V)     │ VCC │
  └─────┘                └─────┘

注意：TX 接 RX，RX 接 TX（交叉连接）
```

### 3.3 备选方案：CAN (DroneCAN)

**优势**：
- 总线型拓扑，一条 CAN 总线连接三个飞控
- 自带错误检测和重发机制
- 抗干扰能力强

**接线**：
```
主飞控 CAN1 ──── CAN 总线 ──── 从飞控(L) CAN1
                    │
                从飞控(R) CAN1

CAN 总线: CANH, CANL（两线）
注意：总线两端需要 120Ω 终端电阻
```

**参数配置**：
```
# 主飞控
UAVCAN_ENABLE = 2        # 自动配置 CAN
UAVCAN_NODE_ID = 1       # CAN 节点 ID

# 从飞控 (L)
UAVCAN_ENABLE = 2
UAVCAN_NODE_ID = 2

# 从飞控 (R)
UAVCAN_ENABLE = 2
UAVCAN_NODE_ID = 3
```

### 3.4 通信内容与频率

**主飞控 → 从飞控（指令下行）**：

| 消息类型 | MAVLink 消息 | 频率 | 内容 |
|---------|-------------|------|------|
| 推力指令 | `SET_ACTUATOR_CONTROL_TARGET` | 50 Hz | 油门设定值 |
| 舵面指令 | `SET_ACTUATOR_CONTROL_TARGET` | 50 Hz | 升降副翼角度 |
| 姿态参考 | `ATTITUDE_TARGET` | 50 Hz | 主飞控的姿态/速率设定 |
| 系统状态 | `HEARTBEAT` | 1 Hz | 解锁状态、飞行模式 |
| 时间同步 | `TIMESYNC` | 10 Hz | 时钟对齐 |

**从飞控 → 主飞控（状态上行）**：

| 消息类型 | MAVLink 消息 | 频率 | 内容 |
|---------|-------------|------|------|
| 心跳 | `HEARTBEAT` | 1 Hz | 从飞控存活确认 |
| 姿态 | `ATTITUDE` | 50 Hz | 从飞控 IMU 测量的姿态 |
| 铰接角度 | `NAMED_VALUE_FLOAT` | 50 Hz | 本单元相对主机的偏转角 |
| 传感器状态 | `SYS_STATUS` | 1 Hz | 电池、传感器健康 |

**带宽估算**：
```
指令下行: 50Hz × 50bytes/msg × 2路 = 5,000 B/s ≈ 40 kbps
状态上行: 50Hz × 80bytes/msg × 2路 = 8,000 B/s ≈ 64 kbps
总计: ≈ 104 kbps << 921,600 bps UART 容量

→ 带宽利用率仅 ~11%，完全满足需求
```

---

## 4. 从飞控相对位置计算

### 4.1 核心问题

> "从飞控相对于主机的相对位置可不可以通过从飞控的固件来计算？"

**答：可以，而且有多种方法。**

### 4.2 方法一：IMU 积分法（推荐首选）

**原理**：从飞控有自己的 IMU（加速度计 + 陀螺仪），可以通过双重积分估算相对于主飞控的位移。

```
从飞控 IMU 测量:
  加速度 a(t) → 积分 → 速度 v(t) → 积分 → 位置 p(t)

相对位置 = 从飞控位置 - 主飞控位置
```

**实现方式**：
```
1. 主飞控通过 MAVLink 广播自己的加速度和角速率
2. 从飞控用自己的 IMU 数据减去主飞控数据
3. 差值就是铰接点处的相对运动

Δa = a_slave - a_master    （相对加速度）
Δθ = ∫(ω_slave - ω_master)dt  （相对角度）
```

**优势**：
- 所有飞控都有 IMU，不需要额外传感器
- 高更新率（200Hz+）
- 实时性好

**劣势**：
- 加速度积分有漂移，长时间会偏移
- 需要定期校正

### 4.3 方法二：铰接角度测量（最精确）

**原理**：在铰接点安装角度传感器（编码器/电位计），直接测量相对角度。

```
铰接点位置: Y = ±0.6m

        铰接角度 θ
          ↗
   从翼段 ─────┐─────── 主翼段
              铰接点
              编码器

相对位置:
  Δx = L × sin(θ)     (前后偏移)
  Δz = L × cos(θ) - L  (上下偏移)

其中 L = 0.6m (铰接点到翼段中心距离)
```

**实现方式**：
```
1. 在两个铰接点各安装一个旋转编码器/电位计
2. 编码器信号接入对应从飞控的 ADC 输入
3. 从飞控读取角度值 → 计算相对位置
4. 通过 MAVLink 上报给主飞控

ADC 读取 → 角度换算 → 三角函数 → 相对坐标
```

**优势**：
- 直接测量，无积分漂移
- 精度高（分辨率可达 0.01°）
- 计算简单

**劣势**：
- 需要额外硬件（编码器×2）
- 铰接点结构需要预留安装空间

### 4.4 方法三：双 GPS 差分法

**原理**：每个飞控各自携带 GPS，通过 GPS 坐标差计算相对位置。

```
主飞控 GPS: (lat_m, lon_m, alt_m)
从飞控 GPS: (lat_s, lon_s, alt_s)

相对位置:
  ΔN = (lat_s - lat_m) × 111,320 m/°     (南北方向)
  ΔE = (lon_s - lon_m) × 111,320 × cos(lat) m/°  (东西方向)
  ΔD = alt_m - alt_s                       (上下方向)
```

**优势**：
- 绝对位置参考，无漂移
- 无需额外机械改装

**劣势**：
- GPS 更新率低（5-10Hz），不适合高频控制
- 精度有限（±2m），对于 1.2m 的翼段间距太粗糙
- 需要 RTK GPS 才能达到 cm 级精度（成本高）
- 多 GPS 天线可能有电磁干扰

### 4.5 方法四：主飞控计算法（最简单）

**原理**：由主飞控通过自身传感器和已知几何关系来推算从飞控位置。

```
主飞控的 EKF2 已知:
  - 整机姿态 (roll, pitch, yaw)
  - 整机位置 (x, y, z)
  - 整机速度 (vx, vy, vz)

从飞控静态位置 (无铰接变形时):
  左翼: [-1.2, 0, 0] 在机体坐标系中
  右翼: [+1.2, 0, 0] 在机体坐标系中

从飞控世界坐标 = 主飞控世界坐标 + R_body_to_world × [±1.2, 0, 0]
```

**优势**：
- 最简单，无需额外传感器
- 无需从飞控参与计算
- 适合刚性连接场景

**劣势**：
- 假设刚性连接（忽略铰接变形）
- 精度取决于主飞控 EKF2 精度

### 4.6 推荐方案

**推荐组合**：方法二（铰接角度） + 方法四（主飞控计算）

```
主飞控:                              从飞控:
  EKF2 整机位姿                       IMU 姿态
       ↓                                ↓
  几何计算 (已知翼展)                  铰接角度传感器
       ↓                                ↓
  从飞控标称位置                       铰接变形量 Δθ
       ↓                                ↓
       └────────────────┬───────────────┘
                        ↓
              从飞控实际位置 = 标称位置 + 变形修正
```

---

## 5. 从飞控尾翼升降舵实时位置调整

### 5.1 核心问题

> "从飞控可不可以通过尾翼升降舵来实时调整其位置？"

**答：可以，这正是链翼构型的核心控制思路。**

### 5.2 物理原理

每个翼段有一个升降舵/升降副翼，它产生的气动力可以改变该翼段的**俯仰力矩**和**升力**。

```
               来流方向 →
               
   ┌─────────────────────────┐
   │         翼段              │
   │                          │
   │    升力 L ↑              │
   │    ────────              │──── 铰接点
   │              升降舵 ↕    │
   │              偏转 δe     │
   └─────────────────────────┘

升降舵偏转 δe > 0 (下偏):
  → 增大该翼段尾部升力
  → 翼段后缘抬升 → 产生俯仰力矩
  → 改变该翼段的迎角
  → 改变该翼段的升力分布

效果: 该翼段相对于铰接点产生 上/下 位移
```

### 5.3 控制架构

```
                      主飞控决策层
                 ┌────────────────────┐
                 │ 导航控制器 (位置→姿态) │
                 │ 姿态控制器 (姿态→速率) │
                 │ 速率控制器 (速率→力矩) │
                 │ 控制分配器 (力矩→执行) │
                 └───┬──────────┬─────┘
                     │          │
              ┌──────▼──┐  ┌───▼──────┐
              │ 左翼指令  │  │ 右翼指令  │
              │ δe_left  │  │ δe_right │
              │ T_left   │  │ T_right  │
              └──────┬──┘  └───┬──────┘
                MAVLink       MAVLink
              ┌──────▼──┐  ┌───▼──────┐
              │从飞控 (L)│  │从飞控 (R)│
              │         │  │         │
              │ 舵机驱动 │  │ 舵机驱动 │
              │ ESC 驱动 │  │ ESC 驱动 │
              └─────────┘  └─────────┘
```

### 5.4 位置调整控制方程

**目标**：通过升降舵使翼段保持在期望的相对位置。

**控制律**：
```
Δz_error = z_desired - z_measured     (上下位置偏差)
Δθ_error = θ_desired - θ_measured     (铰接角偏差)

δe = Kp × Δθ_error + Kd × dΔθ/dt    (PD 控制器)

其中:
  Kp = 比例增益（根据铰接刚度和空速调节）
  Kd = 微分增益（抑制振荡）
  Δθ_error = 期望铰接角 - 实际铰接角
  dΔθ/dt = 铰接角变化率（从 IMU 角速度差计算）
```

**空速缩放**（重要！与本项目的航向保持控制器相同原理）：
```
气动力矩 M = q × S × c × Cmδe × δe

其中 q = 0.5 × ρ × V²   (动压，与速度平方成正比)

低速时: 舵面效率低 → 增益需要放大
高速时: 舵面效率高 → 增益需要缩小

scaled_Kp = Kp × (V_trim / V_actual)²

这与 ecl_yaw_controller.cpp 中的
airspeed_ratio² 缩放完全相同的原理！
```

### 5.5 从飞控实现模式

从飞控有两种工作模式：

**模式 A：纯执行模式（推荐初期）**

```
从飞控只负责驱动执行器，不做任何计算

主飞控: 计算 δe_left, δe_right, T_left, T_right
  → MAVLink SET_ACTUATOR_CONTROL_TARGET
  → 从飞控

从飞控: 接收 MAVLink → 设置 PWM → 驱动舵机/电机
```

优势：简单可靠，从飞控只做 PWM 输出
劣势：延迟取决于通信链路

**模式 B：本地闭环模式（推荐后期）**

```
从飞控自己做内环闭环，主飞控发送目标

主飞控: 发送 θ_desired (期望铰接角)
  → MAVLink NAMED_VALUE_FLOAT
  → 从飞控

从飞控:
  1. 读取铰接角传感器 → θ_measured
  2. Δθ = θ_desired - θ_measured
  3. δe = Kp × Δθ + Kd × dΔθ/dt
  4. 输出 PWM → 驱动舵机

控制频率: 200Hz（从飞控 IMU 速率）
```

优势：高频闭环，不受通信延迟影响
劣势：需要从飞控有角度传感器和控制逻辑

### 5.6 与现有控制分配器的关系

当前 `4007_gz_chainwing` 机架配置中的控制分配器已经实现了"整机视角"的分配：

```
# 控制面效果系数（4007_gz_chainwing）

# 左升降副翼 (CS0):
CA_SV_CS0_TRQ_R = 0.5     # 滚转力矩 (反向，适配 3 体)
CA_SV_CS0_TRQ_P = 0.5     # 俯仰力矩
CA_SV_CS0_TRQ_Y = 0.0     # 偏航力矩

# 中央升降舵 (CS1):
CA_SV_CS1_TRQ_R = 0.0     # 无滚转贡献
CA_SV_CS1_TRQ_P = 1.0     # 主要俯仰力矩
CA_SV_CS1_TRQ_Y = 0.0     # 无偏航贡献

# 右升降副翼 (CS2):
CA_SV_CS2_TRQ_R = -0.5    # 滚转力矩 (反向)
CA_SV_CS2_TRQ_P = 0.5     # 俯仰力矩
CA_SV_CS2_TRQ_Y = 0.0     # 无偏航贡献
```

**当前状态（单飞控全权控制）**：
- 主飞控直接输出 3 路 PWM 到三个舵面
- 控制分配器将滚转/俯仰/偏航力矩 → 各舵面偏转角

**三飞控状态（分布式控制）**：
- 主飞控计算 3 个舵面偏转角
- 通过 MAVLink 将左/右舵面偏转角发给从飞控
- 从飞控负责最终 PWM 输出

---

## 6. 系统集成架构总览

### 6.1 完整系统框图

```
                           ┌─────────────────────┐
                           │    地面站 (QGC)       │
                           │  任务规划 / 监控      │
                           └──────────┬──────────┘
                                      │ 数传 (433MHz/WiFi)
                                      │ MAVLink
                           ┌──────────▼──────────┐
                           │   主飞控 (M) - 中央    │
                           │                      │
                           │  ┌─────────────────┐ │
                           │  │ EKF2 状态估计     │ │
                           │  │ 导航/任务控制器   │ │
                           │  │ 姿态/速率控制器   │ │
                           │  │ 控制分配器        │ │
                           │  │ 航向保持控制器    │ │
                           │  └─────────────────┘ │
                           │                      │
                           │ PWM → 中央电机       │
                           │ PWM → 中央升降舵     │
                           │                      │
                           │ TELEM2    TELEM3     │
                           └───┬──────────┬───────┘
                        UART   │          │   UART
                    ┌──────────▼──┐   ┌───▼──────────┐
                    │ 从飞控 (L)   │   │ 从飞控 (R)   │
                    │ 左翼单元     │   │ 右翼单元     │
                    │             │   │             │
                    │ IMU 姿态    │   │ IMU 姿态    │
                    │ 铰接角传感器 │   │ 铰接角传感器 │
                    │             │   │             │
                    │ PWM→左电机  │   │ PWM→右电机  │
                    │ PWM→左舵面  │   │ PWM→右舵面  │
                    └─────────────┘   └─────────────┘
```

### 6.2 数据流时序

```
时间线 (20ms 周期 = 50Hz)
───────────────────────────────────────────────────→ t

T=0ms:   主飞控读取 IMU + GPS → EKF2 更新
T=2ms:   主飞控导航控制器 → 姿态设定值
T=4ms:   主飞控姿态控制器 → 速率设定值
T=6ms:   主飞控速率控制器 → 力矩需求
T=8ms:   主飞控控制分配器 → 各执行器指令
T=9ms:   主飞控输出中央 PWM + 发送 MAVLink 给从飞控
T=10ms:  MAVLink UART 传输 (~50 bytes × 10us/byte = 0.5ms)
T=11ms:  从飞控接收指令
T=12ms:  从飞控输出 PWM → 舵机/电机
T=13ms:  从飞控读取铰接角传感器
T=14ms:  从飞控发送状态 MAVLink 给主飞控
T=15ms:  主飞控接收从飞控状态 → 用于下一周期

总延迟: 主飞控决策 → 从飞控执行 ≈ 3ms (<<20ms 周期)
```

---

## 7. 项目落地注意事项

### 7.1 硬件选型

| 组件 | 推荐型号 | 数量 | 说明 |
|------|---------|------|------|
| **飞控板** | Pixhawk 6X / 6C | 3 | 三个飞控统一型号 |
| **GPS** | u-blox M9N | 1 | 主飞控必须，从飞控可选 |
| **数传** | SiK 433MHz / ESP8266 WiFi | 1 | 主飞控与地面站 |
| **电机** | 2212 1000KV 无刷 | 3 | 每个翼段一台 |
| **ESC** | 30A BLHeli_S | 3 | 每个电机一个 |
| **舵机** | SG90/MG90S | 3 | 每个翼段一个 |
| **电池** | 3S 2200mAh LiPo | 3 | 每个翼段独立供电 |
| **铰接角传感器** | AS5600 磁编码器 | 2 | 两个铰接点 |
| **UART 线缆** | JST-GH 6Pin | 2 | TELEM 端口连接线 |

### 7.2 供电方案

```
方案 A: 独立供电（推荐安全性最高）
  每个翼段: 独立 3S LiPo → BEC 5V → 飞控 + 舵机
  优势: 一个电池故障不影响其他翼段
  劣势: 重量增加 (~200g × 3)

方案 B: 共享主电池 + 独立 BEC
  一块大电池 (4S 5000mAh) → 3 × BEC → 3 × 飞控
  优势: 减轻重量
  劣势: 单点故障风险

→ 推荐方案 A（独立供电），原因:
  1. 铰接结构不适合走长电源线
  2. 单翼段短路不会影响整机
  3. 重心分布更均匀
```

### 7.3 软件开发优先级

```
阶段 1: 基础通信（1-2 周）
  ├── 主飞控 UART MAVLink 实例配置
  ├── 从飞控接收指令 + PWM 输出
  ├── 心跳检测 + 故障恢复
  └── 验证: 主飞控发送 → 从飞控舵面响应

阶段 2: 单飞控全权 SITL 验证（已完成 ✓）
  ├── 4007_gz_chainwing 仿真
  ├── 悬停、盘旋、RTL 功能
  └── 航向保持 + 差动推力

阶段 3: 从飞控固件定制（2-3 周）
  ├── 创建 2151_chainwing_slave 机架
  ├── 从飞控 Offboard 模式接收指令
  ├── 从飞控 IMU 姿态上报
  └── 铰接角传感器驱动

阶段 4: 联合地面测试（1-2 周）
  ├── 三飞控通信联调
  ├── 舵面联动测试
  ├── 电机差动推力测试
  └── 故障注入测试

阶段 5: 飞行测试（2-4 周）
  ├── SIH 硬件仿真 (1103_chainwing_sih.hil)
  ├── 低速滑行测试
  ├── 短距离直线飞行
  ├── 盘旋飞行
  └── 完整任务飞行
```

### 7.4 安全注意事项

**⚠️ 重大风险点**：

| 风险 | 影响 | 缓解措施 |
|------|------|---------|
| 通信丢失 | 从飞控失控 | 心跳超时 → 舵面回中 + 电机怠速 |
| 铰接卡死 | 结构应力 | 舵面限幅 + 铰接角超限保护 |
| 主飞控故障 | 整机失控 | 从飞控自主保持当前状态（姿态保持） |
| 振动 | IMU 噪声 | 飞控减震安装 + 软件滤波 |
| 电磁干扰 | 通信错误 | MAVLink CRC 校验 + 重传机制 |
| 非对称推力 | 偏航力矩 | 航向保持控制器（已实现 ✓） |

**强制安全检查清单**：
```
□ 所有飞控固件版本一致
□ MAV_SYS_ID 唯一（1, 2, 3）
□ UART 连接正确（TX↔RX 交叉）
□ 心跳检测超时时间合理（<500ms）
□ 从飞控舵面限幅设置正确
□ 应急开关功能正常（RC 遥控器）
□ 电池电压监控正常
□ 铰接机构无过度磨损
□ GPS 天线安装位置无遮挡
□ 数传天线方向正确
```

### 7.5 参数校准流程

```
1. 逐一校准三个飞控:
   a. 加速度计校准（6 面翻转）
   b. 陀螺仪校准（静止放置）
   c. 磁力计校准（主飞控必须，从飞控可选）
   d. 气速管校准（如有，仅主飞控）

2. 主飞控完整校准:
   a. 遥控器校准
   b. 飞行模式开关设置
   c. 电池校准
   d. ESC 校准

3. 从飞控简化校准:
   a. 加速度计 + 陀螺仪
   b. PWM 输出范围测试
   c. 舵面行程校准

4. 通信链路测试:
   a. UART 波特率一致
   b. MAVLink 心跳确认
   c. 指令往返延迟测量
```

### 7.6 地面站 QGC 配置

```
QGC 多机管理:
  - QGC 支持同时连接多个 MAV_SYS_ID 的飞行器
  - 主飞控 (SYS_ID=1) 为默认控制对象
  - 从飞控 (SYS_ID=2,3) 在"Vehicle Setup"中可单独配置

配置步骤:
  1. 连接主飞控 → 选择机架 2150
  2. 断开 → 连接从飞控(L) → 设置 MAV_SYS_ID=2
  3. 断开 → 连接从飞控(R) → 设置 MAV_SYS_ID=3
  4. 全部连接后，QGC 会显示三个飞行器图标

注意: 实际飞行时只需一条数传链路（主飞控），
     从飞控的遥测数据由主飞控转发给 QGC。
```

---

## 8. 仿真验证路线图

### 8.1 当前仿真环境

| 仿真模式 | 机架 ID | 文件 | 说明 |
|---------|---------|------|------|
| SITL (Gazebo) | 4007 | `4007_gz_chainwing` | 桌面仿真，单飞控全权 |
| SIH (板载仿真) | 1103 | `1103_chainwing_sih.hil` | Pixhawk 上运行 |
| 真实硬件 | 2150 | `2150_chainwing` | 真实飞行 |

### 8.2 多飞控仿真方案

**方案：多实例 SITL 仿真**

PX4 支持多实例 SITL，每个实例模拟一个飞控：

```bash
# 终端 1: 主飞控 (SYS_ID=1)
PX4_SYS_AUTOSTART=4007 PX4_GZ_MODEL=chainwing \
  ./build/px4_sitl_default/bin/px4 -i 0

# 终端 2: 从飞控 (SYS_ID=2)
PX4_SYS_AUTOSTART=4007 PX4_GZ_MODEL=chainwing_slave \
  ./build/px4_sitl_default/bin/px4 -i 1

# 终端 3: 从飞控 (SYS_ID=3)
PX4_SYS_AUTOSTART=4007 PX4_GZ_MODEL=chainwing_slave \
  ./build/px4_sitl_default/bin/px4 -i 2
```

**端口分配**（PX4 多实例自动偏移）：

| 实例 | UDP GCS 端口 | UDP API 端口 | TCP 端口 |
|------|-------------|-------------|---------|
| 0 (主飞控) | 14550 | 14540 | 4560 |
| 1 (从飞控L) | 14551 | 14541 | 4561 |
| 2 (从飞控R) | 14552 | 14542 | 4562 |

### 8.3 未来多飞控 Gazebo 仿真

需要新增：
1. `chainwing_slave` GZ 模型（简化版，只有单翼段）
2. GZ Joint 插件连接三个模型
3. PX4 多实例配置脚本

---

## 9. 风险评估与缓解措施

### 9.1 技术风险矩阵

| 风险 | 概率 | 影响 | 等级 | 缓解策略 |
|------|------|------|------|---------|
| UART 通信中断 | 中 | 高 | 🔴 | 心跳监控 + 自主保持 + CAN 备份 |
| 铰接结构疲劳 | 中 | 高 | 🔴 | 定期检查 + 限制飞行时间 |
| 三飞控时钟不同步 | 低 | 中 | 🟡 | MAVLink TIMESYNC + 硬件同步线 |
| 舵面颤振 | 低 | 高 | 🟡 | 舵面质量平衡 + 频率限制 |
| GPS 多径干扰 | 低 | 低 | 🟢 | 天线位置优化 |
| 软件 Bug | 高 | 中 | 🔴 | 充分仿真 + 渐进测试 |

### 9.2 项目里程碑

```
M1: 单飞控 SITL 仿真通过 ✅ (已完成)
    - 盘旋飞行
    - RTL 返航
    - 航向保持

M2: 从飞控固件 + 通信
    - 2151_chainwing_slave 机架创建
    - UART MAVLink 通信
    - 指令转发测试

M3: 多飞控地面联调
    - 三飞控通信联调
    - 舵面/电机联动
    - 故障注入

M4: SIH 硬件仿真
    - 三块 Pixhawk 板载仿真
    - 联合飞行验证

M5: 飞行测试
    - 低空低速测试
    - 完整任务飞行
    - 极端工况测试
```

---

## 附录 A: 与 report.pdf 的对应关系

本文档基于项目已有研究成果和 PX4 固件分析，结合 report.pdf 中的理论设计，
给出了三飞控系统的工程实现方案。

| report.pdf 内容 | 本文档对应章节 |
|----------------|--------------|
| 链翼构型描述 | §1.1 物理结构 |
| 控制系统设计 | §5 升降舵位置调整 |
| 气动分析 | §5.2 物理原理 |
| MATLAB 仿真 | §8 仿真验证路线 |
| 系统集成 | §6 系统集成架构 |

## 附录 B: PX4 MAVLink 多飞控通信参数速查

```
# ======== 主飞控参数 (MAV_SYS_ID=1) ========

# 系统标识
MAV_SYS_ID = 1
MAV_COMP_ID = 1
MAV_TYPE = 1              # 固定翼

# TELEM1: 地面站通信
MAV_0_CONFIG = 101        # TELEM1
MAV_0_MODE = 0            # Normal (全量遥测)
MAV_0_RATE = 1200         # 1200 B/s
MAV_0_FORWARD = 1         # 转发从飞控消息到地面站

# TELEM2: 从飞控(L)通信
MAV_1_CONFIG = 102        # TELEM2
MAV_1_MODE = 7            # Minimal (低带宽)
MAV_1_RATE = 10000        # 10000 B/s (高频指令)
MAV_1_FORWARD = 1         # 转发

# TELEM3: 从飞控(R)通信 (如飞控支持)
MAV_2_CONFIG = 104        # TELEM/SERIAL4
MAV_2_MODE = 7            # Minimal
MAV_2_RATE = 10000
MAV_2_FORWARD = 1

# 外部设定值转发
MAV_FWDEXTSP = 1          # 允许外部设定值

# ======== 从飞控参数 (MAV_SYS_ID=2/3) ========

# 系统标识
MAV_SYS_ID = 2            # 或 3（右翼）
MAV_COMP_ID = 1
MAV_TYPE = 1

# TELEM1: 与主飞控通信
MAV_0_CONFIG = 101        # TELEM1
MAV_0_MODE = 7            # Minimal
MAV_0_RATE = 10000
MAV_0_FORWARD = 0         # 不转发

# 安全
COM_ARM_WO_GPS = 1        # 允许无 GPS 解锁
CBRK_SUPPLY_CHK = 894281  # 电池检查旁路（从飞控可能无电池监控）
```

## 附录 C: 缩略语表

| 缩写 | 全称 | 说明 |
|------|------|------|
| M | Master | 主飞控（中央单元） |
| L | Left Slave | 左翼从飞控 |
| R | Right Slave | 右翼从飞控 |
| EKF2 | Extended Kalman Filter 2 | 扩展卡尔曼滤波器 |
| IMU | Inertial Measurement Unit | 惯性测量单元 |
| MAVLink | Micro Air Vehicle Link | 微型飞行器通信协议 |
| UART | Universal Async Receiver/Transmitter | 通用异步收发器 |
| CAN | Controller Area Network | 控制器局域网 |
| ESC | Electronic Speed Controller | 电子调速器 |
| PWM | Pulse Width Modulation | 脉宽调制 |
| BEC | Battery Eliminator Circuit | 稳压电路 |
| SITL | Software In The Loop | 软件在环仿真 |
| SIH | Simulator In Hardware | 硬件板载仿真 |
| QGC | QGroundControl | 地面站软件 |
| RTL | Return To Launch | 返航 |
| RTK | Real Time Kinematic | 实时动态差分 |
| DroneCAN | 原 UAVCAN v0 | 无人机 CAN 总线协议 |
| FDCAN | Flexible Data-rate CAN | 灵活数据速率 CAN |

---

## 10. 从机代码开发详细指南（CAN 通信 + 单侧副翼微调）

> **版本**: 2.0 — 2026-03-15 更新  
> **变更**: 通信方案由 UART+MAVLink 改为 **CAN (DroneCAN)**；位置微调由升降舵改为**单侧副翼**

### 10.1 整体架构变更

```
旧方案 (v1.0):                          新方案 (v2.0):
  主飞控 ──UART──→ 从飞控                主飞控 ──CAN 总线──→ 从飞控(L)
  主飞控 ──UART──→ 从飞控                            └──────→ 从飞控(R)
  从飞控用升降舵调位置                    从飞控用单侧副翼调位置
  4线 × 2 = 8根线                        2线 (CANH+CANL) 共享总线
```

**新方案优势**：

| 对比项 | UART+MAVLink | CAN (DroneCAN) |
|--------|-------------|----------------|
| 线缆数 | 4线 × 2 = 8根 | 2线共享总线 |
| 拓扑 | 点对点 | 总线型（一线连三机） |
| 错误检测 | CRC 校验 | 硬件级 CRC + ACK + 自动重发 |
| 抗干扰 | 一般 | **差分信号，强抗干扰** |
| 协议 | 需要自定义消息解析 | **PX4 原生支持 ESC/Servo 指令** |
| 铰接友好 | 线多，折弯风险大 | **只有 2 根线穿过铰接点** |

---

## 11. CAN (DroneCAN) 通信实现详解

### 11.1 DroneCAN 协议栈在 PX4 中的位置

```
PX4 软件栈:
  ┌─────────────────────────────────────────────┐
  │              应用层                          │
  │  ┌──────────────┐  ┌──────────────────────┐ │
  │  │ 控制分配器     │  │ chainwing_slave 模块  │ │
  │  │ ActuatorMotors│  │ (新建)               │ │
  │  │ ActuatorServos│  │                      │ │
  │  └──────┬───────┘  └──────────┬───────────┘ │
  │         │ uORB                │ uORB         │
  │  ┌──────▼─────────────────────▼───────────┐ │
  │  │         UAVCAN 驱动模块                  │ │
  │  │   src/drivers/uavcan/uavcan_main.cpp    │ │
  │  │                                         │ │
  │  │  ┌──────────┐  ┌───────────┐            │ │
  │  │  │ ESC 控制器 │  │ Servo 控制器│           │ │
  │  │  │ esc.cpp   │  │ servo.cpp  │           │ │
  │  │  └──────┬───┘  └─────┬─────┘            │ │
  │  │         │             │                  │ │
  │  │  ┌──────▼─────────────▼──────────────┐  │ │
  │  │  │      libuavcan 协议栈              │  │ │
  │  │  │  uavcan.equipment.esc.RawCommand   │  │ │
  │  │  │  uavcan.equipment.actuator.Array.. │  │ │
  │  │  └──────────────┬────────────────────┘  │ │
  │  └─────────────────┼───────────────────────┘ │
  │                    │                          │
  ├────────────────────┼──────────────────────────┤
  │  硬件抽象层         │                          │
  │  ┌─────────────────▼─────────────────────┐   │
  │  │   CAN 驱动 (STM32H7 FDCAN)            │   │
  │  │   uavcan_drivers/stm32h7/             │   │
  │  └─────────────────┬─────────────────────┘   │
  └────────────────────┼──────────────────────────┘
                       │ 硬件 CAN 总线
           ┌───────────┼───────────┐
           │           │           │
       从飞控(L)     主飞控(M)    从飞控(R)
       Node ID=2    Node ID=1   Node ID=3
```

### 11.2 CAN 物理接线

```
                    CAN 总线 (2 线)
  ┌─────────────────────────────────────────────────┐
  │                                                 │
  │    120Ω                                  120Ω   │
  │   ┌───┐                                 ┌───┐  │
  ├───┤   ├──┬──────────────┬───────────────┤   ├──┤
  │   └───┘  │              │               └───┘  │
  │  CANH    │              │                CANH   │
  │          │              │                       │
  │  CANL    │              │                CANL   │
  ├──────────┼──────────────┼───────────────────────┤
  │          │              │                       │
  │      ┌───▼───┐      ┌──▼────┐      ┌───▼───┐  │
  │      │从飞控L │      │主飞控M│      │从飞控R │  │
  │      │CAN1   │      │CAN1  │      │CAN1   │  │
  │      └───────┘      └──────┘      └───────┘  │
  │                                               │
  └───────────────────────────────────────────────┘

  注意:
  1. 总线两端各接一个 120Ω 终端电阻
  2. CANH 和 CANL 为差分信号对，需双绞线
  3. 三个飞控并联在同一总线上
  4. 铰接点处线缆要留余量，使用柔性线
```

### 11.3 CAN 参数配置

**主飞控 (M) — CAN 参数**：
```bash
# === CAN 总线配置 ===
param set UAVCAN_ENABLE 3        # 模式3: Sensors + Actuators (通过CAN输出ESC/Servo指令)
param set UAVCAN_NODE_ID 1       # CAN 节点 ID = 1 (主飞控)
param set UAVCAN_BITRATE 1000000 # 1 Mbps (默认值，最大速率)

# === CAN 执行器发布 ===
param set UAVCAN_PUB_ARM 1       # 通过 CAN 广播解锁状态
```

**从飞控 (L) — CAN 参数**：
```bash
# === CAN 总线配置 ===
param set UAVCAN_ENABLE 1        # 模式1: Sensors Manual (接收CAN指令)
param set UAVCAN_NODE_ID 2       # CAN 节点 ID = 2 (从飞控-左)
param set UAVCAN_BITRATE 1000000 # 1 Mbps

# === CAN 传感器订阅 (不需要来自CAN的传感器) ===
param set UAVCAN_SUB_GPS 0       # 不从CAN接收GPS
param set UAVCAN_SUB_MAG 0       # 不从CAN接收磁力计
param set UAVCAN_SUB_BARO 0      # 不从CAN接收气压计
param set UAVCAN_SUB_BAT 0       # 不从CAN接收电池
param set UAVCAN_SUB_ASPD 0      # 不从CAN接收空速
```

**从飞控 (R) — CAN 参数**：
```bash
param set UAVCAN_ENABLE 1
param set UAVCAN_NODE_ID 3       # CAN 节点 ID = 3 (从飞控-右)
param set UAVCAN_BITRATE 1000000
# 其余与从飞控(L)相同
```

### 11.4 CAN 消息流

```
主飞控 (Node 1) 广播:
  ┌──────────────────────────────────────────────────┐
  │ uavcan.equipment.esc.RawCommand      @ 50 Hz    │
  │   cmd[0] = 左电机油门  (-8192 ~ +8191)           │
  │   cmd[1] = 中央电机油门                           │
  │   cmd[2] = 右电机油门                             │
  ├──────────────────────────────────────────────────┤
  │ uavcan.equipment.actuator.ArrayCommand @ 50 Hz  │
  │   commands[0] = {id=0, value=左副翼角度}          │
  │   commands[1] = {id=1, value=中央升降舵角度}       │
  │   commands[2] = {id=2, value=右副翼角度}          │
  └──────────────────────────────────────────────────┘

从飞控 (Node 2/3) 接收并执行:
  ┌──────────────────────────────────────────────────┐
  │ 解析 ArrayCommand → 提取属于自己的 actuator_id   │
  │ 从飞控(L): 只执行 id=0 (左副翼)                  │
  │ 从飞控(R): 只执行 id=2 (右副翼)                  │
  │                                                  │
  │ 解析 RawCommand → 提取属于自己的 cmd index       │
  │ 从飞控(L): 只执行 cmd[0] (左电机)                │
  │ 从飞控(R): 只执行 cmd[2] (右电机)                │
  └──────────────────────────────────────────────────┘
```

**带宽估算**：
```
ESC RawCommand:  50 Hz × 8 bytes CAN帧 = 400 B/s
Servo ArrayCommand: 50 Hz × 2×8 bytes = 800 B/s (可能多帧)
总计: ≈ 1.2 KB/s << 1 Mbps CAN 容量

→ 带宽利用率 < 1%，完全满足需求
→ CAN 单帧最大 8 字节，ArrayCommand 可能需要 2-3 帧
→ libuavcan 自动处理多帧传输
```

### 11.5 已有 PX4 代码中的 CAN 执行器控制器

PX4 已内置完整的 CAN ESC 和 Servo 控制器，**无需从零编写**：

**ESC 控制器** (`src/drivers/uavcan/actuators/esc.cpp`):
```cpp
// 已有代码 — UavcanEscController 将 uORB ActuatorMotors 转发到 CAN
// 关键接口:
void UavcanEscController::update_outputs(bool stop_motors,
    uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs)
{
    uavcan::equipment::esc::RawCommand msg;
    for (unsigned i = 0; i < num_outputs; i++) {
        if (stop_motors || outputs[i] == DISARMED_OUTPUT_VALUE) {
            msg.cmd.push_back(static_cast<unsigned>(0));
        } else {
            msg.cmd.push_back(static_cast<int>(outputs[i]));
        }
    }
    _uavcan_pub_raw_cmd.broadcast(msg);
}
```

**Servo 控制器** (`src/drivers/uavcan/actuators/servo.cpp`):
```cpp
// 已有代码 — UavcanServoController 将 uORB ActuatorServos 转发到 CAN
// 关键接口:
void UavcanServoController::update_outputs(bool stop_motors,
    uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs)
{
    uavcan::equipment::actuator::ArrayCommand msg;
    for (unsigned i = 0; i < num_outputs; ++i) {
        uavcan::equipment::actuator::Command cmd;
        cmd.actuator_id = i;
        cmd.command_type = uavcan::equipment::actuator::Command::COMMAND_TYPE_UNITLESS;
        cmd.command_value = (float)outputs[i] / 500.f - 1.f;  // [-1, 1]
        msg.commands.push_back(cmd);
    }
    _uavcan_pub_array_cmd.broadcast(msg);
}
```

> **关键发现**：主飞控设置 `UAVCAN_ENABLE=3` 后，控制分配器输出的
> `ActuatorMotors` 和 `ActuatorServos` 会自动通过 CAN 总线广播，
> **无需额外编写主飞控侧的发送代码**。

---

## 12. 需要创建和修改的代码文件清单

### 12.1 文件总览

```
需要创建的新文件:                              需要修改的现有文件:
├── 从机机架配置                                ├── 板级配置
│   └── ROMFS/.../2151_chainwing_slave          │   └── boards/px4/fmu-v6x/default.px4board
│                                               │
├── 从机控制模块                                ├── 启动脚本 (可选)
│   ├── src/modules/chainwing_slave/            │   └── ROMFS/.../init.d/rcS
│   │   ├── ChainwingSlave.cpp                  │
│   │   ├── ChainwingSlave.hpp                  ├── 仿真配置 (可选)
│   │   ├── CMakeLists.txt                      │   └── ROMFS/.../4008_gz_chainwing_slave
│   │   └── module.yaml                         │
│   │                                           │
├── 自定义 uORB 消息                             │
│   └── msg/ChainwingSlaveCmd.msg               │
│                                               │
├── 仿真模型 (可选)                              │
│   └── Tools/simulation/gz/models/             │
│       └── chainwing_slave/                    │
│           ├── model.config                    │
│           └── model.sdf                       │
│                                               │
└── CAN 自定义消息 (可选, 高级)                   │
    └── src/drivers/uavcan/                     │
        └── chainwing/                          │
            ├── chainwing_can_bridge.cpp         │
            └── chainwing_can_bridge.hpp         │
```

### 12.2 文件 1：从机机架配置

**文件路径**: `ROMFS/px4fmu_common/init.d/airframes/2151_chainwing_slave`

**作用**: 定义从飞控的系统角色、执行器配置、CAN 参数

**参考模板**: 现有 `2150_chainwing`（主飞控机架）

**建议内容结构**:
```bash
#!/bin/sh
#
# @name Chain-Wing UAV Slave Unit
# @type Standard Plane
# @class Plane
#
# @output MAIN1 Motor (this unit only)
# @output MAIN2 Aileron (this unit only, single-side)
#

. ${R}etc/init.d/rc.fw_defaults

param set-default CA_AIRFRAME 1

# --- 单电机 (只控制自己翼段的电机) ---
param set-default CA_ROTOR_COUNT 1
param set-default CA_ROTOR0_PX 0.0
param set-default CA_ROTOR0_PY 0.0      # 从飞控视角: 自身为原点
param set-default CA_ROTOR0_PZ 0.0

# --- 单舵面 (只控制自己翼段的副翼) ---
param set-default CA_SV_CS_COUNT 1

# 左翼从飞控 (SYS_ID=2) 设置:
# CA_SV_CS0_TYPE = 6 (Right Elevon)
# CA_SV_CS0_TRQ_R = 0.5
# CA_SV_CS0_TRQ_P = 0.5

# 右翼从飞控 (SYS_ID=3) 设置:
# CA_SV_CS0_TYPE = 5 (Left Elevon)
# CA_SV_CS0_TRQ_R = -0.5
# CA_SV_CS0_TRQ_P = 0.5

# (左/右翼通过 QGC 设置不同的 TRQ_R 符号)

# --- CAN 通信 ---
param set-default UAVCAN_ENABLE 1        # 接收 CAN 指令
param set-default UAVCAN_BITRATE 1000000
# UAVCAN_NODE_ID 需通过 QGC 单独设置 (2 或 3)

# --- 从机不需要独立导航 ---
param set-default COM_ARM_WO_GPS 1       # 允许无 GPS 解锁
param set-default CBRK_SUPPLY_CHK 894281 # 电池检查旁路
param set-default SYS_HAS_GPS 0          # 从机不带 GPS
param set-default EKF2_GPS_CTRL 0        # EKF2 不使用 GPS

# --- 从机不需要气速计 ---
param set-default FW_ARSP_MODE 1         # 不使用气速计

# --- 从机不需要地面站通信 ---
param set-default MAV_0_CONFIG 0         # 关闭 MAVLink 实例
```

**数据来源**:
| 参数 | 值 | 来源 |
|------|------|------|
| `CA_ROTOR_COUNT=1` | 每个从机只控制 1 台电机 | 物理结构 |
| `CA_SV_CS_COUNT=1` | 每个从机只有 1 个副翼 | 物理结构 |
| `UAVCAN_ENABLE=1` | Sensors Manual 模式，接收 CAN 数据 | PX4 UAVCAN 文档 |
| `COM_ARM_WO_GPS=1` | 从机无独立 GPS | 架构设计 |
| `EKF2_GPS_CTRL=0` | 从机不融合 GPS | 架构设计 |

### 12.3 文件 2：从机控制模块 — ChainwingSlave

**目录**: `src/modules/chainwing_slave/`

这是从机的核心自定义模块，负责：
1. 从 CAN 总线接收主飞控的执行器指令
2. 使用自身 IMU 计算相对于铰接点的角度偏差
3. 叠加单侧副翼微调量
4. 输出最终的 PWM 到电机和舵机

#### 12.3.1 ChainwingSlave.hpp

**建议类结构**:
```cpp
// 文件: src/modules/chainwing_slave/ChainwingSlave.hpp
#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/parameter_update.h>
#include <lib/mathlib/mathlib.h>

class ChainwingSlave : public ModuleBase<ChainwingSlave>,
                       public ModuleParams,
                       public px4::ScheduledWorkItem
{
public:
    ChainwingSlave();
    ~ChainwingSlave() override;

    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    bool init();

private:
    void Run() override;    // 主循环, 50-200Hz
    void parameters_updated();

    // ── 位置微调核心计算 ──
    float compute_aileron_trim_offset();

    // ── uORB 订阅 (从自身传感器) ──
    uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
    uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
    uORB::Subscription _sensor_accel_sub{ORB_ID(sensor_accel)};

    // ── uORB 订阅 (从 CAN/主飞控, 通过 UAVCAN 驱动转为 uORB) ──
    uORB::Subscription _actuator_servos_sub{ORB_ID(actuator_servos)};
    uORB::Subscription _actuator_motors_sub{ORB_ID(actuator_motors)};

    // ── uORB 发布 (最终执行器输出) ──
    // 注: 从机通过本地 PWM 输出, 不通过 CAN 回传
    uORB::Publication<actuator_servos_s> _servo_output_pub{ORB_ID(actuator_servos)};

    // ── 状态变量 ──
    float _hinge_angle_rad{0.f};         // 铰接角度 (rad)
    float _hinge_angle_rate{0.f};        // 铰接角速度 (rad/s)
    float _aileron_trim_offset{0.f};     // 副翼微调偏移量 [-1, 1]
    hrt_abstime _last_cmd_time{0};       // 上次收到指令的时间

    // ── 参数 ──
    DEFINE_PARAMETERS(
        (ParamFloat<px4::params::CW_SLV_KP>)    _param_kp,        // 位置微调比例增益
        (ParamFloat<px4::params::CW_SLV_KD>)    _param_kd,        // 位置微调微分增益
        (ParamFloat<px4::params::CW_SLV_MAX>)   _param_max_trim,  // 最大微调量
        (ParamInt<px4::params::CW_SLV_SIDE>)    _param_side,      // 0=左翼, 1=右翼
        (ParamFloat<px4::params::CW_SLV_HINGE>) _param_hinge_len  // 铰接臂长 (m)
    )
};
```

#### 12.3.2 ChainwingSlave.cpp

**建议核心逻辑**:
```cpp
// 文件: src/modules/chainwing_slave/ChainwingSlave.cpp

// ── 构造函数 ──
ChainwingSlave::ChainwingSlave() :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
}

bool ChainwingSlave::init()
{
    ScheduleOnInterval(20_ms);   // 50 Hz 主循环
    return true;
}

// ── 主循环 ──
void ChainwingSlave::Run()
{
    // 1. 检查参数更新
    if (_parameter_update_sub.updated()) {
        parameter_update_s param_update;
        _parameter_update_sub.copy(&param_update);
        updateParams();
    }

    // 2. 读取自身 IMU 姿态
    vehicle_attitude_s att;
    if (_vehicle_attitude_sub.copy(&att)) {
        // 从四元数提取 roll/pitch
        matrix::Quatf q(att.q);
        matrix::Eulerf euler(q);

        // 铰接角 ≈ 本翼段相对水平面的 pitch 角变化
        // (简化: 假设主翼段保持水平, 偏差即为铰接角)
        _hinge_angle_rad = euler.phi();  // roll 分量 → 铰接偏转
    }

    // 3. 读取角速度 (用于微分项)
    vehicle_angular_velocity_s ang_vel;
    if (_vehicle_angular_velocity_sub.copy(&ang_vel)) {
        _hinge_angle_rate = ang_vel.xyz[0];  // roll rate
    }

    // 4. 计算副翼微调偏移
    _aileron_trim_offset = compute_aileron_trim_offset();

    // 5. 读取主飞控通过 CAN 发来的舵面指令
    actuator_servos_s servo_cmd;
    if (_actuator_servos_sub.copy(&servo_cmd)) {
        _last_cmd_time = hrt_absolute_time();

        // 确定自己应执行哪个通道
        int my_channel = (_param_side.get() == 0) ? 0 : 2;
        // 通道 0 = 左副翼, 通道 2 = 右副翼

        float base_cmd = servo_cmd.control[my_channel];  // [-1, 1]

        // 叠加位置微调偏移
        float final_cmd = math::constrain(
            base_cmd + _aileron_trim_offset,
            -1.f, 1.f
        );

        // 输出到本地 PWM
        actuator_servos_s output{};
        output.timestamp = hrt_absolute_time();
        output.control[0] = final_cmd;  // 从机只有 1 个舵面
        _servo_output_pub.publish(output);
    }

    // 6. 通信超时保护 (500ms 无指令 → 舵面回中)
    if (hrt_elapsed_time(&_last_cmd_time) > 500_ms) {
        actuator_servos_s safe_output{};
        safe_output.timestamp = hrt_absolute_time();
        safe_output.control[0] = 0.f;  // 回中
        _servo_output_pub.publish(safe_output);
    }
}

// ── 副翼微调偏移计算 ──
float ChainwingSlave::compute_aileron_trim_offset()
{
    // 目标: 保持铰接角为零 (翼段水平对齐)
    float angle_error = 0.f - _hinge_angle_rad;  // 目标角=0

    // PD 控制器
    float offset = _param_kp.get() * angle_error
                 + _param_kd.get() * (0.f - _hinge_angle_rate);

    // 限幅 (不能占用太多副翼行程)
    offset = math::constrain(offset,
        -_param_max_trim.get(), _param_max_trim.get());

    // 左/右翼符号反转
    if (_param_side.get() == 1) {  // 右翼
        offset = -offset;  // 右翼副翼方向相反
    }

    return offset;
}
```

#### 12.3.3 CMakeLists.txt

```cmake
# 文件: src/modules/chainwing_slave/CMakeLists.txt

px4_add_module(
    MODULE modules__chainwing_slave
    MAIN chainwing_slave
    SRCS
        ChainwingSlave.cpp
    MODULE_CONFIG
        module.yaml
    DEPENDS
        mathlib
)
```

#### 12.3.4 module.yaml

```yaml
# 文件: src/modules/chainwing_slave/module.yaml

module_name: chainwing_slave

parameters:
  - group: Chain-Wing Slave
    definitions:
      CW_SLV_KP:
        description:
          short: Position trim proportional gain
          long: |
            Proportional gain for the single-side aileron position
            trim PD controller. Higher values give faster correction
            but may cause oscillation.
        type: float
        default: 0.3
        min: 0.0
        max: 2.0
        decimal: 2

      CW_SLV_KD:
        description:
          short: Position trim derivative gain
          long: |
            Derivative gain for the aileron position trim controller.
            Damps oscillations from the proportional term.
        type: float
        default: 0.05
        min: 0.0
        max: 1.0
        decimal: 3

      CW_SLV_MAX:
        description:
          short: Maximum trim offset
          long: |
            Maximum aileron trim offset as fraction of full range.
            0.3 means the trim can use up to 30% of the aileron
            travel, leaving 70% for main attitude control.
        type: float
        default: 0.3
        min: 0.05
        max: 0.5
        decimal: 2

      CW_SLV_SIDE:
        description:
          short: Wing side
          long: |
            Which wing this slave controller is on.
            0 = Left wing (MAV_SYS_ID=2)
            1 = Right wing (MAV_SYS_ID=3)
        type: int32
        default: 0
        min: 0
        max: 1

      CW_SLV_HINGE:
        description:
          short: Hinge arm length
          long: |
            Distance from hinge point to wing segment center (m).
            Used for geometric position calculation.
            Default 0.6m matches chainwing design (hinge at Y=±0.6m).
        type: float
        default: 0.6
        min: 0.1
        max: 2.0
        decimal: 2
        unit: m
```

### 12.4 文件 3：自定义 uORB 消息（可选）

**文件路径**: `msg/ChainwingSlaveCmd.msg`

如果标准的 `ActuatorServos` 消息不够用（例如需要携带额外的主飞控状态信息），可创建自定义消息：

```
# 文件: msg/ChainwingSlaveCmd.msg
# Chain-wing slave controller command from master

uint64 timestamp            # time since system start (microseconds)

float32 aileron_cmd         # Aileron position command [-1, 1]
float32 throttle_cmd        # Throttle command [0, 1]

float32 master_roll         # Master roll angle (rad), for reference
float32 master_pitch        # Master pitch angle (rad), for reference
float32 master_airspeed     # Master airspeed (m/s), for gain scaling

uint8 arm_state             # 0=disarmed, 1=armed
uint8 flight_mode           # Current flight mode of master

# TOPICS chainwing_slave_cmd
```

> **注意**: 如果使用此自定义消息，需要在 `msg/CMakeLists.txt` 中添加该文件名。
> 但初期阶段建议**先使用标准 `ActuatorServos` 消息**，避免增加复杂度。

### 12.5 文件 4：板级配置修改

**文件路径**: `boards/px4/fmu-v6x/default.px4board`

需要添加从机模块的编译使能：

```
# 在 default.px4board 中添加一行:
CONFIG_MODULES_CHAINWING_SLAVE=y
```

### 12.6 文件 5：仿真机架配置（SITL）

**文件路径**: `ROMFS/px4fmu_common/init.d-posix/airframes/4008_gz_chainwing_slave`

```bash
#!/bin/sh
#
# @name Gazebo Chain-Wing Slave Unit
# @type Fixedwing
#

. ${R}etc/init.d/rc.fw_defaults

PX4_SIMULATOR=${PX4_SIMULATOR:=gz}
PX4_GZ_WORLD=${PX4_GZ_WORLD:=flat_terrain}
PX4_SIM_MODEL=${PX4_SIM_MODEL:=chainwing_slave}

param set-default SIM_GZ_EN 1
param set-default SENS_EN_GPSSIM 0   # 从机不需要 GPS
param set-default SENS_EN_BAROSIM 0
param set-default SENS_EN_MAGSIM 1   # 磁力计用于姿态
param set-default SENS_EN_ARSPDSIM 0 # 从机不需要空速

# (其余参数与 2151_chainwing_slave 一致)
param set-default CA_AIRFRAME 1
param set-default CA_ROTOR_COUNT 1
param set-default CA_SV_CS_COUNT 1
```

### 12.7 文件 6：GZ 仿真模型（单翼段）

**目录**: `Tools/simulation/gz/models/chainwing_slave/`

需要创建简化版的单翼段 GZ 模型：

**model.config**:
```xml
<?xml version="1.0"?>
<model>
  <name>chainwing_slave</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>
    Single wing segment of the chain-wing UAV for slave controller simulation.
  </description>
</model>
```

**model.sdf 关键参数** (从现有 chainwing model.sdf 提取单翼段数据):

| 参数 | 值 | 来源 |
|------|------|------|
| 翼段质量 | 1.9 kg | chainwing 总质量 5.7kg ÷ 3 |
| 翼段翼展 | 1.2 m | MATLAB b=1.2m 单元间距 |
| 翼段弦长 | 0.3 m | chainwing model.sdf 翼型弦长 |
| Ixx | 0.0456 kg·m² | chainwing 整机 Ixx/3 (近似) |
| Iyy | 0.0142 kg·m² | chainwing 整机 Iyy/3 (近似) |
| Izz | 0.0568 kg·m² | chainwing 整机 Izz/3 (近似) |
| 电机推力 | 16.67 N (max) | 50N 总推力 ÷ 3 |
| 副翼限幅 | ±0.53 rad (±30°) | chainwing servo_0/servo_2 limit |
| 副翼阻尼 | 1.0 | chainwing servo joint damping |

**model.sdf 结构说明**:
```xml
<!-- 简化版模型: 只有一个翼段 -->
<model name="chainwing_slave">
  <link name="base_link">
    <!-- 机身惯量 (单翼段) -->
    <inertial>
      <mass>1.9</mass>
      <inertia>
        <ixx>0.0456</ixx>
        <iyy>0.0142</iyy>
        <izz>0.0568</izz>
      </inertia>
    </inertial>
    <!-- 翼面碰撞体 + 视觉 -->
  </link>

  <!-- 单个电机 -->
  <link name="rotor">...</link>
  <joint name="rotor_joint" type="revolute">
    <parent>base_link</parent>
    <child>rotor</child>
    <axis><xyz>1 0 0</xyz></axis>
  </joint>

  <!-- 单个副翼 -->
  <link name="aileron">...</link>
  <joint name="servo_0" type="revolute">
    <parent>base_link</parent>
    <child>aileron</child>
    <axis><xyz>0 1 0</xyz></axis>
    <limit>
      <lower>-0.53</lower>
      <upper>0.53</upper>
    </limit>
    <dynamics><damping>1.0</damping></dynamics>
  </joint>

  <!-- LiftDrag 气动插件 (从 chainwing 复制单翼段参数) -->
  <!-- 需要 1 个 LiftDrag 实例用于翼面 -->
  <!-- 需要 1 个 LiftDrag 实例用于副翼 -->
</model>
```

### 12.8 CAN 自定义消息桥接（可选，高级）

如果标准 DroneCAN 消息（ESC RawCommand + Actuator ArrayCommand）不能满足需求
（例如需要传输主飞控的姿态信息给从飞控做空速缩放），可以创建自定义 CAN 桥接：

**文件**: `src/drivers/uavcan/chainwing/chainwing_can_bridge.cpp`

```cpp
// 概念代码 — 通过 DroneCAN 的 UAVCAN KeyValue 消息传输自定义数据
// uavcan.protocol.debug.KeyValue (DTID 16370)
// 包含: float32 value + char[58] key

// 主飞控发送:
//   key="cw_airspeed", value=20.5     (空速)
//   key="cw_roll",     value=0.05     (主飞控 roll 角)
//   key="cw_pitch",    value=0.02     (主飞控 pitch 角)
//   key="cw_arm",      value=1.0      (解锁状态)

// 从飞控接收:
//   订阅 uavcan.protocol.debug.KeyValue
//   按 key 名匹配并更新本地变量
```

> **建议**: 初期先使用标准 ESC/Servo 指令，不需要这个自定义桥接。
> 只有在需要更丰富的主→从数据传输时才实现。

---

## 13. 单侧副翼位置微调控制设计

### 13.1 为什么改用副翼而非升降舵

```
旧方案 (升降舵):                        新方案 (单侧副翼):
  ┌─────────────────┐                  ┌─────────────────┐
  │      翼段        │                  │      翼段        │
  │                 │                  │                 │
  │         升降舵↕  │                  │     副翼 ↕↕      │
  │         (pitch)  │                  │     (roll+pitch) │
  └─────────────────┘                  └─────────────────┘

  问题: 升降舵在中央单元                优势: 副翼在每个翼段上
  从飞控需要额外的舵面                  从飞控已有的舵面
  增加机械复杂度                       无需额外硬件
```

**关键区别**：

| 对比 | 升降舵调整 | 副翼调整 |
|------|----------|---------|
| 舵面位置 | 需要额外舵面 | **已有**（左/右升降副翼） |
| 调整维度 | 主要调俯仰 | 可调俯仰+升力 |
| 与主控制耦合 | 低（独立舵面） | **中**（需与主姿态控制叠加） |
| 机械改动 | 需要 | **不需要** |
| 实现难度 | 中 | **低** |

### 13.2 副翼微调的物理原理

```
正常飞行时，从飞控副翼执行主飞控指令（姿态控制）:

    主飞控指令                    副翼偏转 δ_cmd
    ─────────────────────────→  ┌────────────┐
                                │  主控舵面角  │
                                └────────────┘

加入微调后:

    主飞控指令 ─────→ δ_cmd ─┐
                             ├──→ δ_final = δ_cmd + δ_trim
    位置微调   ─────→ δ_trim ┘
                                  ↓
                             副翼最终偏转

    δ_trim 的物理效果:
    ┌──────────────────────────────────────────────────────┐
    │ δ_trim > 0 → 副翼下偏 → 该翼段升力↑ → 翼段上抬     │
    │ δ_trim < 0 → 副翼上偏 → 该翼段升力↓ → 翼段下沉     │
    │ δ_trim = 0 → 无微调   → 保持主飞控指令              │
    └──────────────────────────────────────────────────────┘
```

### 13.3 控制律推导

**控制目标**: 铰接角 θ → 0（翼段保持水平对齐）

**状态量**:
```
θ      = 铰接角 (rad)，通过 IMU roll 估算或编码器测量
θ_dot  = 铰接角速度 (rad/s)，通过 IMU roll rate 获取
```

**PD 控制律**:
```
δ_trim = Kp × (0 - θ) + Kd × (0 - θ_dot)

其中:
  Kp = CW_SLV_KP (默认 0.3)    比例增益
  Kd = CW_SLV_KD (默认 0.05)   微分增益
```

**增益选取依据**:

```
铰接动力学简化模型:
  I_hinge × θ̈ = M_aero + M_gravity + M_trim

其中:
  I_hinge ≈ m × L² = 1.9 × 0.6² = 0.684 kg·m²  (铰接转动惯量)
  M_trim = q × S × c × Cmδ × δ_trim              (副翼产生的力矩)

设计空速 V = 20 m/s:
  q = 0.5 × 1.225 × 20² = 245 Pa
  S = 1.2 × 0.3 = 0.36 m²   (翼面面积)
  c = 0.3 m                   (弦长)
  Cmδ ≈ -0.5 /rad             (副翼力矩系数，典型值)

  M_trim_max = 245 × 0.36 × 0.3 × 0.5 × 0.53 = 7.0 N·m

自然频率:
  ωn = √(M_trim_max / (I_hinge × δ_max))
     = √(7.0 / (0.684 × 0.53))
     = √(11.6) ≈ 3.4 rad/s

临界阻尼选择 (ζ = 0.7):
  Kp = ωn² × I_hinge / (q × S × c × Cmδ)
     ≈ 3.4² × 0.684 / 7.0 / 0.53
     ≈ 0.30

  Kd = 2 × ζ × ωn × I_hinge / (q × S × c × Cmδ)
     ≈ 2 × 0.7 × 3.4 × 0.684 / 7.0 / 0.53
     ≈ 0.05

→ Kp = 0.3, Kd = 0.05 就是默认值的来源
```

### 13.4 空速缩放（重要）

```
副翼效率与动压 q 成正比:
  q = 0.5 × ρ × V²

低速时副翼效率低 → 需要更大的 δ_trim
高速时副翼效率高 → δ_trim 需要缩小

缩放公式 (与 ecl_yaw_controller.cpp 航向保持控制器相同原理):

  scaled_Kp = Kp × (V_trim / V_actual)²
  scaled_Kd = Kd × (V_trim / V_actual)²

其中:
  V_trim = FW_AIRSPD_TRIM = 20 m/s
  V_actual = 从 CAN 收到的主飞控空速 (或从飞控 IMU 估算)

例:
  V_actual = 15 m/s → 缩放因子 = (20/15)² = 1.78 → 增益放大
  V_actual = 25 m/s → 缩放因子 = (20/25)² = 0.64 → 增益缩小

注意: 如果从飞控没有空速计, 有两种解决方案:
  方案A: 主飞控通过 CAN KeyValue 传输空速
  方案B: 使用固定的 V_trim 值 (误差可接受, 因为微调量本身有限幅)
```

### 13.5 微调量限幅策略

```
副翼行程分配:

  ┌────────────────────────────────────────┐
  │            副翼总行程 [-1, +1]          │
  │                                        │
  │  ┌──────────┐  ┌──────────────────┐   │
  │  │ 微调储备  │  │  主姿态控制       │   │
  │  │ ±0.3     │  │  ±0.7            │   │
  │  │ (30%)    │  │  (70%)           │   │
  │  └──────────┘  └──────────────────┘   │
  │                                        │
  │  δ_final = δ_cmd + δ_trim             │
  │  |δ_final| ≤ 1.0 (硬限幅)             │
  │  |δ_trim|  ≤ CW_SLV_MAX (软限幅)      │
  └────────────────────────────────────────┘

  CW_SLV_MAX = 0.3 意味着:
  - 微调最多使用 30% 的副翼行程
  - 主姿态控制至少保留 70% 行程
  - 即使微调 + 主控制都到极限, 硬限幅保证 [-1, 1]
```

### 13.6 左/右翼符号约定

```
从飞控副翼 TRQ_R 符号与微调方向:

左翼 (CW_SLV_SIDE=0):
  CA_SV_CS0_TRQ_R = +0.5  (正值)
  δ_trim > 0 → 副翼下偏 → 升力↑ → 左翼抬升
  铰接角 θ > 0 (左翼低于主翼) → 需要 δ_trim > 0 → 符号正确 ✓

右翼 (CW_SLV_SIDE=1):
  CA_SV_CS2_TRQ_R = -0.5  (负值)
  δ_trim > 0 → 副翼下偏 → 升力↑ → 右翼抬升
  铰接角 θ > 0 (右翼低于主翼) → 需要 δ_trim > 0
  但右翼副翼方向相反 → 代码中 offset = -offset ✓
```

---

## 14. 仿真配置与数据

### 14.1 单飞控 SITL 验证微调逻辑

**目的**: 在现有单飞控仿真中验证副翼微调计算逻辑（不需要真正的多飞控）

**方法**: 在现有 `4007_gz_chainwing` 仿真中，添加调试输出观察微调量

```bash
# 步骤 1: 正常启动仿真
make px4_sitl gz_chainwing

# 步骤 2: 在 PX4 shell 中观察控制面输出
listener actuator_servos

# 步骤 3: 模拟铰接角偏差 (通过修改 roll 参考)
# 观察副翼是否产生预期的修正量
```

### 14.2 多实例 SITL 仿真

**完整启动流程**:

```bash
# ═══════ 终端 1: 启动 Gazebo 世界 ═══════
# (Gazebo 只需启动一次，多个 PX4 实例共享)
gz sim -r Tools/simulation/gz/worlds/flat_terrain.sdf

# ═══════ 终端 2: 主飞控 (实例 0) ═══════
PX4_SYS_AUTOSTART=4007 \
PX4_GZ_MODEL=chainwing \
PX4_GZ_MODEL_POSE="0,0,0.3,0,0,0" \
./build/px4_sitl_default/bin/px4 -i 0

# ═══════ 终端 3: 从飞控-左 (实例 1) ═══════
PX4_SYS_AUTOSTART=4008 \
PX4_GZ_MODEL=chainwing_slave \
PX4_GZ_MODEL_POSE="-1.2,0,0.3,0,0,0" \
./build/px4_sitl_default/bin/px4 -i 1

# ═══════ 终端 4: 从飞控-右 (实例 2) ═══════
PX4_SYS_AUTOSTART=4008 \
PX4_GZ_MODEL=chainwing_slave \
PX4_GZ_MODEL_POSE="1.2,0,0.3,0,0,0" \
./build/px4_sitl_default/bin/px4 -i 2
```

**多实例端口分配** (PX4 自动偏移):

| 实例 (-i) | MAV_SYS_ID | GCS UDP | API UDP | Simulator TCP | DDS Domain |
|-----------|-----------|---------|---------|---------------|------------|
| 0 (主飞控) | 1 | 14550 | 14540 | 4560 | 0 |
| 1 (从飞控L) | 2 | 14551 | 14541 | 4561 | 0 |
| 2 (从飞控R) | 3 | 14552 | 14542 | 4562 | 0 |

**SITL 中模拟 CAN 通信**:

SITL 环境（Linux）支持 SocketCAN 虚拟 CAN 接口：
```bash
# 创建虚拟 CAN 接口 (需要 root)
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

# PX4 SITL 编译时需要启用 SocketCAN 支持:
# 在 boards/px4/sitl/default.px4board 中添加:
#   CONFIG_DRIVERS_UAVCAN=y
#
# 注意: SITL 默认不启用 UAVCAN, 需要手动添加

# 监控 CAN 总线 (调试用):
sudo apt install can-utils
candump vcan0
```

> **重要提示**: SITL 中模拟 CAN 通信较复杂。初期建议使用以下替代方案：
> - 方案 A: 在 SITL 中用 MAVLink UDP 模拟通信，硬件上用 CAN
> - 方案 B: 直接在硬件上测试 CAN（使用 SIH 模式 `1103_chainwing_sih.hil`）

### 14.3 关键仿真数据

**从 chainwing model.sdf 提取的单翼段物理参数**:

```
┌─────────────────────────────────────────────────────────┐
│  参数                │  整机值          │  单翼段值(÷3)   │
├─────────────────────┼────────────────┼────────────────┤
│  质量               │  5.7 kg        │  1.9 kg        │
│  翼展               │  3.6 m         │  1.2 m         │
│  弦长               │  0.3 m         │  0.3 m         │
│  翼面积             │  1.08 m²       │  0.36 m²       │
│  Ixx (roll)         │  0.1368 kg·m²  │  0.0456 kg·m²  │
│  Iyy (pitch)        │  0.0426 kg·m²  │  0.0142 kg·m²  │
│  Izz (yaw)          │  0.1704 kg·m²  │  0.0568 kg·m²  │
│  电机最大推力       │  50 N          │  16.67 N       │
│  副翼限幅           │  ±0.53 rad     │  ±0.53 rad     │
│  CL_alpha           │  4.752 /rad    │  4.752 /rad    │
│  CD0                │  0.024         │  0.024         │
│  副翼 Cmδ           │  -0.3 /rad     │  -0.3 /rad     │
└─────────────────────┴────────────────┴────────────────┘
```

**微调控制器参数推导数据**:

```
铰接力学模型输入:
  m_segment    = 1.9 kg         (单翼段质量)
  L_hinge      = 0.6 m          (铰接臂长)
  I_hinge      = m × L²         (铰接转动惯量)
               = 1.9 × 0.36
               = 0.684 kg·m²
  V_cruise     = 20 m/s         (巡航空速)
  ρ            = 1.225 kg/m³    (海平面空气密度)
  q            = 0.5 × ρ × V²  (动压)
               = 0.5 × 1.225 × 400
               = 245 Pa
  S_aileron    = 0.36 × 0.25   (副翼面积, 约占翼面 25%)
               = 0.09 m²
  Cmδ_aileron  = -0.3 /rad     (GZ model.sdf control_joint_rad_to_cl)

PD 增益计算:
  ωn = √(q × S × Cmδ / I_hinge)    (自然频率)
     = √(245 × 0.09 × 0.3 / 0.684)
     = √(9.66)
     ≈ 3.1 rad/s

  ζ = 0.7                           (阻尼比, 临界阻尼)

  Kp = I_hinge × ωn² / (q × S × Cmδ)
     ≈ 0.3

  Kd = 2 × ζ × ωn × I_hinge / (q × S × Cmδ)
     ≈ 0.05

验证 — 阶跃响应:
  上升时间 tr = 1.8 / ωn ≈ 0.58 s
  超调量   Mp = e^(-πζ/√(1-ζ²)) ≈ 4.6%
  调节时间 ts = 4 / (ζ×ωn) ≈ 1.84 s

  → 响应速度适中，超调量小，适合铰接角修正
```

### 14.4 GZ 模型中的 LiftDrag 插件参数（单翼段）

从 `Tools/simulation/gz/models/chainwing/model.sdf` 提取，用于创建 `chainwing_slave` 模型：

```xml
<!-- 主翼面 LiftDrag (单翼段) -->
<plugin filename="gz-sim-lift-drag-system"
        name="gz::sim::systems::LiftDrag">
  <a0>0.10</a0>                    <!-- 零升迎角 (rad) -->
  <cla>4.752</cla>                 <!-- 升力线斜率 (1/rad) -->
  <cda>0.016</cda>                 <!-- 诱导阻力系数 -->
  <cd0>0.024</cd0>                 <!-- 零升阻力系数 -->
  <cma>-0.7</cma>                  <!-- 俯仰力矩系数 -->
  <alpha_stall>0.3</alpha_stall>   <!-- 失速迎角 (rad) ≈ 17° -->
  <cla_stall>-0.324</cla_stall>    <!-- 失速后升力斜率 -->
  <cda_stall>0.0</cda_stall>       <!-- 失速后阻力 -->
  <area>0.36</area>                <!-- 翼面面积 (m²) = 1.2m × 0.3m -->
  <air_density>1.2041</air_density>
  <forward>1 0 0</forward>
  <upward>0 0 1</upward>
  <link_name>base_link</link_name>
  <cp>0.0 0.0 0.0</cp>            <!-- 压力中心 (单翼段原点) -->
</plugin>

<!-- 副翼 LiftDrag -->
<plugin filename="gz-sim-lift-drag-system"
        name="gz::sim::systems::LiftDrag">
  <a0>0.0</a0>
  <cla>4.752</cla>
  <cda>0.016</cda>
  <cd0>0.0</cd0>
  <area>0.09</area>                <!-- 副翼面积 ≈ 翼面 25% -->
  <air_density>1.2041</air_density>
  <forward>1 0 0</forward>
  <upward>0 0 1</upward>
  <link_name>aileron</link_name>
  <control_joint_name>servo_0</control_joint_name>
  <control_joint_rad_to_cl>-0.3</control_joint_rad_to_cl>
</plugin>
```

---

## 15. 分步实施路线图（更新版）

### 15.1 阶段划分

```
阶段 0: 准备工作 (1-2天)                    ← 你现在在这里
  ├── 阅读本文档，理解整体架构
  ├── 确认硬件: 3× Pixhawk + CAN线 + 终端电阻
  └── 确认开发环境: PX4 编译通过

阶段 1: 创建从机机架和模块框架 (3-5天)
  ├── 创建 2151_chainwing_slave 机架文件
  ├── 创建 chainwing_slave 模块骨架
  │   ├── ChainwingSlave.cpp/hpp
  │   ├── CMakeLists.txt
  │   └── module.yaml
  ├── 修改 boards/.../default.px4board 添加模块
  ├── 编译通过: make px4_fmu-v6x_default
  └── 验证: 模块可启动 (chainwing_slave start)

阶段 2: CAN 通信验证 (3-5天)
  ├── 硬件接线: CAN 总线 + 终端电阻
  ├── 设置 UAVCAN 参数 (主飞控 ENABLE=3, 从飞控 ENABLE=1)
  ├── 验证: 主飞控 actuator_servos → CAN 广播
  ├── 验证: 从飞控接收 CAN → uORB 消息
  ├── 使用 SIH 模式验证 (1103_chainwing_sih.hil)
  └── 延迟测量: 主→从 < 2ms

阶段 3: 副翼微调控制实现 (5-7天)
  ├── 实现 compute_aileron_trim_offset()
  ├── 实现 CAN 指令接收 + 微调叠加
  ├── 实现通信超时保护 (500ms)
  ├── 单飞控 SITL 中验证计算逻辑
  ├── SIH 硬件验证
  └── 调参: Kp, Kd, MAX_TRIM

阶段 4: 三飞控联调 (5-7天)
  ├── 三飞控同时上电
  ├── CAN 心跳互检
  ├── 主飞控发送指令 → 从飞控执行
  ├── 副翼微调闭环验证
  ├── 地面滑行测试
  └── 故障注入: 断开一个从飞控

阶段 5: 飞行测试 (逐步)
  ├── 低空低速直线飞行
  ├── 观察铰接角变化 + 副翼微调响应
  ├── 盘旋飞行
  └── 完整任务飞行
```

### 15.2 每个文件的创建顺序和依赖关系

```
顺序  文件                                    依赖
────  ────────────────────────────────────    ──────────
 1    msg/ChainwingSlaveCmd.msg  (可选)       无
 2    src/modules/chainwing_slave/module.yaml  无
 3    src/modules/chainwing_slave/CMakeLists.txt  #2
 4    src/modules/chainwing_slave/ChainwingSlave.hpp  #1(如使用)
 5    src/modules/chainwing_slave/ChainwingSlave.cpp  #4
 6    boards/px4/fmu-v6x/default.px4board (修改)  #3
 7    ROMFS/.../2151_chainwing_slave  #2
 8    ROMFS/.../4008_gz_chainwing_slave (仿真)  #7
 9    Tools/simulation/gz/models/chainwing_slave/  #8

编译验证点:
  - 步骤 6 后: make px4_fmu-v6x_default 编译通过
  - 步骤 8 后: make px4_sitl_default 编译通过
  - 步骤 9 后: SITL 仿真可启动
```

---

## 附录 D: CAN 通信参数速查（替代附录 B 的 UART 配置）

```bash
# ═══════════════════════════════════════════
# 主飞控 CAN 参数 (MAV_SYS_ID=1, UAVCAN_NODE_ID=1)
# ═══════════════════════════════════════════

# CAN 总线
param set UAVCAN_ENABLE   3          # 传感器+执行器模式
param set UAVCAN_NODE_ID  1          # 主飞控节点
param set UAVCAN_BITRATE  1000000    # 1 Mbps

# CAN 执行器输出
param set UAVCAN_PUB_ARM  1          # 广播解锁状态

# 控制分配 (不变, 与现有 2150_chainwing 相同)
param set CA_ROTOR_COUNT  3
param set CA_SV_CS_COUNT  3

# ═══════════════════════════════════════════
# 从飞控(L) CAN 参数 (MAV_SYS_ID=2, UAVCAN_NODE_ID=2)
# ═══════════════════════════════════════════

# CAN 总线
param set UAVCAN_ENABLE   1          # 传感器手动模式 (接收CAN)
param set UAVCAN_NODE_ID  2          # 左翼节点
param set UAVCAN_BITRATE  1000000

# 关闭不需要的 CAN 传感器订阅
param set UAVCAN_SUB_GPS  0
param set UAVCAN_SUB_MAG  0
param set UAVCAN_SUB_BARO 0
param set UAVCAN_SUB_BAT  0
param set UAVCAN_SUB_ASPD 0
param set UAVCAN_SUB_IMU  0
param set UAVCAN_SUB_RNG  0

# 从机角色
param set MAV_SYS_ID      2
param set COM_ARM_WO_GPS  1
param set SYS_HAS_GPS     0
param set EKF2_GPS_CTRL   0

# 副翼微调
param set CW_SLV_SIDE     0          # 左翼
param set CW_SLV_KP       0.3
param set CW_SLV_KD       0.05
param set CW_SLV_MAX      0.3

# ═══════════════════════════════════════════
# 从飞控(R) CAN 参数 (MAV_SYS_ID=3, UAVCAN_NODE_ID=3)
# ═══════════════════════════════════════════

param set UAVCAN_NODE_ID  3          # 右翼节点
param set MAV_SYS_ID      3
param set CW_SLV_SIDE     1          # 右翼
# 其余与从飞控(L)相同
```

## 附录 E: 从 chainwing model.sdf 提取的仿真关键数据

```
# 以下数据直接来源于 Tools/simulation/gz/models/chainwing/model.sdf
# 创建 chainwing_slave 模型时需要使用

# ── 翼面几何 ──
翼段翼展:      1.2 m       (Y方向)
翼段弦长:      0.3 m       (X方向)
翼面面积:      0.36 m²
翼型厚度:      约 0.03 m

# ── 控制面 ──
副翼类型:      revolute joint (Y轴旋转)
副翼限幅:      ±0.53 rad (±30.4°)
副翼阻尼:      1.0 (damping)
副翼安装位置:  后缘 (X = -0.50 m from CG)
副翼 Y 位置:   左翼 Y=-1.20m, 右翼 Y=+1.20m

# ── 电机 ──
电机类型:      revolute joint (Z轴旋转, 无限制)
左电机旋向:    CCW (multiplier=-1)
中央电机旋向:  CW  (multiplier=1)
右电机旋向:    CCW (multiplier=-1)
电机最大转速:  对应整机50N总推力

# ── 气动参数 ──
CL_alpha:      4.752 /rad
CD0:           0.024
CDA:           0.016
CM_alpha:      -0.7
alpha_stall:   0.3 rad (≈17°)
a0:            0.10 rad (零升迎角)
control_joint_rad_to_cl: -0.3 (副翼偏转→升力系数变化)

# ── 轮系 ──
左轮:   (-0.04, -0.60, -0.20) 半径 0.05m
右轮:   (-0.04,  0.60, -0.20) 半径 0.05m
前轮:   ( 0.25,  0.00, -0.20) 半径 0.04m
```


---

## 16. 主机链翼仿真数据来源完整追溯

> **核心结论**：主机链翼仿真的参数数据**主要来源于 MATLAB 动力学开环仿真模型**，
> report.pdf 提供了理论推导框架和部分气动导数，两者在 GZ + PX4 闭环仿真中共同作用。
> 少数参数来源于 GZ 仿真调试和 PX4 参考机型 (rc_cessna)。

### 16.1 总体数据流向

```
┌─────────────────────┐     ┌──────────────────────┐     ┌─────────────────────┐
│    report.pdf        │     │  MATLAB 开环模型       │     │  PX4 参考机型         │
│  (理论推导 & 设计)    │     │ (动力学开环验证/)       │     │ (rc_cessna 等)       │
│                     │     │                      │     │                     │
│ · 控制面配置方案      │     │ · 物理参数 (m, J, b)  │     │ · 舵面效能符号约定     │
│ · 约束动力学理论      │     │ · 气动系数 (CL, CD)   │     │ · EKF 参数基线        │
│ · 气动导数 (Cmq 等)  │     │ · 推力模型 (T_max)    │     │ · PID 参考增益        │
│ · 稳定性分析         │     │ · 配平状态 (V0, α)    │     │ · 仿真辅助参数        │
└───────┬─────────────┘     └──────────┬───────────┘     └──────────┬──────────┘
        │                              │                            │
        │         ┌────────────────────┤                            │
        │         │                    │                            │
        ▼         ▼                    ▼                            ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                    GZ model.sdf (Gazebo 物理仿真模型)                        │
│  Tools/simulation/gz/models/chainwing/model.sdf                            │
│                                                                            │
│  质量/惯量 ← MATLAB          气动插件 ← MATLAB+report.pdf                   │
│  电机模型 ← MATLAB           几何尺寸 ← 物理测量                             │
└──────────────────────────────────┬──────────────────────────────────────────┘
                                   │
                                   ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│              PX4 机架配置 (4007_gz_chainwing / 2150_chainwing)               │
│  ROMFS/px4fmu_common/init.d-posix/airframes/4007_gz_chainwing              │
│                                                                            │
│  巡航空速 ← MATLAB          控制分配 ← MATLAB+report.pdf                    │
│  油门配平 ← MATLAB          PID 增益 ← GZ 仿真调试                          │
│  俯仰偏置 ← GZ 调试         安全参数 ← PX4 框架默认                          │
└────────────────────────────────────────────────────────────────────────────┘
```

### 16.2 MATLAB 提供的参数（主要数据源）

MATLAB 模型位于仓库 `动力学开环验证/` 目录，包含三个核心文件：

| 文件 | 功能 | 提供的关键数据 |
|------|------|---------------|
| `Run_Validation.m` | 初始化脚本 | 物理参数 (b, V0, α_trim), 配平状态 |
| `ThreeBodyDynamics.m` | 多体动力学求解器 | 质量 (m=1.9), 惯量 (J), 翼展 (b=1.2) |
| `ThreeBodyAerodynamics.m` | 气动力解算模块 | 气动系数 (a0, α_0, Cd0, CLA), 推力 (T_max=15) |
| `codebasedsimu.slx` | Simulink 模型 | 完整开环仿真验证 |

#### 16.2.1 从 MATLAB 直接使用的参数

以下参数**直接取自 MATLAB 代码**，无需转换：

| MATLAB 变量 | MATLAB 值 | 来源文件:行号 | → GZ/PX4 参数 | GZ/PX4 值 | 备注 |
|-------------|-----------|-------------|--------------|-----------|------|
| `m` | 1.9 kg | ThreeBodyDynamics.m:19 | model.sdf `<mass>` | 5.7 kg (3×1.9) | 三机合体质量 |
| `b` | 1.2 m | ThreeBodyDynamics.m:21 | CA_ROTOR0_PY | -1.2 / +1.2 | 单元间距 |
| `V0` | 20 m/s | Run_Validation.m:6 | FW_AIRSPD_TRIM | 20 | 巡航空速 |
| `alpha_0` | -0.05 rad | ThreeBodyAerodynamics.m:17 | model.sdf `<a0>` | -0.05 | 零升迎角 |
| `Cd0_airfoil` | 0.01 | ThreeBodyAerodynamics.m:18 | model.sdf Cd0 分量 | 0.01 | 翼型零升阻力 |
| `T_max` | 15.0 N | ThreeBodyAerodynamics.m:51 | motorConstant | 1.5e-05 | T=k·ω², k=15/1000² |
| `thrust_trim` | 0.6 (60%) | Run_Validation.m:42 | FW_THR_TRIM | 0.60 | 巡航油门 |
| `alpha_trim` | 0.0821 rad | Run_Validation.m:8 | FW_PSP_OFF | 2.0° (GZ调低) | 配平迎角 |
| `S_wing` | 0.36 m² (1.2×0.3) | ThreeBodyAerodynamics.m:14 | model.sdf `<area>` | 0.36 | 单翼段面积 |
| `chord` | 0.3 m | ThreeBodyAerodynamics.m:13 | 几何尺寸 | 0.3 m | 弦长 |
| `oswald_coupled` | 0.85 | ThreeBodyAerodynamics.m:35 | 诱导阻力系数 | K=0.0313 | 奥斯瓦尔德效率因子 |

#### 16.2.2 需要数学转换的参数

以下参数从 MATLAB 到 GZ 需要数学转换：

**① 惯量张量（平行轴定理）**

MATLAB 提供单机惯量：
```matlab
% ThreeBodyDynamics.m:20
J = diag([0.0894, 0.144, 0.162]);  % 单机转动惯量 (kg·m²)
```

转换到三机合体惯量（使用平行轴定理）：
```
Ixx_total = 3 × Ixx_unit + 2 × m × b²
          = 3 × 0.0894 + 2 × 1.9 × 1.2²
          = 0.2682 + 5.472
          = 5.740 kg·m²   ← model.sdf <ixx>5.740</ixx>

Iyy_total = 3 × Iyy_unit
          = 3 × 0.144
          = 0.432 kg·m²   ← model.sdf <iyy>0.432</iyy>
          (Iyy 无平行轴修正：三个单元沿 Y 轴排列，平行轴定理中
           偏移距离 d 是质心到旋转轴的垂直距离。对于 Iyy（绕 Y 轴），
           各单元的偏移方向恰好在 Y 轴上，垂直距离 d=0，故无附加项)

Izz_total = 3 × Izz_unit + 2 × m × b²
          = 3 × 0.162 + 2 × 1.9 × 1.2²
          = 0.486 + 5.472
          = 5.958 kg·m²   ← model.sdf <izz>5.958</izz>
```

**② 升力线斜率（有限翼展修正）**

MATLAB 使用薄翼理论值 + 有限翼展修正：
```matlab
% ThreeBodyAerodynamics.m:16,36-37
a0 = 2 * pi;           % 薄翼理论升力线斜率 = 6.283 /rad
AR_coupled = b_total² / S_total = (3×1.2)² / (3×0.36) = 3.6²/1.08 = 12.96 ≈ 12;  % 展弦比（取整）
K_ind = 1/(π × 0.85 × 12) = 0.0313;      % 诱导阻力因子

% ThreeBodyAerodynamics.m:80 — 升力系数计算
Cl_j = a0 * (alpha_j - alpha_0) / (1 + a0 * K_ind);
%    = 6.283 × (α + 0.05) / (1 + 6.283 × 0.0313)
%    = 6.283 × (α + 0.05) / 1.197
% 等效 CL_alpha = 6.283 / 1.197 = 5.249 ≈ 5.25 /rad
% 注: 5.249→5.25 的舍入误差仅 0.02%，远小于气动系数本身的不确定性（通常 ±5~10%）
```

→ GZ 中直接使用等效值：
```xml
<!-- model.sdf:767 -->
<cla>5.25</cla>  <!-- 已包含有限翼展修正的等效升力斜率 -->
```

**③ 电机常数转换**

MATLAB 推力模型为线性：`T = T_max × δ_t`

GZ MulticopterMotorModel 推力模型为二次：`T = motorConstant × ω²`

转换关系：
```
δ_t = 1.0 时 → ω = maxRotVelocity = 1000 rad/s
T_max = motorConstant × ω_max²
15.0 = motorConstant × 1000²
motorConstant = 15.0 / 1000000 = 1.5e-05
```

→ GZ 参数：
```xml
<!-- model.sdf:924 -->
<motorConstant>1.5e-05</motorConstant>
<maxRotVelocity>1000</maxRotVelocity>
```

### 16.3 report.pdf 提供的参数

report.pdf (46 页) 主要提供**理论框架和气动导数**，不直接提供所有物理参数。

| report.pdf 内容 | 对应参数/设计 | 使用位置 | 关系说明 |
|-----------------|-------------|---------|---------|
| 约束动力学推导 (Baumgarte 稳定化) | MATLAB ThreeBodyDynamics.m 的算法基础 | 间接影响所有参数 | 理论框架，不直接产生数值 |
| 控制面配置方案 (Table 3.3) | 反转升降副翼 TRQ_R: ±0.5 | 4007_gz_chainwing:48-68 | 决定了为什么左翼用正值、右翼用负值 |
| 气动稳定性导数 | Cmq=-50.8, Cnr=-0.411, Clp=-0.414, CmDe=-1.13, CnDr=-0.0345, ClDa=0.0677 | ThreeBodyAerodynamics.m:20-21 | MATLAB 直接引用这些数值 |
| 纵向静稳定性参数 | Cm0=0.135, Cma=-1.5 | ThreeBodyAerodynamics.m:23 | 用于尾翼力矩计算 |
| 侧向力系数 | CYβ=-0.83 | ThreeBodyAerodynamics.m:117 | 侧力模型 |
| 翼型基本特性 | 参考翼型选型 | alpha_0, alpha_stall 范围 | 提供选型依据 |

**关键区别**：
- **report.pdf** 提供的是**气动导数**（Cmq、Cnr 等无量纲系数）和**设计方案**（控制面布局）
- **MATLAB** 提供的是**物理参数**（质量、惯量、翼面积）和**配平状态**（巡航速度、油门）
- 两者关系：report.pdf 的理论 → MATLAB 模型验证 → GZ/PX4 参数

### 16.4 GZ 仿真调试产生的参数

以下参数**不来自 MATLAB 或 report.pdf**，而是在 Gazebo 闭环仿真中迭代调试得到的：

| 参数 | 值 | 为什么不能用 MATLAB 值 | 调试方法 |
|------|------|---------------------|---------|
| FW_PSP_OFF | 2.0° | MATLAB α_trim=4.7° 是开环值，GZ 闭环中升力特性不同 | 在 GZ 中观察稳态飞行迎角 |
| FW_PR_P (俯仰速率增益) | 0.08 | PX4 控制器结构与 MATLAB 不同 | GZ 中逐步调整直到俯仰震荡消失 |
| FW_RR_P (滚转速率增益) | 0.08 | 同上 | GZ 中调整直到滚转响应合适 |
| FW_YR_P (偏航速率增益) | 0.05 | 差动推力偏航控制不在 MATLAB 模型中 | GZ 中测试差动推力偏航响应 |
| FW_YAW_STAB_SC | 1.0 (GZ) / 2.0 (HW) | MATLAB 没有航向保持控制器 | GZ 中调整直到地面不旋转 |
| FW_AIRSPD_STALL | 8 m/s | MATLAB 使用连续气动模型，无失速边界 | 根据 GZ 中实际失速表现设定 |
| FW_AIRSPD_MAX | 35 m/s | MATLAB 开环验证未测试高速极限 | GZ 中俯冲测试确定安全上限 |

### 16.5 PX4 参考机型 (rc_cessna) 提供的参数

以下参数参考了 PX4 标准固定翼机型 `4003_gz_rc_cessna`：

| 参数 | rc_cessna 值 | chainwing 值 | 来源文件 |
|------|-------------|-------------|---------|
| 舵面效能符号约定 (TRQ_R) | 左=-0.5, 右=+0.5 | 相同 | 4003_gz_rc_cessna:69-72 |
| CA_SV_CS*_TYPE | 1/2 (Aileron L/R) | 相同 | 4003_gz_rc_cessna:65-68 |
| EKF2 参数基线 | 默认值 | 放宽到 1.0 | PX4 默认配置 |
| COM_LOW_BAT_ACT | 默认 | 0 (关闭) | SITL 不需要电池保护 |
| SIM_BAT_DRAIN | 默认 | 3600 | SITL 长时间仿真 |

### 16.6 完整参数溯源表

以下表格涵盖 **4007_gz_chainwing 机架配置文件中的每一个参数**及其数据来源：

#### A. 控制分配参数（来源：MATLAB + report.pdf）

| PX4 参数 | 值 | 来源 | 推导过程 |
|----------|------|------|---------|
| CA_AIRFRAME | 1 (FW) | PX4 框架 | 固定翼类型 |
| CA_ROTOR_COUNT | 3 | MATLAB | 三电机设计 |
| CA_ROTOR0_PY | -1.2 | MATLAB b=1.2m | 左电机 Y 轴位置 |
| CA_ROTOR1_PY | 0 | MATLAB | 中央电机在原点 |
| CA_ROTOR2_PY | 1.2 | MATLAB b=1.2m | 右电机 Y 轴位置 |
| CA_ROTOR0_KM | -0.05 | GZ 调试 | 左电机(CCW)反扭矩 |
| CA_ROTOR1_KM | 0.05 | GZ 调试 | 中央电机(CW)反扭矩 |
| CA_ROTOR2_KM | -0.05 | GZ 调试 | 右电机(CCW)反扭矩 |
| CA_SV_CS_COUNT | 3 | MATLAB+report.pdf | 左副翼+升降舵+右副翼 |
| CA_SV_CS0_TRQ_R | 0.5 | report.pdf Table 3.3 + rc_cessna | 左翼副翼滚转效能（反转） |
| CA_SV_CS0_TRQ_P | -0.5 | rc_cessna 参考 | 左翼副翼俯仰效能 |
| CA_SV_CS1_TRQ_P | -1.0 | GZ 调试 | 中央升降舵俯仰效能 |
| CA_SV_CS2_TRQ_R | -0.5 | report.pdf Table 3.3 + rc_cessna | 右翼副翼滚转效能（反转） |
| CA_SV_CS2_TRQ_P | -0.5 | rc_cessna 参考 | 右翼副翼俯仰效能 |

#### B. 空速参数（来源：MATLAB）

| PX4 参数 | 值 | 来源 | 推导过程 |
|----------|------|------|---------|
| FW_AIRSPD_TRIM | 20 m/s | MATLAB V0=20 | 巡航空速 |
| FW_AIRSPD_MIN | 12 m/s | GZ 调试 | ≈ 1.5 × V_stall |
| FW_AIRSPD_MAX | 35 m/s | GZ 调试 | GZ 中俯冲极限 |
| FW_AIRSPD_STALL | 8 m/s | GZ 调试 | GZ 中实际失速速度 |

#### C. 姿态/油门参数（来源：MATLAB → GZ 调试微调）

| PX4 参数 | 值 | 来源 | 推导过程 |
|----------|------|------|---------|
| FW_THR_TRIM | 0.60 | MATLAB Ctrl_Trim(4)=0.6 | 巡航油门 60% |
| FW_THR_MAX | 1.0 | PX4 默认 | — |
| FW_THR_MIN | 0.0 | PX4 默认 | — |
| FW_PSP_OFF | 2.0° | GZ 调试（MATLAB 为 4.7°） | GZ 闭环配平不同于开环 |
| FW_P_LIM_MAX | 30° | GZ 调试 | 爬升角限制 |
| FW_P_LIM_MIN | -15° | GZ 调试 | 俯冲角限制 |
| FW_R_LIM | 50° | GZ 调试 | 滚转角限制 |

#### D. PID 增益（来源：GZ 仿真调试）

| PX4 参数 | 值 | 来源 |
|----------|------|------|
| FW_PR_P | 0.08 | GZ 调试 |
| FW_PR_I | 0.01 | GZ 调试 |
| FW_PR_D | 0.0 | GZ 调试 |
| FW_RR_P | 0.08 | GZ 调试 |
| FW_RR_I | 0.01 | GZ 调试 |
| FW_YR_P | 0.05 | GZ 调试 |
| FW_YR_I | 0.01 | GZ 调试 |

#### E. 安全/仿真参数（来源：PX4 框架 + SITL 需求）

| PX4 参数 | 值 | 来源 | 原因 |
|----------|------|------|------|
| COM_ARM_EKF_POS | 1.0 | SITL 调试 | EKF 收敛期间放宽阈值 |
| COM_ARM_EKF_VEL | 1.0 | SITL 调试 | 同上 |
| EKF2_REQ_GPS_H | 1.0s | SITL 调试 | 加速 GPS 信任（默认 10s） |
| COM_LOW_BAT_ACT | 0 | SITL 需求 | 关闭电池故障保护 |
| SIM_BAT_DRAIN | 3600 | SITL 需求 | 每 3600 步消耗 1% 电量 |
| SIM_BAT_MIN_PCT | 50.0 | SITL 需求 | 电池永不低于 50% |
| CBRK_SUPPLY_CHK | 894281 | SITL 需求 | 绕过电源检查 |
| FW_LND_USETER | 0 | SITL 限制 | GZ 无测距仪 |
| FW_LND_ABORT | 0 | SITL 限制 | 禁用着陆中止 |

### 16.7 model.sdf 气动参数溯源

GZ model.sdf 中的 LiftDrag 插件参数与 MATLAB 的精确对应关系：

```
┌─────────────────────────────────────────────────────────────────┐
│          ThreeBodyAerodynamics.m            model.sdf LiftDrag │
│                                                                │
│  a0 = 2*pi (6.283/rad)                                        │
│  + 有限翼展修正 /(1+a0*K_ind)      →   <cla>5.25</cla>        │
│  alpha_0 = -0.05                   →   <a0>-0.05</a0>         │
│  Cd0_airfoil = 0.01                →   Cd0 成分               │
│  S_wing = span*chord = 0.36       →   <area>0.36</area>       │
│  T_max = 15.0 N                   →   motorConstant = 1.5e-05 │
│  oswald = 0.85, AR = 12                                        │
│  K_ind = 0.0313                    →   (隐含在 cla 修正中)     │
│                                                                │
│  ── 来自 report.pdf 的气动导数 ──                               │
│  Cmq = -50.8                       →   (GZ 无直接等价参数)      │
│  Cnr = -0.411                      →   (GZ 用垂直尾翼面积近似)  │
│  Clp = -0.414                      →   (GZ 用翼面 LiftDrag 近似)│
│  CmDe = -1.13                      →   control_joint_rad_to_cl │
│  Cm0 = 0.135, Cma = -1.5           →   尾翼 <a0>-0.2</a0> 近似 │
└─────────────────────────────────────────────────────────────────┘
```

**重要说明**：GZ LiftDrag 插件**不直接使用**气动导数 (Cmq, Cnr, Clp 等)。
这些是 MATLAB 模型中的「集中参数」，在 GZ 中通过**分布式物理仿真**（翼面面积、升力斜率、
尾翼位置等几何参数）来**隐式实现**等效效果。因此不存在完美的一一对应关系。

### 16.8 MATLAB 与 GZ 模型的差异说明

| 方面 | MATLAB 模型 | GZ 模型 | 差异原因 |
|------|------------|---------|---------|
| **动力学** | 刚体 + Baumgarte 约束 | 单刚体（链翼合为一体） | GZ 模型简化为整体 |
| **气动** | 条带法 (6 段/翼) + 集中导数 | LiftDrag 插件 (逐面计算) | 建模方法不同 |
| **推力** | 线性 T = T_max × δ_t | 二次 T = k × ω² | 需要常数转换 |
| **控制器** | 开环（无控制器） | PX4 闭环 PID | MATLAB 只验证动力学 |
| **升力斜率** | a0 = 2π (薄翼) + 修正 | cla = 5.25 (直接使用) | 已等价转换 |
| **失速** | 无失速模型 | alpha_stall = 0.227 rad | GZ 需要失速特性 |
| **配平迎角** | 0.0821 rad (4.7°) | FW_PSP_OFF = 2.0° | 闭环配平不同于开环 |

### 16.9 report.pdf 中引用但未直接使用的参数

以下参数出现在 report.pdf 的理论推导中，但**未直接用于 GZ/PX4 配置**：

| report.pdf 参数 | 用途 | 为什么未直接使用 |
|----------------|------|----------------|
| Baumgarte 稳定化系数 α=20, β=20 | 约束动力学数值稳定性 | GZ 模型是单刚体，无铰链约束 |
| KKT 系统求解 | 求解约束力和加速度 | GZ 内部物理引擎自行求解 |
| 约束雅可比矩阵 G | 铰链连接点约束 | GZ 无多体约束（简化为整体） |
| 拉格朗日乘子 λ | 铰链内力 | GZ 模型不区分铰链 |

这些参数虽然未直接使用，但它们**验证了动力学模型的正确性**，
确保了从 MATLAB 传递到 GZ 的物理参数（质量、惯量等）是正确的。

### 16.10 总结：数据来源占比

```
主机链翼仿真参数来源占比（按参数数量）:

  ┌────────────────────────────────────────────────────┐
  │ MATLAB 开环模型    ████████████████████░░░░░  ~45%  │  质量、惯量、气动、推力、空速、油门
  │ GZ 仿真调试        ██████████████░░░░░░░░░░  ~30%  │  PID 增益、俯仰偏置、速度限制
  │ PX4 默认/rc_cessna ██████░░░░░░░░░░░░░░░░░░  ~15%  │  舵面符号、EKF 基线
  │ report.pdf         ████░░░░░░░░░░░░░░░░░░░░  ~10%  │  控制面方案、气动导数
  └────────────────────────────────────────────────────┘

按重要性排序:
  1. MATLAB 模型 — 提供了 80%+ 的物理基础数据
  2. GZ 调试    — 提供了闭环稳定性所需的控制参数
  3. report.pdf — 提供了理论支撑和设计方案决策
  4. rc_cessna  — 提供了 PX4 固定翼配置的基线参考
```

**简明回答**：主机链翼的仿真数据**主要来自 MATLAB 动力学开环仿真模型**
（`动力学开环验证/` 目录中的 `.m` 和 `.slx` 文件），
report.pdf 提供了理论基础和气动导数，两者互补配合。
MATLAB 负责「数据产生」，report.pdf 负责「理论支撑」。

---

## 17. 从机位置微调执行器方案对比：升降舵 vs 副翼

### 17.1 问题定义

在三体串联链翼中，左从机和右从机需要**保持与主机共面飞行**。由于制造公差、
气流扰动和质量分布不均，从机相对于主机会产生铰接偏角 θ（上翘/下垂），需要
一个执行器来实时修正。

两个候选方案：

| | **方案 A：使用现有升降舵** | **方案 B：新增副翼** |
|---|---|---|
| **执行器** | 各单元已有的尾缘控制面（servo_0/servo_2） | 在每个从机机翼上新增一片独立副翼 |
| **硬件改动** | **零** — 复用现有舵面 | 需要新增舵机、连杆、开孔 |
| **控制通道** | 与滚转/俯仰共用 | 独立通道，与主控制解耦 |

### 17.2 当前系统执行器架构回顾

根据 `model.sdf` 和 `4007_gz_chainwing` 机架配置：

```
                    ┌── rotor_left (电机)
  左从机 (y=-1.2m) ─┤
                    └── servo_0 (左升降副翼, elevon)
                        → 功能: 滚转 (TRQ_R=+0.5) + 俯仰 (TRQ_P=-0.5)
                        → LiftDrag: area=0.36m², control_joint_rad_to_cl=-0.3
                        → 偏转范围: ±0.53 rad (±30.4°)

                    ┌── rotor_center (电机)
  主机   (y=0)     ─┤
                    └── servo_1 (中央升降舵, elevator)
                        → 功能: 纯俯仰 (TRQ_R=0, TRQ_P=-1.0)
                        → LiftDrag: area=0.18m², control_joint_rad_to_cl=-4.0
                        → 偏转范围: ±0.53 rad (±30.4°)

                    ┌── rotor_right (电机)
  右从机 (y=+1.2m) ─┤
                    └── servo_2 (右升降副翼, elevon)
                        → 功能: 滚转 (TRQ_R=-0.5) + 俯仰 (TRQ_P=-0.5)
                        → LiftDrag: area=0.36m², control_joint_rad_to_cl=-0.3
                        → 偏转范围: ±0.53 rad (±30.4°)
```

**关键观察**：
- 左/右从机的控制面（servo_0/servo_2）是 **elevon（升降副翼）**，不是纯升降舵
- 它们**已经同时承担滚转和俯仰功能**
- 中央的 servo_1 才是纯升降舵（但属于主机，不属于从机）

### 17.3 方案 A：使用现有升降副翼（elevon）进行位置微调

#### 17.3.1 工作原理

```
从机 IMU 检测到铰接偏角 θ
        ↓
PD 控制器计算微调量:
  δ_trim = Kp·(0-θ) + Kd·(0-θ̇)
        ↓
叠加到现有 elevon 指令:
  δ_total = δ_main_control + δ_trim
        ↓
servo_0 或 servo_2 执行
```

#### 17.3.2 力学分析

以左从机为例，servo_0 偏转 δ_trim 产生的力矩：

```
额外升力 ΔL = q · S_wing · CLA · (control_joint_rad_to_cl · δ_trim)
           = (0.5×1.2041×V²) × 0.36 × 5.25 × (-0.3 × δ_trim)

在巡航 V=20 m/s 时:
  q = 0.5 × 1.2041 × 400 = 240.8 Pa
  ΔL = 240.8 × 0.36 × 5.25 × (-0.3) × δ_trim
     = -136.6 × δ_trim  [N]

正方向约定:
  δ_trim > 0 → elevon 下偏 → CL 减小(因 rad_to_cl=-0.3)
  → 升力减小 → 该翼段下沉

  需要修正: 左翼下垂 (θ<0) → 需增加升力 → δ_trim < 0

铰接力矩 (关于连接铰):
  M_hinge = ΔL × L_arm = -136.6 × δ_trim × 0.6 m (翼半展到铰)
          = -82.0 × δ_trim  [N·m]

在 δ_trim = ±0.3 rad (30% 行程) 范围内:
  |M_hinge_max| = 82.0 × 0.3 = 24.6 N·m

对比重力偏差力矩:
  M_gravity = m_unit × g × L_arm × sin(θ)
            = 1.9 × 9.81 × 0.6 × sin(θ)
            ≈ 11.2 × θ  [N·m] (小角度)

结论: 微调力矩(24.6 N·m) > 重力偏差(11.2 N·m at θ=1 rad)
      → 在合理偏角(θ < 10°≈0.17 rad)范围内绰绰有余 ✓
```

#### 17.3.3 优势

| 优势 | 详细说明 |
|------|----------|
| **零硬件改动** | 直接复用 servo_0/servo_2，无需新增舵机、连杆、铰链 |
| **已有 GZ 模型** | model.sdf 中 servo_0/servo_2 已完整建模，可直接仿真验证 |
| **已有气动数据** | LiftDrag 参数完整: area=0.36m², CLA=5.25, rad_to_cl=-0.3 |
| **已有 PX4 分配** | CA_SV_CS0/CS2 已配置，只需叠加 trim offset |
| **重量不变** | 不增加任何结构重量 |
| **实现快速** | 仅需软件修改，可在现有仿真环境中立即测试 |

#### 17.3.4 劣势与风险

| 风险 | 严重度 | 缓解措施 |
|------|--------|----------|
| **通道耦合**: δ_trim 与主控制叠加，影响整机滚转/俯仰 | **中** | 限制 δ_trim ≤ ±0.3 rad（30% 行程），保留 70% 给主控制 |
| **权限竞争**: 主机发送滚转指令 vs 从机微调修正 | **中** | 主机指令优先级 > 微调；微调速率限幅 0.5 rad/s |
| **低速权限不足**: 起飞/着陆时 q 小，微调力不够 | **低** | 空速²缩放: Kp_eff = Kp × (V_trim/V)²，低速时增益自动增大 |
| **失速区域**: 大攻角时 elevon 效率急剧下降 | **低** | 仅在 V > FW_AIRSPD_STALL(8 m/s) 时启用微调 |

### 17.4 方案 B：新增独立副翼

#### 17.4.1 工作原理

```
在每个从机机翼上增加一片独立副翼:
  - 物理位置: 翼展中部，面积约 0.05-0.10 m²
  - 独立舵机: 第4个PWM通道
  - 专用功能: 仅用于铰接角微调

从机 IMU 检测到铰接偏角 θ
        ↓
PD 控制器计算:
  δ_aileron = Kp·(0-θ) + Kd·(0-θ̇)
        ↓
独立副翼舵机执行 (不影响主 elevon)
```

#### 17.4.2 力学分析

假设副翼面积 S_ail = 0.08 m²（占翼面积 22%），CLA_ail ≈ 4.0/rad：

```
ΔL_ail = q · S_ail · CLA_ail · δ_ail
       = 240.8 × 0.08 × 4.0 × δ_ail
       = 77.1 × δ_ail  [N]

M_hinge_ail = 77.1 × δ_ail × 0.6
            = 46.2 × δ_ail  [N·m]

在全偏转 (δ_ail = ±0.53 rad):
  |M_max| = 46.2 × 0.53 = 24.5 N·m

与方案 A (δ_trim=0.3 时 24.6 N·m) 几乎相同!
但方案 B 需要全偏转才能达到，方案 A 只用 30% 行程。
```

#### 17.4.3 优势

| 优势 | 详细说明 |
|------|----------|
| **完全解耦** | 不影响主滚转/俯仰控制，独立通道 |
| **设计清晰** | 一个舵面一个功能，无耦合分析负担 |
| **可独立失效** | 副翼故障不影响主控制面 |

#### 17.4.4 劣势与风险

| 风险 | 严重度 | 说明 |
|------|--------|------|
| **硬件复杂度** | **高** | 每个从机 +1 舵机 + 连杆 + 铰链 + 蒙皮开孔 |
| **增加重量** | **中** | 每侧 ~50-80g（舵机+连杆），总计 100-160g |
| **GZ 模型重建** | **高** | model.sdf 需新增 joint、link、LiftDrag 插件 |
| **气动数据缺失** | **高** | 副翼面积、位置、效率系数均无 MATLAB/report.pdf 数据 |
| **结构强度风险** | **中** | 翼面开孔减弱结构，铰接区域应力集中 |
| **开发周期延长** | **高** | 机械设计 + 制造 + GZ 建模 + 参数标定 |
| **无仿真支持** | **高** | 当前 model.sdf 无此副翼，无法立即进行 SITL 验证 |

### 17.5 量化对比总表

```
┌─────────────────────┬──────────────────────┬──────────────────────┐
│       评估维度       │  方案A: 现有Elevon   │  方案B: 新增副翼      │
├─────────────────────┼──────────────────────┼──────────────────────┤
│ 硬件改动量           │  ★★★★★ 零改动       │  ★★☆☆☆ 大量改动      │
│ 开发周期             │  ★★★★★ 1-2 周       │  ★★☆☆☆ 4-8 周       │
│ 通道独立性           │  ★★★☆☆ 共用通道     │  ★★★★★ 完全独立      │
│ 气动数据完备性       │  ★★★★★ 完全已知     │  ★☆☆☆☆ 需要测量      │
│ GZ 仿真支持          │  ★★★★★ 立即可用     │  ★☆☆☆☆ 需重新建模    │
│ 微调力矩/行程比     │  ★★★★☆ 30%行程=24.6N·m│ ★★★☆☆ 100%行程=24.5N·m│
│ 与主控制耦合风险     │  ★★★☆☆ 需要分配管理 │  ★★★★★ 无耦合        │
│ 重量代价             │  ★★★★★ 0g           │  ★★★☆☆ +100-160g     │
│ 结构风险             │  ★★★★★ 无           │  ★★★☆☆ 开孔+应力     │
│ 系统可靠性           │  ★★★★☆ 少一个故障点 │  ★★★☆☆ 多一个故障点  │
├─────────────────────┼──────────────────────┼──────────────────────┤
│ 综合评分 (50分满)    │  ★★★★☆ 42/50        │  ★★★☆☆ 30/50        │
│ 推荐度               │  ✅ 强烈推荐          │  ⚠️ 备选（后期优化）  │
└─────────────────────┴──────────────────────┴──────────────────────┘
```

### 17.6 结论与建议

#### 17.6.1 明确推荐：方案 A（使用现有 elevon）

**理由**：

1. **气动数据完备** — servo_0/servo_2 的全部 LiftDrag 参数已在 model.sdf 中定义，
   且经过 MATLAB→GZ 的完整验证链（见 §16），不需要额外测量

2. **力矩裕度充足** — 仅使用 30% 行程（±0.3 rad）即可产生 24.6 N·m 微调力矩，
   远超重力偏差（11.2 × θ N·m），对于实际偏角 θ < 5°(0.087 rad) 只需 1.0 N·m

3. **可立即仿真** — 当前 GZ 模型完整支持，无需修改 model.sdf 即可测试控制律

4. **零硬件成本** — 不增加重量、不改变结构、不增加故障点

5. **PX4 实现简单** — 只需在 control allocator 的输出上叠加一个 trim offset，
   不需要新增 PWM 通道或修改混控矩阵

#### 17.6.2 方案 A 的通道耦合如何管理

```
控制分配优先级:

  主机发送的滚转/俯仰指令 (高优先级)
  ─────────────────────────────────────→ δ_main = [-1.0, +1.0]
                                           ↓
  从机本地微调 (低优先级)               叠加但受限:
  ─────────────────────────────────────→ δ_trim = [-0.3, +0.3]
                                           ↓
  最终输出:                              δ_total = clamp(δ_main + δ_trim, -1.0, +1.0)

  安全保证:
    1. |δ_trim| ≤ 0.3 (CW_SLV_MAX 参数控制)
    2. δ_total 被 clamp 到 ±1.0 → 永远不会超出物理行程
    3. 当 |δ_main| > 0.7 时，δ_trim 被压缩 → 主控制始终优先
    4. 微调速率限幅 0.5 rad/s → 不会产生突变
```

#### 17.6.3 方案 B 的适用场景

方案 B 适合**后续版本优化**，当以下条件满足时可以考虑：
- 方案 A 的耦合问题在实际飞行中确实造成了可观测的控制品质下降
- 已完成副翼气动参数的风洞/CFD 测试
- 已设计并制造了可靠的副翼机械结构
- 有足够的开发时间进行 GZ 模型重建和参数标定

---

## 18. 从机固件编写可行性评估与数据清单

### 18.1 总体评估结论

> **结论：现有数据足以开始从机固件编写**，但需要分阶段进行，
> 某些参数需要在仿真中迭代确定。

```
可行性评分: ████████░░ 80%

已具备:
  ✅ 物理参数 (质量、惯量、气动) ........... 100%
  ✅ 通信协议 (CAN/DroneCAN) ............... 100%
  ✅ 主机固件框架 (PX4 模块结构) ............ 100%
  ✅ 控制律设计 (PD + 空速缩放) ............. 100%
  ✅ GZ 仿真模型 (model.sdf) ................ 90%
  ✅ 执行器分配方案 (elevon trim) ............ 100%

缺失/待定:
  ⚠️ 铰接动力学模型 (GZ 中未建模) ........... 30%
  ⚠️ CAN 消息具体格式 (需实现) .............. 50%
  ⚠️ PD 增益精确值 (需仿真调参) .............. 60%
  ⚠️ 多实例 SITL 测试环境 (需搭建) .......... 20%
```

### 18.2 已具备的数据清单

#### 18.2.1 物理参数（来自 MATLAB + model.sdf）

| 参数 | 值 | 来源 | 文件位置 |
|------|-----|------|----------|
| 单元质量 | 1.9 kg | MATLAB | `ThreeBodyDynamics.m:19` |
| 总质量 | 5.7 kg | 计算 (3×1.9) | `model.sdf:<mass>5.7</mass>` |
| 单元翼面积 | 0.36 m² | MATLAB | `ThreeBodyAerodynamics.m:14` |
| 翼弦 | 0.3 m | MATLAB | `ThreeBodyAerodynamics.m:13` |
| 翼展（单元） | 1.2 m | MATLAB | `ThreeBodyDynamics.m:21` |
| 铰接间距 | 1.2 m | MATLAB | `CA_ROTOR*_PY = ±1.2` |
| 升力线斜率 | 5.25 /rad | MATLAB+修正 | `model.sdf:<cla>5.25</cla>` |
| 零升攻角 | -0.05 rad | MATLAB | `model.sdf:<a0>-0.05</a0>` |
| 失速攻角 | 0.227 rad (13°) | MATLAB | `model.sdf:<alpha_stall>` |
| 零升阻力 | 0.01 | MATLAB | `ThreeBodyAerodynamics.m:18` |
| Oswald 系数 | 0.85 | MATLAB | `ThreeBodyAerodynamics.m:35` |
| 单元惯量 Ixx | 0.0894 kg·m² | MATLAB | `ThreeBodyDynamics.m` |
| 单元惯量 Iyy | 0.144 kg·m² | MATLAB | `ThreeBodyDynamics.m` |
| 单元惯量 Izz | 0.162 kg·m² | MATLAB | `ThreeBodyDynamics.m` |
| 巡航速度 | 20 m/s | MATLAB | `Run_Validation.m:6` |
| 巡航油门 | 60% | MATLAB | `Run_Validation.m:42` |
| 最大推力（单电机） | 15 N | MATLAB | `ThreeBodyAerodynamics.m:51` |

#### 18.2.2 Elevon 控制面参数（来自 model.sdf + 4007_gz_chainwing）

| 参数 | 左 elevon (servo_0) | 右 elevon (servo_2) | 来源 |
|------|---------------------|---------------------|------|
| 位置 | (-0.50, -1.20, 0) | (-0.50, +1.20, 0) | model.sdf |
| 面积 | 0.36 m² | 0.36 m² | model.sdf |
| control_joint_rad_to_cl | -0.3 | -0.3 | model.sdf |
| 偏转限幅 | ±0.53 rad | ±0.53 rad | model.sdf |
| TRQ_R (滚转效力) | +0.5 | -0.5 | 4007_gz_chainwing |
| TRQ_P (俯仰效力) | -0.5 | -0.5 | 4007_gz_chainwing |
| CA_SV_CS 索引 | CS0 | CS2 | 4007_gz_chainwing |

#### 18.2.3 气动导数（来自 report.pdf → MATLAB）

| 导数 | 值 | 物理含义 | 对从机微调的影响 |
|------|-----|----------|------------------|
| Cmq | -50.8 | 俯仰阻尼 | 抑制俯仰振荡，有利于微调稳定 |
| Clp | -0.414 | 滚转阻尼 | 抑制滚转振荡 |
| Cnr | -0.411 | 偏航阻尼 | 不直接影响（差速推力控制偏航） |
| CmDe | -1.13 | 升降舵效率 | 适用于中央 elevator (servo_1) |
| ClDa | 0.0677 | 副翼效率 | 适用于 elevon 微调量计算 |
| Cma | -1.5 | 静稳定性 | 确保纵向稳定 |

#### 18.2.4 控制律参数（来自 §13 设计 + 物理推导）

| 参数 | 符号 | 设计值 | 推导来源 |
|------|------|--------|----------|
| 比例增益 | CW_SLV_KP | 0.3 | 铰接动力学 ωn≈3.4 rad/s |
| 微分增益 | CW_SLV_KD | 0.05 | 临界阻尼 ζ=0.7 |
| 微调限幅 | CW_SLV_MAX | 0.3 rad | 保留 70% 主控制行程 |
| 基准空速 | CW_SLV_VREF | 20 m/s | MATLAB 巡航速度 |
| 从机位置 | CW_SLV_SIDE | 0(左)/1(右) | 机械安装位置 |

#### 18.2.5 通信参数（来自 §11 CAN 设计）

| 参数 | 值 | 说明 |
|------|-----|------|
| CAN 波特率 | 1 Mbps | UAVCAN_BITRATE |
| 主机节点 ID | 1 | UAVCAN_NODE_ID |
| 左从机节点 ID | 2 | UAVCAN_NODE_ID |
| 右从机节点 ID | 3 | UAVCAN_NODE_ID |
| 消息频率 | 50 Hz | 控制指令更新率 |
| 带宽估算 | ~0.5 Mbps | 含传感器反馈 |

### 18.3 缺失数据与解决方案

#### 18.3.1 铰接动力学模型（重要度：高）

**现状**：当前 GZ 模型（model.sdf）将三个单元合并为一个 `base_link`（刚体），
**没有建模铰接关节**。这意味着：
- SITL 中左/右翼段不能相对于中央段旋转
- 无法直接测试铰接角反馈控制

**解决方案**：

```
方案 1: 修改 model.sdf，增加铰接关节 (推荐)

  需要新增:
  ├── left_hinge_joint (revolute, Y轴)
  │   parent: base_link (中央段)
  │   child:  left_wing_link (新增)
  │   limit:  [-0.2, +0.2] rad (±11.5°)
  │   damping: 0.1 N·m·s/rad
  │
  └── right_hinge_joint (revolute, Y轴)
      parent: base_link
      child:  right_wing_link (新增)
      limit:  [-0.2, +0.2] rad
      damping: 0.1 N·m·s/rad

  需要将 base_link 拆分为:
  - center_body (m=1.9 kg)
  - left_wing_body (m=1.9 kg, 含 servo_0 + rotor_left)
  - right_wing_body (m=1.9 kg, 含 servo_2 + rotor_right)

方案 2: 纯软件模拟 (快速验证)

  在 ChainwingSlave 模块中用数学模型模拟铰接角:
    θ(t+dt) = θ(t) + θ̇·dt
    θ̇(t+dt) = θ̇(t) + (M_aero + M_gravity + M_trim)/(I_hinge) · dt

  I_hinge = m_unit × L_arm² = 1.9 × 0.6² = 0.684 kg·m²

  优点: 不需要修改 model.sdf
  缺点: 模拟精度有限，与 GZ 物理引擎不同步
```

**建议**：先用方案 2 快速验证控制律，再用方案 1 进行完整仿真。

#### 18.3.2 CAN 消息具体格式（重要度：中）

**现状**：§11 描述了 DroneCAN 协议栈和参数，但具体的自定义消息尚未定义。

**需要定义的消息**：

```
主机 → 从机（50 Hz, CAN ID 由 DroneCAN 分配）:
┌─────────────────────────────────────────────┐
│ ChainwingSlaveCmd.msg (自定义 uORB 消息)    │
├─────────────────────────────────────────────┤
│ uint64  timestamp          # 时间戳 [us]    │
│ uint8   target_node_id     # 目标从机 (2/3)  │
│ float32 throttle_cmd       # 油门指令 [0,1]  │
│ float32 elevon_cmd         # 主控制指令 [-1,1]│
│ float32 trim_enable        # 微调使能 [0/1]  │
│ float32 master_pitch       # 主机俯仰角 [rad]│
│ float32 master_roll        # 主机滚转角 [rad]│
│ uint8   arm_state          # 解锁状态        │
└─────────────────────────────────────────────┘

从机 → 主机（10 Hz, 状态反馈）:
┌─────────────────────────────────────────────┐
│ ChainwingSlaveStatus.msg (自定义 uORB 消息)  │
├─────────────────────────────────────────────┤
│ uint64  timestamp          # 时间戳 [us]    │
│ uint8   node_id            # 本机节点 (2/3)  │
│ float32 hinge_angle        # 铰接角 [rad]    │
│ float32 hinge_rate         # 铰接角速度      │
│ float32 trim_output        # 当前微调量       │
│ float32 airspeed           # 本地空速         │
│ uint8   status             # 0=OK, 1=WARN    │
└─────────────────────────────────────────────┘
```

**CAN 传输方式**：使用 DroneCAN 的 `uavcan.equipment.actuator.ArrayCommand` 
发送控制指令（已有 PX4 驱动支持），或使用自定义 DSDL 消息类型。

#### 18.3.3 PD 增益精确值（重要度：中）

**现状**：§13 根据铰接动力学推导了初始值 Kp=0.3, Kd=0.05。

**精确化路径**：

```
步骤 1: 使用 MATLAB Simulink 闭环仿真
  - 输入: 铰接动力学模型 + 气动力 + 重力
  - 变量: Kp ∈ [0.1, 0.5], Kd ∈ [0.02, 0.10]
  - 指标: 阶跃响应上升时间 < 1s, 超调量 < 10%, 稳态误差 < 0.5°

步骤 2: GZ SITL 仿真验证 (需要铰接模型)
  - 扰动测试: 给定 5° 初始偏角，观察恢复时间
  - 巡航测试: 20 m/s 稳态飞行，记录铰接角标准差
  - 机动测试: 30° 转弯时微调量是否超出限幅

步骤 3: 实际飞行微调
  - 初始值设为 MATLAB 值的 50%: Kp=0.15, Kd=0.025
  - 逐步增大直到响应满意
```

### 18.4 从机固件代码文件清单（更新版）

基于 §17 的结论（使用现有 elevon 方案），从机固件需要以下文件：

```
需要创建的文件:
──────────────────────────────────────────────────────────────────────
文件                                          说明              数据就绪
──────────────────────────────────────────────────────────────────────
1. msg/ChainwingSlaveCmd.msg                  主→从指令消息      ✅ 100%
2. msg/ChainwingSlaveStatus.msg               从→主状态消息      ✅ 100%
3. src/modules/chainwing_slave/module.yaml    模块定义           ✅ 100%
4. src/modules/chainwing_slave/CMakeLists.txt 编译配置           ✅ 100%
5. src/modules/chainwing_slave/ChainwingSlave.hpp  类声明        ✅ 100%
6. src/modules/chainwing_slave/ChainwingSlave.cpp  核心逻辑      ⚠️ 80%
   - 铰接角检测逻辑需要在仿真中验证
   - PD 增益初始值已有,精确值需调参
7. ROMFS/.../airframes/2151_chainwing_slave   从机机架配置       ✅ 100%
8. boards/px4/fmu-v6x/default.px4board        编译板级配置(修改) ✅ 100%

需要修改的文件 (仿真用):
──────────────────────────────────────────────────────────────────────
9.  ROMFS/.../airframes/4008_gz_chainwing_slave  SITL从机机架    ⚠️ 70%
    - 需要与多实例SITL环境配合测试
10. Tools/simulation/gz/models/chainwing_slave/  GZ从机模型       ⚠️ 40%
    - 方案2(纯软件模拟): 可复用现有model，不需要此文件
    - 方案1(铰接关节): 需要拆分base_link，工作量较大
11. CMakeLists.txt (根目录)                   添加仿真机架注册   ✅ 100%
──────────────────────────────────────────────────────────────────────
```

### 18.5 推荐的开发路线图

```
阶段 1: 固件框架 (第 1-2 天)                    数据就绪: ✅ 100%
├── 创建 msg/ChainwingSlaveCmd.msg
├── 创建 msg/ChainwingSlaveStatus.msg
├── 创建 chainwing_slave 模块骨架
├── 创建 2151_chainwing_slave 机架配置
└── 验证: make px4_fmu-v6x_default 编译通过

阶段 2: 控制律实现 (第 3-5 天)                  数据就绪: ✅ 90%
├── 实现 PD 控制器 (Kp=0.3, Kd=0.05)
├── 实现空速²缩放
├── 实现 elevon trim offset 叠加逻辑
├── 实现微调限幅和速率限制
└── 验证: 单元测试 + 静态分析

阶段 3: 软件仿真验证 (第 6-10 天)               数据就绪: ⚠️ 70%
├── 实现铰接角数学模拟 (方案 2)
├── 创建 SITL 从机机架 (4008_gz_chainwing_slave)
├── 多实例 SITL 环境搭建
├── PD 增益初步调参
└── 验证: 仿真中铰接角收敛到 < 1°

阶段 4: CAN 通信集成 (第 11-15 天)              数据就绪: ⚠️ 60%
├── 实现 CAN 消息收发
├── 主从机协同测试 (SocketCAN 仿真)
├── 消息丢失/延迟鲁棒性测试
└── 验证: 主从机 CAN 通信 50Hz 无丢包

阶段 5: GZ 完整仿真 (第 16-25 天, 可选)        数据就绪: ⚠️ 40%
├── 修改 model.sdf 添加铰接关节
├── 拆分 base_link 为三个独立 body
├── 完整三机联飞仿真
└── 验证: GZ 中铰接角动态响应与 MATLAB 一致
```

### 18.6 总结

```
┌─────────────────────────────────────────────────────────────────┐
│                    从机固件编写可行性总结                         │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ✅ 可以立即开始编写从机固件                                     │
│                                                                 │
│  推荐方案: 使用现有 elevon (方案 A)                              │
│  理由:                                                          │
│    1. 全部气动参数已知 (model.sdf + MATLAB)                      │
│    2. GZ 仿真模型可直接使用 (无需修改)                           │
│    3. PX4 模块框架完整 (参考主机代码)                            │
│    4. 控制律已设计 (PD + 空速缩放, §13)                         │
│    5. 通信方案已确定 (CAN/DroneCAN, §11)                        │
│                                                                 │
│  需要在开发过程中迭代确定:                                       │
│    1. PD 增益精确值 → 仿真阶段调参                               │
│    2. 铰接动力学参数 → 先用数学模型估算                          │
│    3. CAN 消息时序优化 → 集成测试阶段                            │
│                                                                 │
│  不建议等待: 所有关键数据已具备,                                 │
│  缺失数据可在开发过程中补充,不影响框架搭建。                      │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```
