# 链翼无人机硬件仿真指南 (HITL Guide)

## 目录

1. [硬件仿真概述](#1-硬件仿真概述)
2. [三种仿真模式对比](#2-三种仿真模式对比)
3. [方案一：SIH 板载仿真（推荐入门）](#3-方案一sih-板载仿真推荐入门)
4. [方案二：Gazebo HITL 外部仿真](#4-方案二gazebo-hitl-外部仿真)
5. [方案三：SITL 纯软件仿真（参考）](#5-方案三sitl-纯软件仿真参考)
6. [硬件需求清单](#6-硬件需求清单)
7. [QGC 地面站配置](#7-qgc-地面站配置)
8. [常见问题排查](#8-常见问题排查)
9. [从仿真到实飞的过渡](#9-从仿真到实飞的过渡)

---

## 1. 硬件仿真概述

### 什么是硬件仿真？

硬件仿真（Hardware-In-The-Loop, HITL）是指在**真实飞控硬件**上运行 PX4 固件，但使用**虚拟的传感器和执行器**替代真实的物理设备。这样可以在不冒真实飞行风险的情况下，验证：

- 固件在真实硬件上的运行性能
- 控制回路的实时性
- 传感器融合算法的正确性
- 故障保护逻辑
- QGC 地面站通信

### 链翼的特殊性

链翼无人机有以下特殊之处，使得硬件仿真尤为重要：

| 特性 | 传统固定翼 | 链翼 |
|------|-----------|------|
| 偏航控制 | 方向舵 | **差动推力**（3电机） |
| 横滚控制 | 副翼 | **反向升降副翼** |
| 航向保持 | 被动稳定 | **主动控制**（FW_YAW_STAB_SC） |
| 惯性矩 | 小 | **大**（展开结构） |

这些非标准设计必须在真实硬件上验证后才能放心试飞。

---

## 2. 三种仿真模式对比

### 2.1 对比表

| 特性 | SITL (纯软件) | SIH (板载仿真) | Gazebo HITL (外部仿真) |
|------|:---:|:---:|:---:|
| 需要飞控硬件 | ❌ | ✅ | ✅ |
| 需要外部电脑 | ✅ (运行PX4+GZ) | ❌ (仅QGC) | ✅ (运行Gazebo) |
| 物理仿真精度 | ★★★★★ | ★★★ | ★★★★★ |
| 实时性验证 | ❌ | ✅ | ✅ |
| 传感器验证 | 模拟 | 模拟(板载) | 模拟(外部) |
| 控制回路延迟 | 0 (同进程) | 真实 | 真实+通信延迟 |
| 设置难度 | ★★ | ★ | ★★★★ |
| PX4参数 | — | `SYS_HITL=2` | `SYS_HITL=1` |
| 机架文件 | `4007_gz_chainwing` | `1103_chainwing_sih.hil` | 需自定义 |
| 3D 可视化 | ✅ (Gazebo) | ❌ (仅QGC) | ✅ (Gazebo) |

### 2.2 推荐选择

```
首次硬件验证 → SIH (板载仿真)  ← 最简单，5分钟上手
需要3D可视化 → Gazebo HITL     ← 复杂，需要额外配置
无硬件可用时 → SITL            ← 已有配置，直接使用
```

---

## 3. 方案一：SIH 板载仿真（推荐入门）

### 3.1 原理

```
┌─────────────────────────────────────────────────┐
│                  Pixhawk 飞控板                   │
│                                                  │
│  ┌──────────┐    ┌──────────┐    ┌───────────┐  │
│  │simulator │    │  EKF2    │    │ Commander  │  │
│  │  _sih    │───▶│ 传感器   │───▶│  控制器    │  │
│  │ (物理引擎)│    │  融合    │    │  分配器    │  │
│  └─────▲────┘    └──────────┘    └─────┬─────┘  │
│        │                               │        │
│        │   执行器输出（虚拟）             │        │
│        └───────────────────────────────┘        │
│                                                  │
│  ┌──────────┐    ┌──────────┐    ┌───────────┐  │
│  │sensor_   │    │sensor_   │    │sensor_    │  │
│  │baro_sim  │    │mag_sim   │    │gps_sim    │  │
│  └──────────┘    └──────────┘    └───────────┘  │
│                                                  │
│                  MAVLink USB/UART                 │
└───────────────────────┬─────────────────────────┘
                        │
                        ▼
              ┌──────────────────┐
              │   QGC 地面站      │
              │  (笔记本电脑)     │
              └──────────────────┘
```

**所有仿真在飞控板上运行**，无需外部仿真器。只需 USB 连接 QGC 即可。

### 3.2 所需硬件

| 硬件 | 规格 | 备注 |
|------|------|------|
| Pixhawk 飞控 | FMU-v5/v5x/v6x | 推荐 Holybro Pixhawk 6X |
| USB 数据线 | Micro-USB 或 Type-C | 连接飞控到电脑 |
| 电源 | USB 供电即可 | SIH 不需要电池 |
| 电脑 | 安装 QGC | Windows/Mac/Linux |

### 3.3 详细步骤

#### 步骤1：编译固件

```bash
# 进入 PX4 源码目录
cd ~/PX4-Autopilot    # 或你的 PX4_test 目录

# 确认 SIH 机架文件存在
ls ROMFS/px4fmu_common/init.d/airframes/1103_chainwing_sih.hil

# 编译固件（以 Pixhawk 6X 为例）
make px4_fmu-v6x_default

# 其他常见板型：
# make px4_fmu-v5_default      # Pixhawk 4
# make px4_fmu-v5x_default     # Pixhawk 5X
# make px4_fmu-v6c_default     # Pixhawk 6C
```

> **编译时间**：首次编译约 5-10 分钟，后续增量编译约 1-2 分钟。

#### 步骤2：烧录固件

**方法A：命令行烧录**
```bash
# 用 USB 连接 Pixhawk，然后：
make px4_fmu-v6x_default upload
```

**方法B：QGC 烧录**
1. 打开 QGC → 齿轮图标（Vehicle Setup）
2. 点击 **Firmware**
3. 用 USB 连接 Pixhawk（QGC 自动检测）
4. 选择 **Custom firmware file...**
5. 浏览到 `build/px4_fmu-v6x_default/px4_fmu-v6x_default.px4`
6. 点击 OK，等待烧录完成

#### 步骤3：配置机架

1. QGC → Vehicle Setup → **Airframe**
2. 在列表中找到 **Simulation** 分类
3. 选择 **Chain-Wing UAV SIH (3-unit fixed-wing)**（ID: 1103）
4. 点击 **Apply and Restart**
5. 等待飞控重启完成

> **注意**：选择机架后，PX4 会自动加载 `1103_chainwing_sih.hil` 中的所有参数。

#### 步骤4：验证参数

在 QGC **Parameters** 面板中确认以下关键参数：

| 参数 | 期望值 | 说明 |
|------|--------|------|
| `SYS_HITL` | 2 | SIH 模式已启用 |
| `CA_ROTOR_COUNT` | 3 | 3个电机 |
| `CA_SV_CS_COUNT` | 3 | 3个舵面 |
| `SIH_VEHICLE_TYPE` | 1 | 固定翼 |
| `SIH_MASS` | 1.0 | 1.0 kg |
| `FW_YAW_STAB_SC` | 2.0 | 航向保持增益 |
| `CBRK_SUPPLY_CHK` | 894281 | 电源检查已绕过 |

#### 步骤5：起飞测试

```
1. QGC → 地图视图
2. 确认飞控连接（顶部栏显示 "Ready to Fly"）
3. 选择飞行模式：Takeoff 或 Mission
4. 滑动解锁（Arm）
5. 观察 QGC 上飞机姿态和位置变化
6. 飞机应在 QGC 地图上显示虚拟飞行轨迹
```

#### 步骤6：测试各飞行模式

```
# 按以下顺序测试：

1. Takeoff → 观察起飞爬升
2. Loiter → 观察盘旋（应为圆形轨迹）
3. Mission → 上传航点任务
4. RTL → 观察返航降落
5. Stabilized → 使用虚拟摇杆手飞（需 COM_RC_IN_MODE=3）
```

### 3.4 SIH 物理参数说明

`1103_chainwing_sih.hil` 中的 SIH 参数对应链翼 MATLAB 模型：

| SIH 参数 | 值 | 物理含义 | MATLAB 对应 |
|----------|-----|---------|------------|
| `SIH_MASS` | 1.0 kg | 飞机总质量 | m_total |
| `SIH_IXX` | 1.02 kg·m² | 横滚惯性矩 | Ixx |
| `SIH_IYY` | 0.164 kg·m² | 俯仰惯性矩 | Iyy |
| `SIH_IZZ` | 1.17 kg·m² | 偏航惯性矩 | Izz |
| `SIH_IXZ` | 0.01 kg·m² | 交叉惯性积 | Ixz |
| `SIH_T_MAX` | 15.0 N | 最大推力（3电机总和） | 3×T_max |
| `SIH_KDV` | 0.5 | 阻力系数 | CD |
| `SIH_VEHICLE_TYPE` | 1 | 固定翼类型 | — |

> **注意**：SIH 的气动模型是简化的（线性气动力），不如 Gazebo 精确。
> 但足以验证控制逻辑、故障保护、通信链路等。

### 3.5 SIH 的局限性

| 方面 | 说明 |
|------|------|
| 气动精度 | 简化线性模型，无翼尖涡流/地面效应 |
| 3D 可视化 | 无 Gazebo 3D 画面（仅 QGC 地图） |
| 碰撞检测 | 无（飞机可穿过地面） |
| 风模拟 | 有（SIH_DISTURBANCE_X/Y/Z） |
| 差动推力效果 | 取决于 SIH 内部偏航力矩模型 |

---

## 4. 方案二：Gazebo HITL 外部仿真

### 4.1 原理

```
┌──────────────┐         MAVLink          ┌───────────────────┐
│  Pixhawk     │◀════════════════════════▶│  Linux 电脑       │
│  飞控板      │    USB/UART/UDP          │                   │
│              │                          │  ┌─────────────┐  │
│  PX4 固件    │   传感器数据 ◀────────────│  │  Gazebo     │  │
│  SYS_HITL=1  │   执行器指令 ────────────▶│  │  仿真器     │  │
│              │                          │  │  (chainwing) │  │
│  pwm_out_sim │                          │  └─────────────┘  │
│  (HIL mode)  │                          │                   │
└──────────────┘                          │  ┌─────────────┐  │
                                          │  │  QGC        │  │
                                          │  │  地面站      │  │
                                          │  └─────────────┘  │
                                          └───────────────────┘
```

### 4.2 所需硬件和软件

| 项目 | 规格 | 备注 |
|------|------|------|
| Pixhawk 飞控 | FMU-v5 或更高 | 运行 PX4 固件 |
| Linux 电脑 | Ubuntu 22.04, 8GB+ RAM | 运行 Gazebo + QGC |
| USB 线 | — | 连接飞控到电脑 |
| Gazebo | Harmonic 或 Garden | 已安装并配置 |
| PX4 SITL 环境 | — | `make px4_sitl gz_chainwing` 能成功 |

### 4.3 详细步骤

#### 步骤1：编译 HITL 固件

```bash
cd ~/PX4-Autopilot

# 编译硬件固件（不是SITL！）
make px4_fmu-v6x_default
```

#### 步骤2：烧录固件

```bash
make px4_fmu-v6x_default upload
```

#### 步骤3：配置 SYS_HITL=1

在 QGC 中设置：
```
SYS_HITL = 1        # 启用外部 HITL 模式
```
或通过 NSH 控制台：
```
pxh> param set SYS_HITL 1
pxh> reboot
```

#### 步骤4：配置 HIL 执行器映射

需要将 PX4 的执行器输出映射到 MAVLink HIL 通道：

```
# 舵面（Servo）
HIL_ACT_FUNC1 = 201    # Servo 1 → 左升降副翼
HIL_ACT_FUNC2 = 202    # Servo 2 → 升降舵
HIL_ACT_FUNC3 = 203    # Servo 3 → 右升降副翼

# 电机（ESC）
HIL_ACT_FUNC4 = 101    # Motor 1 → 左电机
HIL_ACT_FUNC5 = 102    # Motor 2 → 中间电机
HIL_ACT_FUNC6 = 103    # Motor 3 → 右电机
```

#### 步骤5：启动 Gazebo 仿真

```bash
# 在 Linux 电脑上启动 Gazebo（HITL 模式）
cd ~/PX4-Autopilot
# 先启动 Gazebo 世界
gz sim -r Tools/simulation/gz/worlds/flat_terrain.sdf &

# 等待 Gazebo 完全启动，然后添加链翼模型
gz service -s /world/flat_terrain/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 5000 \
  --req 'sdf_filename: "chainwing/model.sdf"'
```

#### 步骤6：连接飞控到 Gazebo

需要 MAVLink 桥接器将飞控的 HIL 数据转发到 Gazebo：

```bash
# 使用 mavlink-router 或 mavproxy
# 将 USB 串口 (/dev/ttyACM0) 桥接到 UDP
mavlink-routerd -e 127.0.0.1:14540 /dev/ttyACM0:921600
```

> **注意**：Gazebo HITL 配置较复杂，需要自定义 MAVLink 桥接插件。
> 对于大多数验证需求，推荐使用 SIH 方案。

### 4.4 Gazebo HITL 的优势

- 完整的 3D 可视化（看到飞机姿态、轨迹）
- 精确的气动模型（LiftDrag 插件、多体动力学）
- 真实的碰撞检测和地面效应
- 可添加风干扰、地形障碍
- 与 SITL 共用同一套 GZ 模型（`chainwing/model.sdf`）

---

## 5. 方案三：SITL 纯软件仿真（参考）

已有完整配置，仅需：

```bash
cd ~/PX4-Autopilot

# 清除旧参数（重要！）
rm -f build/px4_sitl_default/rootfs/parameters*.bson

# 启动仿真
make px4_sitl gz_chainwing
```

详细说明参见 [CHAINWING_FIRMWARE_DOC.md](CHAINWING_FIRMWARE_DOC.md) 和
[CHAINWING_TUNING_GUIDE.md](CHAINWING_TUNING_GUIDE.md)。

---

## 6. 硬件需求清单

### 6.1 最小配置（SIH 方案）

| 项目 | 数量 | 型号建议 | 预估价格 |
|------|------|---------|---------|
| Pixhawk 飞控 | 1 | Holybro Pixhawk 6C Mini | ~$100 |
| USB 数据线 | 1 | Type-C | ~$5 |
| 电脑 | 1 | 任何（安装QGC） | 已有 |

**总计：约 $105**

### 6.2 完整配置（HITL + 未来实飞）

| 项目 | 数量 | 型号建议 | 预估价格 |
|------|------|---------|---------|
| Pixhawk 飞控 | 1 | Holybro Pixhawk 6X | ~$300 |
| GPS 模块 | 1 | Holybro M9N GPS | ~$55 |
| 空速管 | 1 | Holybro Digital Airspeed | ~$60 |
| 电源模块 | 1 | Holybro PM07 | ~$25 |
| RC 遥控器 | 1 | RadioMaster TX16S | ~$200 |
| RC 接收机 | 1 | TBS Crossfire Nano RX | ~$25 |
| USB 数据线 | 1 | Type-C | ~$5 |
| Ubuntu 电脑 | 1 | 安装 Gazebo + QGC | 已有 |

**总计：约 $670**

### 6.3 链翼专用硬件

| 项目 | 数量 | 说明 |
|------|------|------|
| 无刷电机 | 3 | 对应 Motor 0/1/2 |
| ESC 电调 | 3 | 支持 PWM/DShot |
| 舵机 | 3 | 对应 CS0/CS1/CS2 |
| 电池 | 1 | 4S LiPo |
| 机身结构 | 1 | 3单元链翼框架 |

> **注意**：SIH 方案**不需要**连接任何电机/舵机。所有执行器信号都是虚拟的。

---

## 7. QGC 地面站配置

### 7.1 安装 QGC

```bash
# Ubuntu
sudo usermod -a -G dialout $USER
sudo apt-get install gstreamer1.0-plugins-bad gstreamer1.0-libav -y
# 下载并安装 QGC AppImage
chmod +x QGroundControl.AppImage
./QGroundControl.AppImage
```

### 7.2 连接飞控

1. USB 连接 Pixhawk
2. QGC 自动检测连接（状态栏显示 "Connected"）
3. 如果未自动连接：
   - QGC → Application Settings → Comm Links
   - 添加 Serial: `/dev/ttyACM0`, Baud: `921600`

### 7.3 虚拟摇杆设置（增稳模式必须）

1. QGC → Application Settings → **Virtual Joystick**
2. ✅ 启用 **Virtual Joystick**
3. 这提供屏幕上的虚拟摇杆用于手动控制

> **重要**：链翼已设置 `COM_RC_IN_MODE=3`（接受 QGC 摇杆输入）。
> 如果你有物理 RC 遥控器，可跳过此步骤。

### 7.4 飞行模式配置

在 QGC → Vehicle Setup → **Flight Modes** 中配置：

| 通道/按钮 | 推荐模式 |
|----------|---------|
| 模式1 | Takeoff |
| 模式2 | Loiter |
| 模式3 | Mission |
| 模式4 | RTL |
| 模式5 | Stabilized |
| 模式6 | Manual |

---

## 8. 常见问题排查

### 8.1 SIH 模式问题

| 问题 | 原因 | 解决方案 |
|------|------|---------|
| "SYS_HITL not found" | 固件版本太旧 | 重新编译最新固件 |
| 重启后 SYS_HITL 变回 0 | 参数未保存 | 用 `param save` 保存 |
| 飞机在QGC上不动 | SIH 未启动 | 确认 SYS_HITL=2，重启 |
| "Arming denied: battery" | 电源检查 | 确认 CBRK_SUPPLY_CHK=894281 |
| "Arming denied: safety switch" | IO安全开关 | 确认 CBRK_IO_SAFETY=22027 |
| 飞机翻转/不稳定 | 惯性参数错误 | 检查 SIH_IXX/IYY/IZZ |
| "No valid data from IMU" | SIH 物理异常 | 重启飞控 |

### 8.2 Gazebo HITL 问题

| 问题 | 原因 | 解决方案 |
|------|------|---------|
| Gazebo 无法连接飞控 | MAVLink桥接未配置 | 检查 mavlink-router 设置 |
| 传感器数据不更新 | HIL 数据流中断 | 检查 USB 连接和波特率 |
| 飞机在 Gazebo 中不动 | 执行器映射错误 | 检查 HIL_ACT_FUNC1-6 |
| 延迟/卡顿 | 串口带宽不足 | 使用 921600 波特率 |

### 8.3 通用问题

| 问题 | 原因 | 解决方案 |
|------|------|---------|
| QGC 无法连接 | USB 驱动/权限 | `sudo chmod 666 /dev/ttyACM0` |
| 参数加载失败 | 机架文件未找到 | 确认 CMakeLists.txt 已注册 |
| 编译错误 | 依赖缺失 | `make distclean` 后重新编译 |
| "Preflight Fail: GPS" | SIH GPS 未启动 | 确认 SYS_HITL=2（自动启动GPS） |

---

## 9. 从仿真到实飞的过渡

### 9.1 验证清单

在从仿真过渡到实飞之前，必须完成以下所有项目：

#### 阶段1：SITL 验证 ✓
- [x] Gazebo 模型飞行正常
- [x] 所有飞行模式可用
- [x] RTL 正确返航降落
- [x] 电池故障保护工作

#### 阶段2：SIH 硬件验证
- [ ] 固件在 Pixhawk 上正常启动
- [ ] 所有传感器模拟数据正常
- [ ] QGC 通信稳定
- [ ] 虚拟飞行轨迹合理
- [ ] 故障保护逻辑正确触发

#### 阶段3：地面测试（连接真实硬件）
- [ ] 切换到 `2150_chainwing` 机架（`SYS_HITL=0`）
- [ ] 校准真实传感器（加速度计、磁力计、陀螺仪）
- [ ] 校准空速管
- [ ] 验证 GPS 定位
- [ ] 验证 RC 遥控器连接和通道映射
- [ ] 验证电机转向和舵面方向
- [ ] 进行地面带桨静力测试

#### 阶段4：首飞
- [ ] 选择开阔平坦场地
- [ ] 风速 < 5 m/s
- [ ] 电池满电
- [ ] 首先使用 **Stabilized** 模式手动滑跑
- [ ] 确认增稳响应正确后尝试起飞
- [ ] 高度 > 50m 后切换 **Loiter** 验证自稳
- [ ] 飞行 2-3 分钟后 RTL 降落

### 9.2 参数调整

从 SIH/SITL 过渡到实飞时，以下参数可能需要调整：

| 参数 | SIH/SITL 值 | 实飞建议 | 原因 |
|------|------------|---------|------|
| `SYS_HITL` | 2 | **0** | 使用真实传感器 |
| `CBRK_SUPPLY_CHK` | 894281 | **0** | 启用电池检查 |
| `CBRK_IO_SAFETY` | 22027 | **0** | 启用安全开关 |
| `COM_LOW_BAT_ACT` | 0 | **2 (RTL)** | 启用电池故障保护 |
| `SIM_BAT_DRAIN` | 3600 | 删除 | 无仿真电池 |
| `FD_ESCS_EN` | 0 | **1** | 启用ESC故障检测 |
| `NAV_DLL_ACT` | 0 | **2 (RTL)** | 启用数据链路丢失保护 |
| `COM_RC_IN_MODE` | 3 | **0 (RC only)** | 仅接受RC输入 |

### 9.3 硬件机架文件对比

| 参数 | SITL (`4007`) | SIH (`1103`) | 硬件 (`2150`) |
|------|:---:|:---:|:---:|
| SYS_HITL | 0 | 2 | 0 |
| 电机/舵面 | GZ 映射 | HIL 映射 | PWM 映射 |
| 传感器 | 模拟(GZ) | 模拟(SIH) | 真实 |
| 安全检查 | 绕过 | 绕过 | **启用** |
| 控制参数 | ✅ 相同 | ✅ 相同 | ✅ 相同 |

---

## 附录 A：SIH 机架文件参考

机架文件路径：`ROMFS/px4fmu_common/init.d/airframes/1103_chainwing_sih.hil`

关键参数一览：

```bash
SYS_HITL = 2                # SIH 模式
SIH_VEHICLE_TYPE = 1        # 固定翼
SIH_MASS = 1.0              # 1.0 kg
SIH_IXX = 1.02              # 横滚惯性矩
SIH_IYY = 0.164             # 俯仰惯性矩
SIH_IZZ = 1.17              # 偏航惯性矩
SIH_T_MAX = 15.0            # 最大推力 15N
CA_ROTOR_COUNT = 3          # 3电机差动推力
CA_SV_CS_COUNT = 3          # 3舵面
FW_YAW_STAB_SC = 2.0        # 航向保持
```

## 附录 B：编译目标速查

| 板型 | 编译命令 | 适用硬件 |
|------|---------|---------|
| Pixhawk 4 | `make px4_fmu-v5_default` | Holybro Pixhawk 4 |
| Pixhawk 5X | `make px4_fmu-v5x_default` | Holybro Pixhawk 5X |
| Pixhawk 6C | `make px4_fmu-v6c_default` | Holybro Pixhawk 6C |
| Pixhawk 6X | `make px4_fmu-v6x_default` | Holybro Pixhawk 6X |
| CUAV V5+ | `make px4_fmu-v5_default` | CUAV V5+ |
| mRo Control Zero | `make mro_ctrl-zero-h7_default` | mRo |
| SITL | `make px4_sitl gz_chainwing` | 无硬件 |

## 附录 C：完整硬件仿真测试流程图

```
┌─────────────────────────────────────────────────────────┐
│                    硬件仿真测试流程                        │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  ① 编译固件                                              │
│     make px4_fmu-v6x_default                            │
│         │                                               │
│         ▼                                               │
│  ② 烧录固件                                              │
│     make px4_fmu-v6x_default upload                     │
│         │                                               │
│         ▼                                               │
│  ③ 选择机架                                              │
│     QGC → Airframe → "Chain-Wing UAV SIH" (1103)        │
│         │                                               │
│         ▼                                               │
│  ④ 验证参数                                              │
│     SYS_HITL=2, CA_ROTOR_COUNT=3, SIH_MASS=1.0         │
│         │                                               │
│         ▼                                               │
│  ⑤ 基础测试                                              │
│     Takeoff → Loiter → RTL                              │
│         │                                               │
│         ▼                                               │
│  ⑥ 模式测试                                              │
│     Mission → Stabilized → Manual                       │
│         │                                               │
│         ▼                                               │
│  ⑦ 故障保护测试                                          │
│     电池低电 → 数据链丢失 → GPS 丢失                      │
│         │                                               │
│         ▼                                               │
│  ⑧ 参数调优                                              │
│     根据 SIH 结果微调 PID 增益                            │
│         │                                               │
│         ▼                                               │
│  ⑨ 切换到实飞配置                                        │
│     2150_chainwing, SYS_HITL=0, 启用安全检查              │
│                                                         │
└─────────────────────────────────────────────────────────┘
```
