# ChainWing 参数扫描工具

## 概述

自动化参数扫描 + 日志分析工具，用于 ChainWing 3-body GZ SITL 仿真。

**版本**: v2.0 — 22 轮完整参数扫描，覆盖全部 10 个关键参数

**功能**:
1. `param_sweep_sitl.sh` — 自动遍历参数组合，运行仿真，保存日志
2. `analyze_param_sweep.py` — 分析所有日志，生成对比图 + 性能指标
3. `sweep_config.json` — 22 轮参数扫描配置（9 个阶段）

## 覆盖参数

| 参数 | PX4 默认 | 机架默认 | 扫描阶段 | 扫描值 |
|------|----------|---------|---------|--------|
| FW_YR_P | 0.05 | 0.6 | B_yaw_P | 0.05→0.15→0.3→0.6 |
| FW_YR_I | 0.1 | 0.5 | C_yaw_I | 0.1→0.2→0.5 |
| FW_YR_FF | 0.3 | 0.5 | D_yaw_FF | 0.3→0.5→0.7 |
| FW_YR_D | 0.0 | 0.0 | E_yaw_D | 0→0.005→0.01→0.02 |
| FW_YAW_STAB_SC | 0.0 | 1.0 | F_heading_hold | 0→1.0→2.0 |
| FW_RR_P | 0.05 | 0.3 | G_roll_P | 0.05→0.15→0.3→0.5 |
| FW_PR_P | 0.08 | 0.9 | H_pitch_P | 0.08→0.2→0.5→0.9 |
| CW_SLV_EN | 0 | 1 | I_hinge | 0→1 |
| CW_SLV_KP | 1.5 | 1.5 | I_hinge | 1.0→1.5→2.0 |
| CW_SLV_KD | 0.2 | 0.2 | I_hinge | 0.1→0.2→0.3 |

## 快速开始

### 1. 安装依赖

```bash
# 系统依赖
sudo apt install jq

# Python 依赖
pip3 install pyulog matplotlib numpy
```

### 2. 编译 PX4 SITL（首次）

```bash
cd PX4_test
DONT_RUN=1 make px4_sitl_default
```

### 3. 运行参数扫描

```bash
# 运行所有轮次（22 轮 × 90s 飞行 ≈ 66 分钟）
bash scripts/param_sweep_sitl.sh

# 只运行第 3 轮
bash scripts/param_sweep_sitl.sh --run 3

# 只运行某个阶段（如偏航 P 扫描）
bash scripts/param_sweep_sitl.sh --phase B_yaw_P

# 使用自定义配置
bash scripts/param_sweep_sitl.sh my_config.json
```

### 4. 分析日志

```bash
# 基本分析（生成 8 张 PNG 对比图 + CSV 汇总表）
python3 scripts/analyze_param_sweep.py sweep_logs/

# 生成 PDF 报告
python3 scripts/analyze_param_sweep.py sweep_logs/ --pdf

# 按偏航角速率误差排序
python3 scripts/analyze_param_sweep.py sweep_logs/ --sort yaw_rms

# 指定输出目录
python3 scripts/analyze_param_sweep.py sweep_logs/ --output results/
```

## 9 个扫描阶段

| 阶段 | 名称 | 轮次 | 扫描变量 | 目的 |
|------|------|------|---------|------|
| A | baseline | 1 | — | PX4 默认基线 |
| B | yaw_P | 2-4 | FW_YR_P | 偏航 P 增益扫描 |
| C | yaw_I | 5-6 | FW_YR_I | 偏航积分扫描 |
| D | yaw_FF | 7-8 | FW_YR_FF | 偏航前馈扫描 |
| E | yaw_D | 9-11 | FW_YR_D | 偏航微分阻尼扫描 |
| F | heading_hold | 12-13 | FW_YAW_STAB_SC | 航向保持增益 |
| G | roll_P | 14-16 | FW_RR_P | 滚转角速率 P 扫描 |
| H | pitch_P | 17-19 | FW_PR_P | 俯仰角速率 P 扫描 |
| I | hinge | 20-22 | CW_SLV_KP/KD | 铰链修正 PD 扫描 |

## 完整 22 轮参数表

| # | 名称 | 阶段 | YR_P | YR_I | YR_D | YR_FF | STAB | RR_P | PR_P | SLV | KP | KD |
|---|------|------|------|------|------|-------|------|------|------|-----|----|----|
| 1 | baseline | A | 0.05 | 0.1 | 0 | 0.3 | 0 | 0.05 | 0.08 | 0 | - | - |
| 2 | yrp_015 | B | **0.15** | 0.1 | 0 | 0.3 | 0 | 0.05 | 0.08 | 0 | - | - |
| 3 | yrp_030 | B | **0.3** | 0.1 | 0 | 0.3 | 0 | 0.05 | 0.08 | 0 | - | - |
| 4 | yrp_060 | B | **0.6** | 0.1 | 0 | 0.3 | 0 | 0.05 | 0.08 | 0 | - | - |
| 5 | yri_020 | C | 0.6 | **0.2** | 0 | 0.3 | 0 | 0.05 | 0.08 | 0 | - | - |
| 6 | yri_050 | C | 0.6 | **0.5** | 0 | 0.3 | 0 | 0.05 | 0.08 | 0 | - | - |
| 7 | yrff_050 | D | 0.6 | 0.5 | 0 | **0.5** | 0 | 0.05 | 0.08 | 0 | - | - |
| 8 | yrff_070 | D | 0.6 | 0.5 | 0 | **0.7** | 0 | 0.05 | 0.08 | 0 | - | - |
| 9 | yrd_005 | E | 0.6 | 0.5 | **0.005** | 0.5 | 0 | 0.05 | 0.08 | 0 | - | - |
| 10 | yrd_010 | E | 0.6 | 0.5 | **0.01** | 0.5 | 0 | 0.05 | 0.08 | 0 | - | - |
| 11 | yrd_020 | E | 0.6 | 0.5 | **0.02** | 0.5 | 0 | 0.05 | 0.08 | 0 | - | - |
| 12 | stab_10 | F | 0.6 | 0.5 | 0 | 0.5 | **1.0** | 0.05 | 0.08 | 0 | - | - |
| 13 | stab_20 | F | 0.6 | 0.5 | 0 | 0.5 | **2.0** | 0.05 | 0.08 | 0 | - | - |
| 14 | rrp_015 | G | 0.6 | 0.5 | 0 | 0.5 | 1.0 | **0.15** | 0.08 | 0 | - | - |
| 15 | rrp_030 | G | 0.6 | 0.5 | 0 | 0.5 | 1.0 | **0.3** | 0.08 | 0 | - | - |
| 16 | rrp_050 | G | 0.6 | 0.5 | 0 | 0.5 | 1.0 | **0.5** | 0.08 | 0 | - | - |
| 17 | prp_020 | H | 0.6 | 0.5 | 0 | 0.5 | 1.0 | 0.3 | **0.2** | 0 | - | - |
| 18 | prp_050 | H | 0.6 | 0.5 | 0 | 0.5 | 1.0 | 0.3 | **0.5** | 0 | - | - |
| 19 | prp_090 | H | 0.6 | 0.5 | 0 | 0.5 | 1.0 | 0.3 | **0.9** | 0 | - | - |
| 20 | hinge_kp10 | I | 0.6 | 0.5 | 0 | 0.5 | 1.0 | 0.3 | 0.9 | **1** | **1.0** | **0.1** |
| 21 | hinge_kp15 | I | 0.6 | 0.5 | 0 | 0.5 | 1.0 | 0.3 | 0.9 | **1** | **1.5** | **0.2** |
| 22 | hinge_kp20 | I | 0.6 | 0.5 | 0 | 0.5 | 1.0 | 0.3 | 0.9 | **1** | **2.0** | **0.3** |

> **设计原则**: 每个阶段只变一个参数，前一阶段的最优值锁定后带入下一阶段。

## 配置文件说明

`sweep_config.json` 结构:

```json
{
    "sitl": {
        "model": "chainwing_3body",
        "world": "default",
        "headless": true,
        "boot_wait_sec": 20,
        "flight_duration_sec": 90,
        "land_wait_sec": 25
    },
    "runs": [
        {
            "name": "01_baseline",
            "phase": "A_baseline",
            "description": "PX4 defaults",
            "params": {
                "FW_YR_P": 0.05,
                "FW_RR_P": 0.05,
                "FW_PR_P": 0.08,
                "CW_SLV_EN": 0
            }
        }
    ]
}
```

### 字段说明

| 字段 | 说明 |
|------|------|
| `sitl.model` | Gazebo 模型名 |
| `sitl.world` | Gazebo 世界文件（不含 .sdf） |
| `sitl.headless` | 无 GUI 运行（推荐 true） |
| `sitl.boot_wait_sec` | PX4 启动等待时间 |
| `sitl.flight_duration_sec` | 每轮飞行时长 |
| `runs[].name` | 轮次名称（用于日志文件名） |
| `runs[].phase` | 阶段名（用于 `--phase` 过滤） |
| `runs[].params` | 该轮设置的 PX4 参数 |

## 输出文件

```
sweep_logs/
├── run_01_01_baseline.ulg              # ULog 日志
├── run_01_01_baseline_params.json      # 参数记录
├── run_02_02_yrp_015.ulg
├── ...
└── analysis/
    ├── 01_attitude_comparison.png      # 姿态角对比 (Roll/Pitch/Yaw)
    ├── 02_yaw_rate_tracking.png        # 偏航角速率跟踪 (per run)
    ├── 02b_roll_rate_tracking.png      # 滚转角速率跟踪 (per run)
    ├── 02c_pitch_rate_tracking.png     # 俯仰角速率跟踪 (per run)
    ├── 03_servo_output.png             # 舵面输出 (S0/S1/S2)
    ├── 04_hinge_correction.png         # 铰链修正 (angle+trim)
    ├── 05_metrics_comparison.png       # 性能指标柱状图 (9 metrics)
    ├── 06_metrics_table.png            # 汇总表
    ├── metrics_summary.csv             # CSV 数据
    └── sweep_report.pdf                # PDF 报告 (--pdf)
```

## 性能指标

| 指标 | 说明 | 越小越好 |
|------|------|---------|
| Roll RMS Error | 滚转角跟踪均方根误差 (deg) | ✅ |
| Pitch RMS Error | 俯仰角跟踪均方根误差 (deg) | ✅ |
| Roll Rate RMS Error | 滚转角速率跟踪误差 (deg/s) | ✅ |
| Pitch Rate RMS Error | 俯仰角速率跟踪误差 (deg/s) | ✅ |
| Yaw Rate RMS Error | 偏航角速率跟踪误差 (deg/s) | ✅ |
| Yaw Rate Max Error | 偏航角速率最大偏差 (deg/s) | ✅ |
| Yaw Rate Oscillation | 偏航振荡频率 (Hz) | ✅ |
| Servo Activity | 舵面活动量 (sum|delta|/s) | ⚠️ 过小=不响应，过大=振荡 |
| Hinge Angle RMS | 铰链偏角均方根 (deg) | ✅ (有修正时) |

## 自定义扫描

编辑 `sweep_config.json` 添加你的参数组合:

```json
{
    "name": "my_custom_test",
    "phase": "X_custom",
    "description": "Custom parameter test",
    "params": {
        "FW_RR_P": 0.5,
        "FW_PR_P": 0.3,
        "FW_YR_P": 0.6,
        "CW_SLV_EN": 1,
        "CW_SLV_KP": 2.0
    }
}
```

## 故障排查

| 问题 | 解决方案 |
|------|---------|
| `jq: command not found` | `sudo apt install jq` |
| `PX4 SITL 未编译` | `DONT_RUN=1 make px4_sitl_default` |
| PX4 启动失败 | 查看 `/tmp/px4_sweep_stdout_*.log` |
| 无日志文件 | 增加 `boot_wait_sec`，检查 SDLOG_MODE |
| Gazebo 未退出 | 手动 `ps aux \| grep gz` 并 kill |
| matplotlib 报错 | 确保已安装: `pip3 install matplotlib` |
| 图表中文乱码 | 安装中文字体: `sudo apt install fonts-wqy-zenhei` |
| 22 轮太多 | 用 `--phase B_yaw_P` 只运行一个阶段 |
