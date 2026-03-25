# ChainWing 参数扫描工具

## 概述

自动化参数扫描 + 日志分析工具，用于 ChainWing 3-body GZ SITL 仿真。

**功能**:
1. `param_sweep_sitl.sh` — 自动遍历参数组合，运行仿真，保存日志
2. `analyze_param_sweep.py` — 分析所有日志，生成对比图 + 性能指标
3. `sweep_config.json` — 参数扫描配置（可自定义）

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
# 运行所有轮次（默认 10 轮 × 90s 飞行 ≈ 30 分钟）
bash scripts/param_sweep_sitl.sh

# 只运行第 3 轮
bash scripts/param_sweep_sitl.sh --run 3

# 使用自定义配置
bash scripts/param_sweep_sitl.sh my_config.json
```

### 4. 分析日志

```bash
# 基本分析（生成 6 张 PNG 对比图 + CSV 汇总表）
python3 scripts/analyze_param_sweep.py sweep_logs/

# 生成 PDF 报告
python3 scripts/analyze_param_sweep.py sweep_logs/ --pdf

# 按偏航角速率误差排序
python3 scripts/analyze_param_sweep.py sweep_logs/ --sort yaw_rms

# 指定输出目录
python3 scripts/analyze_param_sweep.py sweep_logs/ --output results/
```

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
            "name": "baseline_no_yaw",
            "description": "...",
            "params": {
                "FW_YR_P": 0.05,
                "CW_SLV_EN": 0
            }
        }
    ]
}
```

### 参数说明

| 字段 | 说明 |
|------|------|
| `sitl.model` | Gazebo 模型名 |
| `sitl.world` | Gazebo 世界文件（不含 .sdf） |
| `sitl.headless` | 无 GUI 运行（推荐 true） |
| `sitl.boot_wait_sec` | PX4 启动等待时间 |
| `sitl.flight_duration_sec` | 每轮飞行时长 |
| `runs[].name` | 轮次名称（用于日志文件名） |
| `runs[].params` | 该轮设置的 PX4 参数 |

## 默认扫描方案

| 轮次 | 名称 | FW_YR_P | FW_YR_D | CW_SLV_EN | 说明 |
|------|------|---------|---------|------------|------|
| 1 | baseline_no_yaw | 0.05 | 0 | 0 | PX4 默认基线 |
| 2 | yaw_P_015 | 0.15 | 0 | 0 | 轻度 P 增强 |
| 3 | yaw_P_030 | 0.3 | 0 | 0 | 中度 P 增强 |
| 4 | yaw_P_060_full | 0.6 | 0 | 0 | 完整机架参数 |
| 5 | yaw_PD_D005 | 0.6 | 0.005 | 0 | + 轻 D 阻尼 |
| 6 | yaw_PD_D010 | 0.6 | 0.01 | 0 | + 中 D 阻尼 |
| 7 | yaw_PD_D020 | 0.6 | 0.02 | 0 | + 强 D 阻尼 |
| 8 | hinge_corr_default | 0.6 | 0 | 1 | 铰链修正 KP=1.5 |
| 9 | hinge_corr_KP20 | 0.6 | 0 | 1 | 铰链修正 KP=2.0 |
| 10 | hinge_corr_KP10 | 0.6 | 0 | 1 | 铰链修正 KP=1.0 |

## 输出文件

```
sweep_logs/
├── run_01_baseline_no_yaw.ulg          # ULog 日志
├── run_01_baseline_no_yaw_params.json  # 参数记录
├── run_02_yaw_P_015.ulg
├── ...
└── analysis/
    ├── 01_attitude_comparison.png      # 姿态对比图
    ├── 02_yaw_rate_tracking.png        # 偏航角速率跟踪
    ├── 03_servo_output.png             # 舵面输出
    ├── 04_hinge_correction.png         # 铰链修正
    ├── 05_metrics_comparison.png       # 性能指标柱状图
    ├── 06_metrics_table.png            # 汇总表
    ├── metrics_summary.csv             # CSV 数据
    └── sweep_report.pdf                # PDF 报告（--pdf）
```

## 性能指标

| 指标 | 说明 | 越小越好 |
|------|------|---------|
| Roll RMS Error | 滚转角跟踪均方根误差 (deg) | ✅ |
| Pitch RMS Error | 俯仰角跟踪均方根误差 (deg) | ✅ |
| Yaw Rate RMS Error | 偏航角速率跟踪误差 (deg/s) | ✅ |
| Yaw Rate Max Error | 偏航角速率最大偏差 (deg/s) | ✅ |
| Yaw Rate Oscillation | 偏航振荡频率 (Hz) | ✅ |
| Servo Activity | 舵面活动量 (sum of abs changes per sec) | ⚠️ 过小=不响应，过大=振荡 |
| Hinge Angle RMS | 铰链偏角均方根 (deg) | ✅ (有修正时) |

## 自定义扫描

编辑 `sweep_config.json` 添加你的参数组合:

```json
{
    "name": "my_custom_test",
    "description": "Roll rate P sweep",
    "params": {
        "FW_RR_P": 0.5,
        "FW_RR_I": 0.3,
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
