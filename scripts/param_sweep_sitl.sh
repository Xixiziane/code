#!/bin/bash
# ============================================================================
# param_sweep_sitl.sh — ChainWing 自动参数扫描脚本
# ============================================================================
#
# 功能:
#   1. 读取 sweep_config.json 中的参数组合
#   2. 对每组参数: 启动 GZ SITL → 设参 → 起飞 → 飞行 → 降落 → 保存日志
#   3. 所有日志保存到 sweep_logs/ 目录，带描述性文件名
#
# 用法:
#   cd PX4_test
#   # 首次使用需先编译:
#   DONT_RUN=1 make px4_sitl_default
#   # 运行扫描:
#   bash scripts/param_sweep_sitl.sh [sweep_config.json] [--run 3] [--phase B_yaw_P]
#
# 依赖: jq (JSON parser), PX4 SITL 已编译
# ============================================================================

set -euo pipefail

# ── 颜色输出 ────────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; NC='\033[0m'

log_info()  { echo -e "${CYAN}[INFO]${NC}  $(date +%H:%M:%S) $*"; }
log_ok()    { echo -e "${GREEN}[OK]${NC}    $(date +%H:%M:%S) $*"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC}  $(date +%H:%M:%S) $*"; }
log_err()   { echo -e "${RED}[ERROR]${NC} $(date +%H:%M:%S) $*"; }

# ── 路径设置 ────────────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PX4_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
BUILD_DIR="${PX4_DIR}/build/px4_sitl_default"
ROOTFS="${BUILD_DIR}/rootfs"
LOG_OUTPUT_DIR="${PX4_DIR}/sweep_logs"

# ── 参数解析 ────────────────────────────────────────────────────────────────
CONFIG_FILE="${1:-${SCRIPT_DIR}/sweep_config.json}"
SINGLE_RUN=""
PHASE_FILTER=""

shift || true
while [[ $# -gt 0 ]]; do
    case "$1" in
        --run) SINGLE_RUN="$2"; shift 2 ;;
        --phase) PHASE_FILTER="$2"; shift 2 ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

# ── 前置检查 ────────────────────────────────────────────────────────────────
if ! command -v jq &>/dev/null; then
    log_err "需要 jq (JSON parser)。安装: sudo apt install jq"
    exit 1
fi

if [ ! -f "$CONFIG_FILE" ]; then
    log_err "配置文件不存在: $CONFIG_FILE"
    exit 1
fi

if [ ! -x "${BUILD_DIR}/bin/px4" ]; then
    log_err "PX4 SITL 未编译。请先运行: DONT_RUN=1 make px4_sitl_default"
    exit 1
fi

# ── 读取配置 ────────────────────────────────────────────────────────────────
MODEL=$(jq -r '.sitl.model' "$CONFIG_FILE")
WORLD=$(jq -r '.sitl.world' "$CONFIG_FILE")
HEADLESS=$(jq -r '.sitl.headless' "$CONFIG_FILE")
BOOT_WAIT=$(jq -r '.sitl.boot_wait_sec' "$CONFIG_FILE")
FLIGHT_DURATION=$(jq -r '.sitl.flight_duration_sec' "$CONFIG_FILE")
LAND_WAIT=$(jq -r '.sitl.land_wait_sec' "$CONFIG_FILE")
NUM_RUNS=$(jq '.runs | length' "$CONFIG_FILE")

mkdir -p "$LOG_OUTPUT_DIR"

log_info "===== ChainWing 参数扫描 ====="
log_info "配置文件: $CONFIG_FILE"
log_info "模型: $MODEL | 世界: $WORLD | 无头模式: $HEADLESS"
log_info "飞行时长: ${FLIGHT_DURATION}s | 总轮次: $NUM_RUNS"
log_info "日志输出: $LOG_OUTPUT_DIR"
echo ""

# ── 进程清理函数 ────────────────────────────────────────────────────────────
PX4_PID=""
KEEPER_PID=""
FIFO_PATH=""

cleanup() {
    log_info "清理进程..."

    # 关闭 FIFO keeper
    if [ -n "$KEEPER_PID" ] && kill -0 "$KEEPER_PID" 2>/dev/null; then
        kill "$KEEPER_PID" 2>/dev/null || true
        wait "$KEEPER_PID" 2>/dev/null || true
    fi

    # 关闭 PX4
    if [ -n "$PX4_PID" ] && kill -0 "$PX4_PID" 2>/dev/null; then
        kill "$PX4_PID" 2>/dev/null || true
        sleep 2
        # 强制终止
        if kill -0 "$PX4_PID" 2>/dev/null; then
            kill -9 "$PX4_PID" 2>/dev/null || true
        fi
        wait "$PX4_PID" 2>/dev/null || true
    fi

    # 清理 Gazebo 进程
    # gz sim 产生的 ruby 子进程通过精确匹配清理
    for pid in $(ps -eo pid,comm,args 2>/dev/null | \
                 grep -E 'gz[[:space:]].*sim|ruby.*ign|ruby.*gz' | \
                 grep -v grep | awk '{print $1}'); do
        kill "$pid" 2>/dev/null || true
    done

    sleep 3

    # 清理 FIFO
    if [ -n "$FIFO_PATH" ] && [ -p "$FIFO_PATH" ]; then
        rm -f "$FIFO_PATH"
    fi

    PX4_PID=""
    KEEPER_PID=""
}

trap cleanup EXIT

# ── 发送 PX4 命令 ───────────────────────────────────────────────────────────
send_cmd() {
    local cmd="$1"
    local wait="${2:-1}"
    if [ -n "$FIFO_PATH" ] && [ -p "$FIFO_PATH" ]; then
        echo "$cmd" > "$FIFO_PATH"
        sleep "$wait"
    fi
}

# ── 查找最新日志文件 ────────────────────────────────────────────────────────
find_latest_log() {
    local marker_time="$1"
    find "${ROOTFS}/log" -name "*.ulg" -newer "$marker_time" 2>/dev/null | sort | tail -1
}

# ── 单次运行 ────────────────────────────────────────────────────────────────
run_single() {
    local run_index="$1"
    local run_name=$(jq -r ".runs[$run_index].name" "$CONFIG_FILE")
    local run_desc=$(jq -r ".runs[$run_index].description" "$CONFIG_FILE")
    local run_phase=$(jq -r ".runs[$run_index].phase // \"\"" "$CONFIG_FILE")
    local param_keys=$(jq -r ".runs[$run_index].params | keys[]" "$CONFIG_FILE")

    log_info "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    log_info "轮次 $((run_index + 1))/${NUM_RUNS}: ${run_name}  [阶段: ${run_phase}]"
    log_info "说明: ${run_desc}"
    log_info "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

    # 清理上一轮
    cleanup

    # 清除旧参数文件 (确保使用 airframe defaults)
    rm -f "${ROOTFS}/parameters.bson" "${ROOTFS}/parameters_backup.bson" 2>/dev/null || true

    # 创建时间标记 (用于找到本轮日志)
    local marker="/tmp/sweep_marker_$$"
    touch "$marker"
    sleep 1

    # 创建 FIFO
    FIFO_PATH="/tmp/px4_sweep_fifo_$$"
    rm -f "$FIFO_PATH"
    mkfifo "$FIFO_PATH"

    # 保持 FIFO 打开 (防止 PX4 读到 EOF)
    tail -f /dev/null > "$FIFO_PATH" &
    KEEPER_PID=$!

    # 设置环境变量
    export PX4_SIM_MODEL="$MODEL"
    export PX4_GZ_WORLD="$WORLD"
    if [ "$HEADLESS" = "true" ]; then
        export HEADLESS=1
    fi

    # 启动 PX4
    log_info "启动 PX4 SITL + Gazebo..."
    cd "$ROOTFS"
    "${BUILD_DIR}/bin/px4" -s etc/init.d-posix/rcS < "$FIFO_PATH" \
        > "/tmp/px4_sweep_stdout_$$.log" 2>&1 &
    PX4_PID=$!

    log_info "PX4 PID: $PX4_PID, 等待启动 ${BOOT_WAIT}s..."
    sleep "$BOOT_WAIT"

    # 检查 PX4 是否还活着
    if ! kill -0 "$PX4_PID" 2>/dev/null; then
        log_err "PX4 启动失败！查看 /tmp/px4_sweep_stdout_$$.log"
        rm -f "$marker"
        return 1
    fi
    log_ok "PX4 启动成功"

    # 设置参数
    log_info "设置参数..."
    for key in $param_keys; do
        local value=$(jq -r ".runs[$run_index].params[\"$key\"]" "$CONFIG_FILE")
        send_cmd "param set $key $value" 0.5
        log_info "  param set $key $value"
    done
    sleep 2

    # 解锁
    log_info "解锁并起飞..."
    send_cmd "commander arm" 3

    # 检查是否解锁成功 (通过检查 PX4 是否还活)
    if ! kill -0 "$PX4_PID" 2>/dev/null; then
        log_err "PX4 在解锁时崩溃"
        rm -f "$marker"
        return 1
    fi

    # 起飞
    send_cmd "commander takeoff" 5
    log_ok "已发送起飞命令"

    # 等待飞行
    log_info "飞行中... (${FLIGHT_DURATION}s)"
    local elapsed=0
    while [ $elapsed -lt "$FLIGHT_DURATION" ]; do
        if ! kill -0 "$PX4_PID" 2>/dev/null; then
            log_err "PX4 在飞行中崩溃 (${elapsed}s)"
            rm -f "$marker"
            return 1
        fi
        sleep 10
        elapsed=$((elapsed + 10))
        echo -ne "  进度: ${elapsed}/${FLIGHT_DURATION}s\r"
    done
    echo ""

    # 降落
    log_info "降落..."
    send_cmd "commander land" "$LAND_WAIT"
    send_cmd "commander disarm" 3
    log_ok "已降落"

    # 关闭 PX4 (让 logger 正常刷写)
    send_cmd "shutdown" 5

    # 等待 PX4 退出
    wait "$PX4_PID" 2>/dev/null || true
    PX4_PID=""
    sleep 3

    # 收集日志
    local latest_log=$(find_latest_log "$marker")
    rm -f "$marker"

    if [ -n "$latest_log" ]; then
        local output_name="run_$(printf '%02d' $((run_index + 1)))_${run_name}.ulg"
        cp "$latest_log" "${LOG_OUTPUT_DIR}/${output_name}"
        local log_size=$(du -h "${LOG_OUTPUT_DIR}/${output_name}" | cut -f1)
        log_ok "日志已保存: ${output_name} (${log_size})"

        # 同时保存参数记录
        jq ".runs[$run_index]" "$CONFIG_FILE" > \
            "${LOG_OUTPUT_DIR}/run_$(printf '%02d' $((run_index + 1)))_${run_name}_params.json"
    else
        log_warn "未找到日志文件！检查 PX4 输出: /tmp/px4_sweep_stdout_$$.log"
    fi

    # 清理 Gazebo
    cleanup
    sleep 5

    log_ok "轮次 $((run_index + 1)) 完成"
    echo ""
}

# ── 主循环 ──────────────────────────────────────────────────────────────────
START_TIME=$(date +%s)

if [ -n "$SINGLE_RUN" ]; then
    # 单轮运行
    run_idx=$((SINGLE_RUN - 1))
    if [ $run_idx -lt 0 ] || [ $run_idx -ge "$NUM_RUNS" ]; then
        log_err "无效的轮次编号: $SINGLE_RUN (有效范围: 1-$NUM_RUNS)"
        exit 1
    fi
    run_single "$run_idx"
elif [ -n "$PHASE_FILTER" ]; then
    # 按阶段过滤运行
    log_info "过滤阶段: $PHASE_FILTER"
    phase_count=0
    for ((i = 0; i < NUM_RUNS; i++)); do
        run_phase=$(jq -r ".runs[$i].phase // \"\"" "$CONFIG_FILE")
        if [[ "$run_phase" == "$PHASE_FILTER" ]]; then
            phase_count=$((phase_count + 1))
            run_single "$i" || {
                log_warn "轮次 $((i + 1)) 失败，继续下一轮..."
                sleep 5
            }
        fi
    done
    if [ $phase_count -eq 0 ]; then
        log_err "未找到匹配阶段: $PHASE_FILTER"
        log_info "可用阶段: $(jq -r '[.runs[].phase // ""] | unique | .[]' "$CONFIG_FILE" | tr '\n' ' ')"
        exit 1
    fi
else
    # 全部运行
    for ((i = 0; i < NUM_RUNS; i++)); do
        run_single "$i" || {
            log_warn "轮次 $((i + 1)) 失败，继续下一轮..."
            sleep 5
        }
    done
fi

END_TIME=$(date +%s)
TOTAL_MIN=$(( (END_TIME - START_TIME) / 60 ))

echo ""
log_ok "===== 扫描完成 ====="
log_ok "总耗时: ${TOTAL_MIN} 分钟"
log_ok "日志目录: ${LOG_OUTPUT_DIR}/"
echo ""
log_info "下一步: 运行分析脚本生成对比图:"
log_info "  python3 scripts/analyze_param_sweep.py sweep_logs/"
