#!/bin/bash

# ================== Path Configuration ==================
PROJ_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
DATA_COLLECTION_DIR="$PROJ_DIR/data_collection"
ARX_CAN_DIR="$PROJ_DIR/A5/ARX_CAN"

# Machine-specific, non-secret hardware mapping. Copy config/arx.env.example to
# config/arx.local.env and keep that local file out of Git.
ARX_CONFIG_FILE="${ARX_CONFIG_FILE:-$PROJ_DIR/config/arx.local.env}"
if [ -f "$ARX_CONFIG_FILE" ]; then
    set -a
    # shellcheck disable=SC1090
    source "$ARX_CONFIG_FILE"
    set +a
fi

# CAN interfaces for left/right arms. Must match dual_arm_ctrl.py defaults.
LEFT_CAN="${ARX_LEFT_CAN:-${LEFT_CAN:-can1}}"
RIGHT_CAN="${ARX_RIGHT_CAN:-${RIGHT_CAN:-can3}}"

# Task name becomes a subdir so different tasks don't share traj numbering.
# Override with TASK_NAME=... ./dual_arm_sys.sh if collecting a different task.
TASK_NAME="${ARX_TASK_NAME:-${TASK_NAME:-egg_to_bowl}}"
DATA_ROOT="${ARX_DATA_ROOT:-$HOME/workspace/raw_data}"
DATA_DIR="$DATA_ROOT/$TASK_NAME"

# Failed episodes go here instead of being deleted. Underscore prefix means
# the success traj_number scan (regex .*/[0-9]+$) ignores it, so the next [2]
# still re-uses the slot of the failed run.
FAILED_DIR="$DATA_DIR/_failed"

# Runtime state — the dir created by start_record so stop_record can prompt
# y/n and delete it if the operator marks the episode invalid.
CURRENT_TRAJ_DIR=""

# Conda env used by all sub-processes (roscore / python scripts).
# Override with CONDA_ENV=... ./dual_arm_sys.sh if needed.
CONDA_BASE="${CONDA_BASE:-$HOME/miniconda3}"
CONDA_ENV="${CONDA_ENV:-robo_ctrl}"

# ================== Color Definitions ==================
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# ================== Initial Checks ==================
if [ ! -d "$DATA_COLLECTION_DIR" ]; then
    echo -e "${RED}错误 / Error: 数据采集脚本目录不存在 / data collection directory not found: $DATA_COLLECTION_DIR${NC}"
    exit 1
fi

if [[ ! "$LEFT_CAN" =~ ^[A-Za-z0-9_.:-]+$ ]] || [[ ! "$RIGHT_CAN" =~ ^[A-Za-z0-9_.:-]+$ ]]; then
    echo -e "${RED}错误 / Error: invalid CAN interface name${NC}"
    exit 1
fi

if [ "$LEFT_CAN" = "$RIGHT_CAN" ]; then
    echo -e "${RED}错误 / Error: left and right arms cannot share one CAN interface${NC}"
    exit 1
fi

if [[ ! "$TASK_NAME" =~ ^[A-Za-z0-9_.:-]+$ ]] || [ "$TASK_NAME" = "." ] || [ "$TASK_NAME" = ".." ]; then
    echo -e "${RED}错误 / Error: invalid task name: $TASK_NAME${NC}"
    exit 1
fi

if [ ! -d "$DATA_DIR" ]; then
    echo -e "${YELLOW}数据存储目录不存在,创建中 / Data directory does not exist, creating: $DATA_DIR${NC}"
    mkdir -p "$DATA_DIR" || { echo -e "${RED}创建失败 / Failed to create${NC}"; exit 1; }
fi

# ================== Command / Title Configuration ==================
# Use script filenames as the pkill match key, so `pkill -f` can directly hit the python process
DARM_SCRIPT="dual_arm_ctrl.py"
CAMERA_SCRIPT="realsense_pub_node.py"
RECORD_SCRIPT="data_record.py"

DARM_CMD="python ${DARM_SCRIPT} --left-can ${LEFT_CAN} --right-can ${RIGHT_CAN} --execute --clearance-confirmed --estop-ready --exclusive-control-confirmed"
CAMERA_CMD="python ${CAMERA_SCRIPT}"
ROS_CMD="roscore"

# Terminal titles (used by gnome-terminal and as pkill window match keys)
ROSCORE_TITLE="arx_roscore"
CAMERA_TITLE="arx_camera_pub"
DARM_TITLE="arx_dual_arm_pub"
RECORD_TITLE="arx_data_record"
CAN_TITLE_PREFIX="arx_canpub"   # final title is arx_canpub_can1 / arx_canpub_can3

# ================== Helper Functions ==================

# Spawn a gnome-terminal running the given command.
# Args: $1=title  $2=command  $3=cwd
spawn_terminal() {
    local title="$1"
    local cmd="$2"
    local cwd="${3:-$PWD}"

    if [ ! -d "$cwd" ]; then
        echo -e "${RED}错误 / Error: 工作目录不存在 / working directory not found: $cwd${NC}"
        return 1
    fi

    echo -e "${YELLOW}[启动 / Start] ${BLUE}${title}${NC}: ${GREEN}${cmd}${NC}"

    # Trick: inject the title as a dummy variable into the bash command line.
    # gnome-terminal's own cmdline does not contain the title (only the --title arg
    # value is consumed), but the bash -c child it spawns DOES contain TITLE_TAG=xxx,
    # so `pkill -f "TITLE_TAG=xxx"` can target that specific window's bash later.
    gnome-terminal --title="$title" -- bash -c \
        "TITLE_TAG=$title; \
         source '$CONDA_BASE/etc/profile.d/conda.sh' && conda activate '$CONDA_ENV' && \
         cd '$cwd' && \
         echo -e '${YELLOW}已启动 / Started: ${title}${NC} (env: $CONDA_ENV)' && \
         $cmd; \
         echo '进程已退出,按任意键关闭窗口 / Process exited. Press any key to close window...'; \
         read -n 1"

    sleep 0.3
    return 0
}

# Gracefully terminate processes matched by command pattern (SIGINT first, escalate if needed).
# Args: $1=cmd_pattern  $2=name  $3=timeout(seconds)
kill_by_cmd() {
    local pattern="$1"
    local name="$2"
    local timeout="${3:-10}"

    local pids
    pids=$(pgrep -f "$pattern" 2>/dev/null)

    if [ -z "$pids" ]; then
        echo -e "${YELLOW}$name 未运行 / not running${NC}"
        return 0
    fi

    echo -e "${YELLOW}发送 SIGINT 给 / Sending SIGINT to $name (PIDs: $pids)${NC}"
    pkill -INT -f "$pattern" 2>/dev/null

    # Wait for exit
    local i=0
    while [ $i -lt "$timeout" ]; do
        if ! pgrep -f "$pattern" >/dev/null 2>&1; then
            echo -e "${GREEN}$name 已正常退出 / exited cleanly${NC}"
            return 0
        fi
        sleep 1
        i=$((i + 1))
        echo -ne "  等待 / Waiting for $name 退出 / to exit ($i/$timeout)...\r"
    done
    echo

    # Still alive: SIGTERM
    if pgrep -f "$pattern" >/dev/null 2>&1; then
        echo -e "${RED}$name 未在 ${timeout}s 内退出,发送 SIGTERM / did not exit within ${timeout}s, sending SIGTERM${NC}"
        pkill -TERM -f "$pattern" 2>/dev/null
        sleep 1
    fi

    # Still alive: SIGKILL
    if pgrep -f "$pattern" >/dev/null 2>&1; then
        echo -e "${RED}$name 仍未退出,发送 SIGKILL / still alive, sending SIGKILL${NC}"
        pkill -KILL -f "$pattern" 2>/dev/null
    fi
}

# Close terminal window by matching the TITLE_TAG marker on its bash child.
# Args: $1=title
close_terminal_by_title() {
    local title="$1"
    # Kill the bash process tagged with TITLE_TAG=xxx; the window closes when bash exits.
    pkill -f "TITLE_TAG=${title}" 2>/dev/null

    # Fallback: use wmctrl to close by window title if available.
    if command -v wmctrl >/dev/null 2>&1; then
        local windows
        windows=$(wmctrl -l 2>/dev/null | awk -v t="$title" '$0 ~ t {print $1}')
        for w in $windows; do
            wmctrl -ic "$w" 2>/dev/null
        done
    fi
}

is_roscore_running() {
    pgrep -f "$ROS_CMD" >/dev/null 2>&1
}

check_wrapper_install() {
    local env_python="$CONDA_BASE/envs/$CONDA_ENV/bin/python"
    if [ ! -x "$env_python" ]; then
        echo -e "${RED}Conda 环境不存在 / environment not found: $CONDA_ENV${NC}"
        return 1
    fi
    if ! "$env_python" -c "import arx_wrapper" >/dev/null 2>&1; then
        echo -e "${RED}arx_wrapper 尚未安装到 $CONDA_ENV / is not installed${NC}"
        echo -e "${YELLOW}运行 / Run: conda activate $CONDA_ENV && pip install -e '$PROJ_DIR'${NC}"
        return 1
    fi
}

confirm_teaching_motion() {
    echo -e "${YELLOW}即将把双臂切换到重力补偿模式 / About to enable gravity compensation.${NC}"
    echo "  - 双臂工作区已清空 / workspace is clear"
    echo "  - 急停在手边 / e-stop is ready"
    echo "  - 无其他控制进程 / no competing controller is running"
    echo -n "输入 TEACH 继续 / Type TEACH to continue: "
    local confirmation
    read -r confirmation
    if [ "$confirmation" != "TEACH" ]; then
        echo -e "${YELLOW}已取消,未连接机械臂 / cancelled before arm connection${NC}"
        return 1
    fi
}

# Bring a CAN interface up via the ARX-provided arx_canN watchdog.
# Args: $1=can name (e.g. can1)
# Note: arx_canN is a LONG-RUNNING daemon (while-true loop that monitors and
# auto-restarts the interface on disconnect). It must run in its own terminal
# so the menu shell doesn't block. We skip relaunch if the interface is already
# UP AND a watchdog already owns it.
ensure_can_up() {
    local can_name="$1"
    local title="${CAN_TITLE_PREFIX}_${can_name}"

    # Already up + watchdog running → nothing to do.
    if ip link show "$can_name" 2>/dev/null | grep -q '<.*UP.*>'; then
        if pgrep -f "TITLE_TAG=$title" >/dev/null 2>&1 \
           || pgrep -f "ARX_CAN/arx_$can_name" >/dev/null 2>&1; then
            echo -e "${GREEN}$can_name 已就绪 / already up with watchdog${NC}"
            return 0
        fi
        # UP but no watchdog (rare — manual ip link, or stale state). Still
        # OK for our purposes; warn and continue.
        echo -e "${YELLOW}$can_name 已 UP 但无守护进程,跳过启动 / UP without watchdog, skipping launch${NC}"
        return 0
    fi

    local script="$ARX_CAN_DIR/arx_$can_name"
    if [ ! -x "$script" ]; then
        echo -e "${RED}启动脚本不存在 / launcher missing: $script${NC}"
        return 1
    fi

    echo -e "${YELLOW}启动 / Starting $can_name ...${NC}"

    # Launch the watchdog in its own terminal — no conda env needed; the script
    # only uses bash + sudo + slcand + ip. We invoke with sudo -n; if NOPASSWD
    # isn't configured the inner bash will exit immediately and the window will
    # show the prompt asking the user to press any key to close.
    gnome-terminal --title="$title" -- bash -c \
        "TITLE_TAG=$title; \
         echo -e '${YELLOW}CAN watchdog: $can_name (sudo -n)${NC}'; \
         sudo -n '$script'; \
         echo 'watchdog exited. Press any key to close...'; \
         read -n 1" >/dev/null 2>&1 &

    # Poll for the interface to come UP. start_can in the daemon takes ~0.5s.
    local i=0
    while [ $i -lt 10 ]; do
        sleep 0.5
        if ip link show "$can_name" 2>/dev/null | grep -q '<.*UP.*>'; then
            echo -e "${GREEN}$can_name 启动成功 / up${NC}"
            return 0
        fi
        i=$((i + 1))
    done
    echo -e "${RED}$can_name 5s 内未 UP / not UP within 5s${NC}"
    echo -e "${YELLOW}查看 / check window '$title' for the watchdog output${NC}"
    return 1
}

# ================== Stage Functions ==================

start_teach_env() {
    echo -e "${GREEN}=== 启动示教数采环境 / Starting teaching environment ===${NC}"

    if ! check_wrapper_install; then
        return 1
    fi

    # Bring CAN interfaces up first — without these the arms in [2] are
    # reachable in the SDK but commands (incl. gravity comp) never get to
    # the firmware, so the arms feel rigid.
    if ! ensure_can_up "$LEFT_CAN"; then
        echo -e "${RED}左臂 CAN ($LEFT_CAN) 未就绪,中止 / left CAN not ready, aborting${NC}"
        return 1
    fi
    if ! ensure_can_up "$RIGHT_CAN"; then
        echo -e "${RED}右臂 CAN ($RIGHT_CAN) 未就绪,中止 / right CAN not ready, aborting${NC}"
        return 1
    fi

    if is_roscore_running; then
        echo -e "${YELLOW}roscore 已在运行,跳过启动 / already running, skipping${NC}"
    else
        spawn_terminal "$ROSCORE_TITLE" "$ROS_CMD" "$HOME"
        sleep 3
        if ! is_roscore_running; then
            echo -e "${RED}roscore 启动失败 / failed to start${NC}"
            return 1
        fi
    fi

    spawn_terminal "$CAMERA_TITLE" "$CAMERA_CMD" "$DATA_COLLECTION_DIR"
    sleep 2

    echo -e "${GREEN}示教环境就绪,可选择 [2] 开始录制 / Teaching environment ready. Choose [2] to start recording.${NC}"
}

start_record() {
    echo -e "${GREEN}=== 开始录制数据 / Start recording ===${NC}"

    if ! is_roscore_running; then
        echo -e "${RED}错误:roscore 未运行,请先选择 [1] / Error: roscore is not running. Choose [1] first.${NC}"
        return 1
    fi

    if pgrep -f "$RECORD_SCRIPT" >/dev/null 2>&1; then
        echo -e "${RED}已有录制在运行,请先选 [3] 停止 / A recording session is already running. Choose [3] to stop first.${NC}"
        return 1
    fi

    if ! confirm_teaching_motion; then
        return 1
    fi

    # Use max(existing)+1 instead of count(existing). Counting breaks the
    # moment an episode is deleted (failed/丢弃) — the next traj_number would
    # collide with an existing dir and overwrite valid data.
    local traj_number
    traj_number=$(find "$DATA_DIR" -mindepth 1 -maxdepth 1 -type d -regex '.*/[0-9]+$' \
        -printf '%f\n' 2>/dev/null | sort -n | tail -1)
    if [ -z "$traj_number" ]; then
        traj_number=0
    else
        traj_number=$((10#$traj_number + 1))
    fi

    local traj_padded
    traj_padded=$(printf "%04d" "$traj_number")
    CURRENT_TRAJ_DIR="$DATA_DIR/$traj_padded"
    echo -e "${BLUE}本次轨迹 / Trajectory: $TASK_NAME/$traj_padded${NC}"

    local record_cmd="python ${RECORD_SCRIPT} --root_dir=${DATA_DIR} --traj_number=${traj_number}"

    spawn_terminal "$DARM_TITLE" "$DARM_CMD" "$DATA_COLLECTION_DIR"
    sleep 3

    spawn_terminal "$RECORD_TITLE" "$record_cmd" "$DATA_COLLECTION_DIR"

    echo -e "${GREEN}录制中,选择 [3] 停止录制 / Recording in progress... Choose [3] to stop.${NC}"
}

stop_record() {
    echo -e "${GREEN}=== 停止录制 / Stop recording ===${NC}"

    # 1. Stop subscriber first (let it finalize streaming writers)
    # 30s is generous — the streaming writer just needs to close mp4/h5 handles,
    # which usually finishes in well under 1s. The headroom is for very long
    # episodes where mp4 trailer write can take a few seconds.
    kill_by_cmd "$RECORD_SCRIPT" "数据订阅 / data subscriber" 30

    # 2. Close subscriber window
    close_terminal_by_title "$RECORD_TITLE"

    # 3. Stop arm publisher
    kill_by_cmd "$DARM_SCRIPT" "机械臂发布 / arm publisher" 3

    # 4. Close arm publisher window
    close_terminal_by_title "$DARM_TITLE"

    echo -e "${GREEN}录制已停止,数据已写入 / Recording stopped, data flushed${NC}"

    # 5. Ask operator whether this episode succeeded. SOP requires keeping failed
    # runs OUT of training data, but we still archive them under _failed/ for
    # failure analysis + debugging the collection setup (deleted data can't be
    # recovered if we later realize the judgment was wrong).
    if [ -n "$CURRENT_TRAJ_DIR" ] && [ -d "$CURRENT_TRAJ_DIR" ]; then
        echo
        echo -e "${YELLOW}本次轨迹 / This episode: ${BLUE}$CURRENT_TRAJ_DIR${NC}"
        echo -n "是否保留为有效数据? / Keep as valid? [Y/n]: "
        read -r keep
        case "$keep" in
            n|N|no|NO)
                mkdir -p "$FAILED_DIR"
                # Sequential f0000, f0001, ... numbering inside _failed/
                # (independent of which main-dir slot was attempted).
                local fail_num fail_padded failed_target traj_name
                traj_name=$(basename "$CURRENT_TRAJ_DIR")
                fail_num=$(find "$FAILED_DIR" -mindepth 1 -maxdepth 1 -type d -regex '.*/f[0-9]+$' \
                    -printf '%f\n' 2>/dev/null | sed 's/^f//' | sort -n | tail -1)
                if [ -z "$fail_num" ]; then
                    fail_num=0
                else
                    fail_num=$((10#$fail_num + 1))
                fi
                fail_padded=$(printf "f%04d" "$fail_num")
                failed_target="$FAILED_DIR/$fail_padded"
                if mv "$CURRENT_TRAJ_DIR" "$failed_target" 2>/dev/null; then
                    echo -e "${YELLOW}已移到失败档案 / Moved to failed archive:${NC}"
                    echo -e "  ${BLUE}$failed_target${NC}  (原槽位 / from slot $traj_name)"
                    echo -e "${YELLOW}编号 $traj_name 将在下次 [2] 时复用 / slot $traj_name will be reused on next [2]${NC}"
                else
                    echo -e "${RED}移动失败,保留原位 / mv failed, keeping in place: $CURRENT_TRAJ_DIR${NC}"
                fi
                ;;
            *)
                echo -e "${GREEN}已保留为有效数据 / Kept as valid: $CURRENT_TRAJ_DIR${NC}"
                ;;
        esac
        CURRENT_TRAJ_DIR=""
    fi
}

close_all() {
    echo -e "${YELLOW}正在关闭所有进程和终端 / Closing all processes and terminals...${NC}"

    # Order: subscriber (saves data) -> arm -> camera -> roscore -> CAN
    # CAN watchdogs go last because the arm publisher needs CAN to send its
    # final shutdown commands.
    kill_by_cmd "$RECORD_SCRIPT" "数据订阅 / data subscriber" 30
    kill_by_cmd "$DARM_SCRIPT" "机械臂发布 / arm publisher" 3
    kill_by_cmd "$CAMERA_SCRIPT" "相机发布 / camera publisher" 1
    kill_by_cmd "$ROS_CMD" "roscore" 1

    # CAN watchdogs (sudo -n started them, so we need sudo to kill them too).
    for can_name in "$LEFT_CAN" "$RIGHT_CAN"; do
        if pgrep -f "ARX_CAN/arx_$can_name" >/dev/null 2>&1; then
            echo -e "${YELLOW}停止 CAN 守护进程 / stopping watchdog: $can_name${NC}"
            sudo -n pkill -f "ARX_CAN/arx_$can_name" 2>/dev/null
            sudo -n pkill -f "arx_can_.*$can_name" 2>/dev/null
            sudo -n pkill -f "slcand.* $can_name\$" 2>/dev/null
        fi
    done

    # Close all terminal windows
    for title in "$ROSCORE_TITLE" "$CAMERA_TITLE" "$DARM_TITLE" "$RECORD_TITLE" \
                 "${CAN_TITLE_PREFIX}_${LEFT_CAN}" "${CAN_TITLE_PREFIX}_${RIGHT_CAN}"; do
        close_terminal_by_title "$title"
    done

    echo -e "${GREEN}系统已安全退出 / System exited safely${NC}"
}

visualize_episode() {
    echo -e "${GREEN}=== 可视化轨迹 / Visualize episode ===${NC}"
    echo -n "轨迹编号(留空 = 最近一条有效;前缀 f = 失败档案,如 f0007) / Episode (blank=latest valid; f<N>=failed): "
    read -r ep_input

    local ep_dir
    if [ -z "$ep_input" ]; then
        # Blank → latest valid episode under $DATA_DIR (only NNNN dirs, not _failed/).
        ep_dir=$(find "$DATA_DIR" -mindepth 1 -maxdepth 1 -type d -regex '.*/[0-9]+$' 2>/dev/null | sort -V | tail -1)
        if [ -z "$ep_dir" ]; then
            echo -e "${RED}$DATA_DIR 下没有任何有效轨迹 / no valid episodes${NC}"
            return 1
        fi
        echo -e "${BLUE}使用最近有效轨迹 / Using latest valid: $(basename "$ep_dir")${NC}"
    else
        # Detect "f0007"/"f7" prefix → search failed archive instead of main dir.
        local search_failed=0 raw="$ep_input"
        if [[ "$ep_input" == f* ]]; then
            search_failed=1
            raw="${ep_input#f}"
        fi

        local ep_padded
        if ! ep_padded=$(printf "%04d" "$raw" 2>/dev/null) || ! [[ "$ep_padded" =~ ^[0-9]{4}$ ]]; then
            echo -e "${RED}无效编号 / invalid number: $ep_input${NC}"
            return 1
        fi

        if [ "$search_failed" -eq 1 ]; then
            # f<N> maps directly to _failed/f<NNNN>/
            ep_dir="$FAILED_DIR/f${ep_padded}"
            if [ ! -d "$ep_dir" ]; then
                echo -e "${RED}失败档案不存在 / not found: $ep_dir${NC}"
                return 1
            fi
            echo -e "${YELLOW}失败档案 / Failed archive: $(basename "$ep_dir")${NC}"
        else
            ep_dir="$DATA_DIR/$ep_padded"
            if [ ! -d "$ep_dir" ]; then
                echo -e "${RED}目录不存在 / dir not found: $ep_dir${NC}"
                return 1
            fi
        fi
    fi

    local title="arx_viz_$(basename "$ep_dir")"
    local cmd="python visualize_episode.py '$ep_dir'"
    spawn_terminal "$title" "$cmd" "$DATA_COLLECTION_DIR"
}

show_menu() {
    clear
    echo -e "${GREEN}=== ARX 双臂数据采集系统 / ARX Dual-Arm Data Collection System ===${NC}"
    local n_eps=0 n_failed=0
    if [ -d "$DATA_DIR" ]; then
        n_eps=$(find "$DATA_DIR" -mindepth 1 -maxdepth 1 -type d -regex '.*/[0-9]+$' 2>/dev/null | wc -l)
    fi
    if [ -d "$FAILED_DIR" ]; then
        n_failed=$(find "$FAILED_DIR" -mindepth 1 -maxdepth 1 -type d 2>/dev/null | wc -l)
    fi
    echo -e "${BLUE}任务 / Task: ${TASK_NAME}    有效 / Valid: ${n_eps}    失败 / Failed: ${n_failed}${NC}"
    echo -e "1. 启动示教数采 / Start teaching environment"
    echo -e "2. 开始录制数据 / Start recording"
    echo -e "3. 停止录制数据 / Stop recording"
    echo -e "4. 可视化轨迹 / Visualize episode (Rerun)"
    echo -e "0. 退出并关闭所有终端 / Exit and close all terminals"
    echo -n "请选择 / Select: "
}

trap 'echo; close_all; exit 0' INT TERM

# ================== Main Loop ==================
while true; do
    show_menu
    read -r choice
    case $choice in
        1) start_teach_env ;;
        2) start_record ;;
        3) stop_record ;;
        4) visualize_episode ;;
        0) close_all; exit 0 ;;
        *) echo -e "${RED}无效输入,请重试 / Invalid input, please retry${NC}"; sleep 1; continue ;;
    esac
    echo -e "${YELLOW}按回车继续 / Press Enter to continue...${NC}"
    read -r
done
