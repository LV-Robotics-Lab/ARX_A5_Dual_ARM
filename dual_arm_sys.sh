#!/bin/bash

# ================== Path Configuration ==================
PROJ_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
DATA_COLLECTION_DIR="$PROJ_DIR/data_collection"
DATA_DIR="$HOME/workspace/raw_data"

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

if [ ! -d "$DATA_DIR" ]; then
    echo -e "${YELLOW}数据存储目录不存在,创建中 / Data directory does not exist, creating: $DATA_DIR${NC}"
    mkdir -p "$DATA_DIR" || { echo -e "${RED}创建失败 / Failed to create${NC}"; exit 1; }
fi

# ================== Command / Title Configuration ==================
# Use script filenames as the pkill match key, so `pkill -f` can directly hit the python process
DARM_SCRIPT="dual_arm_ctrl.py"
CAMERA_SCRIPT="realsense_pub_node.py"
RECORD_SCRIPT="data_record.py"

DARM_CMD="python ${DARM_SCRIPT}"
CAMERA_CMD="python ${CAMERA_SCRIPT}"
ROS_CMD="roscore"

# Terminal titles (used by gnome-terminal and as pkill window match keys)
ROSCORE_TITLE="arx_roscore"
CAMERA_TITLE="arx_camera_pub"
DARM_TITLE="arx_dual_arm_pub"
RECORD_TITLE="arx_data_record"

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

# ================== Stage Functions ==================

start_teach_env() {
    echo -e "${GREEN}=== 启动示教数采环境 / Starting teaching environment ===${NC}"

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

    local traj_number
    traj_number=$(find "$DATA_DIR" -mindepth 1 -maxdepth 1 -type d 2>/dev/null | wc -l)
    echo -e "${BLUE}本次轨迹编号 / Trajectory number: $traj_number${NC}"

    local record_cmd="python ${RECORD_SCRIPT} --root_dir=${DATA_DIR} --traj_number=${traj_number}"

    spawn_terminal "$DARM_TITLE" "$DARM_CMD" "$DATA_COLLECTION_DIR"
    sleep 3

    spawn_terminal "$RECORD_TITLE" "$record_cmd" "$DATA_COLLECTION_DIR"

    echo -e "${GREEN}录制中,选择 [3] 停止录制 / Recording in progress... Choose [3] to stop.${NC}"
}

stop_record() {
    echo -e "${GREEN}=== 停止录制 / Stop recording ===${NC}"

    # 1. Stop subscriber first — must wait long enough for pickle.dump to finish.
    #    A single trajectory can dump several GB of image+depth; killing partway
    #    leaves truncated .pkl files. Default 120s; override via RECORD_TIMEOUT.
    kill_by_cmd "$RECORD_SCRIPT" "数据订阅 / data subscriber" "${RECORD_TIMEOUT:-120}"

    # 2. Close subscriber window
    close_terminal_by_title "$RECORD_TITLE"

    # 3. Stop arm publisher (gives SDK time for DisableMotor / clean CAN close)
    kill_by_cmd "$DARM_SCRIPT" "机械臂发布 / arm publisher" "${DARM_TIMEOUT:-10}"

    # 4. Close arm publisher window
    close_terminal_by_title "$DARM_TITLE"

    echo -e "${GREEN}录制已停止,数据已保存 / Recording stopped, data saved${NC}"
}

close_all() {
    echo -e "${YELLOW}正在关闭所有进程和终端 / Closing all processes and terminals...${NC}"

    # Order: subscriber (saves data) -> arm -> camera -> roscore
    kill_by_cmd "$RECORD_SCRIPT" "数据订阅 / data subscriber" "${RECORD_TIMEOUT:-120}"
    kill_by_cmd "$DARM_SCRIPT" "机械臂发布 / arm publisher" "${DARM_TIMEOUT:-10}"
    kill_by_cmd "$CAMERA_SCRIPT" "相机发布 / camera publisher" 3
    kill_by_cmd "$ROS_CMD" "roscore" 3

    # Close all terminal windows
    for title in "$ROSCORE_TITLE" "$CAMERA_TITLE" "$DARM_TITLE" "$RECORD_TITLE"; do
        close_terminal_by_title "$title"
    done

    echo -e "${GREEN}系统已安全退出 / System exited safely${NC}"
}

visualize_episode() {
    echo -e "${GREEN}=== 可视化轨迹 / Visualize episode ===${NC}"
    echo -n "轨迹编号(留空 = 最近一条) / Episode number (blank = latest): "
    read -r ep_input

    local ep_dir
    if [ -z "$ep_input" ]; then
        # Pick the highest-numbered episode dir under $DATA_DIR (sort -V handles 0009 vs 0010).
        ep_dir=$(find "$DATA_DIR" -mindepth 1 -maxdepth 1 -type d 2>/dev/null | sort -V | tail -1)
        if [ -z "$ep_dir" ]; then
            echo -e "${RED}$DATA_DIR 下没有任何采集数据 / no episodes in $DATA_DIR${NC}"
            return 1
        fi
        echo -e "${BLUE}使用最近一条 / Using latest: $(basename "$ep_dir")${NC}"
    else
        local ep_padded
        if ! ep_padded=$(printf "%04d" "$ep_input" 2>/dev/null); then
            echo -e "${RED}无效编号 / invalid number: $ep_input${NC}"
            return 1
        fi
        ep_dir="$DATA_DIR/$ep_padded"
        if [ ! -d "$ep_dir" ]; then
            echo -e "${RED}目录不存在 / dir not found: $ep_dir${NC}"
            return 1
        fi
    fi

    local title="arx_viz_$(basename "$ep_dir")"
    local cmd="python visualize_episode.py '$ep_dir'"
    spawn_terminal "$title" "$cmd" "$DATA_COLLECTION_DIR"
}

show_menu() {
    clear
    echo -e "${GREEN}=== ARX 双臂数据采集系统 / ARX Dual-Arm Data Collection System ===${NC}"
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