# ARX 双臂数据采集系统使用文档

## 概述

该repo是一个交互式 shell 脚本,用于管理 ARX 双臂示教数据采集的完整流程。通过菜单选项控制 ROS、相机、机械臂、数据订阅器的启停,自动管理 gnome-terminal 子窗口,采集数据保存为 pickle 文件。

![双臂](./docs/dual_arm.jpg)
![架构图](./docs/sys.png)

## 安装
请参考 [A5 SDK 安装文档](./A5/README.md) 安装好 ARX A5双臂机械臂。官方仓库请参考：[https://github.com/ARXroboticsX/A5](https://github.com/ARXroboticsX/A5)。另外，你还需要以下必须的软件包：

```bash
pip install opencv-python==4.6.0 pyrealsense2==2.54.2.5684 numpy==1.26.4
```


## 启动

```bash
cd ~/workspace/ARX_ARM/data_collection
./start_collection.sh
```

启动后会进入交互菜单:

```
=== ARX 双臂数据采集系统 / ARX Dual-Arm Data Collection System ===
1. 启动示教数采 / Start teaching environment (roscore + camera)
2. 开始录制数据 / Start recording (arm teaching + data subscriber)
3. 停止录制数据 / Stop recording (save pickle and stop arm)
0. 退出并关闭所有终端 / Exit and close all terminals
请选择 / Select:
```

## 操作步骤

### 步骤 1:首先选择 [1] 启动示教数采

**功能**:启动数据采集所需的基础环境,只需启动一次,后续所有录制都复用。

**会启动的进程和窗口**:

| 弹出窗口标题 | 启动的进程 | 作用 |
|---|---|---|
| `arx_roscore` | `roscore` | ROS Master,所有话题通信的中枢 |
| `arx_camera_pub` | `python realsense_pub_node.py` | RealSense 多相机发布,自动检测连接的相机 |

**发布的话题**:

- `/camera_1_image`、`/camera_1_depth`(每个相机各一对 RGB + Depth)
- `/camera_2_image`、`/camera_2_depth`
- (有几个相机就发几对)

完成后控制台会提示「示教环境就绪」。

### 步骤 2:选择 [2] 开始录制数据

**功能**:启动机械臂示教和数据订阅器,开始记录一条新轨迹。

**前置条件**:必须先执行过步骤 1,否则会提示 roscore 未运行。

**会启动的进程和窗口**:

| 弹出窗口标题 | 启动的进程 | 作用 |
|---|---|---|
| `arx_dual_arm_pub` | `python dual_arm_ctrl.py` | 双臂进入重力补偿模式,实时发布关节状态和末端位姿 |
| `arx_data_record` | `python data_record.py --root_dir=... --traj_number=N` | 订阅所有话题,缓存到内存,等待结束信号统一保存 |

**轨迹编号**:每次按 [2],脚本自动扫描 `$DATA_DIR` 下已有的轨迹目录数量,新轨迹编号 = 已有数量(从 0000 开始递增)。

**发布的话题**(由机械臂发布器):

- `/arx_left/joint_states`、`/arx_left/eef_pose`
- `/arx_right/joint_states`、`/arx_right/eef_pose`

此时手动拖动两条机械臂进行示教,所有数据会被订阅器持续记录到内存。

### 步骤 3:选择 [3] 停止录制数据

**功能**:结束本次录制,保存 pickle,关闭机械臂和录制相关的两个窗口。

**操作流程**(脚本自动执行):

1. 给订阅器发送 `SIGINT`,触发其内部 `signal_handler` → 写入 4 个 pickle:
   - `image.pkl` — 所有相机的 RGB 帧 + 时间戳
   - `depth.pkl` — 所有相机的深度帧 + 时间戳
   - `state.pkl` — 双臂关节状态 + 时间戳
   - `eef_pose.pkl` — 双臂末端位姿 + 时间戳
2. 等待最多 6 秒让订阅器写盘完成
3. 关闭 `arx_data_record` 窗口
4. 给机械臂发布器发送 `SIGINT`,等 3 秒退出
5. 关闭 `arx_dual_arm_pub` 窗口

**保留运行**:`arx_roscore` 和 `arx_camera_pub` 窗口**不会**关闭,可以直接进入下一轮 [2] 录制。

### 循环采集

录制流程可以反复执行:

```
[2] → 拖动示教 → [3] (保存 0000)
[2] → 拖动示教 → [3] (保存 0001)
[2] → 拖动示教 → [3] (保存 0002)
...
```

每次 traj_number 自动递增,数据保存在:

```
$DATA_DIR/0000/
├── image.pkl
├── depth.pkl
├── state.pkl
└── eef_pose.pkl
$DATA_DIR/0001/
...
```

### 选择 [0] 退出并关闭所有终端

**功能**:全部停止,清理所有相关进程和窗口。

**清理顺序**:

1. 数据订阅器(给 6 秒保存数据的时间)
2. 机械臂发布器(3 秒)
3. 相机发布器(1 秒)
4. roscore(1 秒)
5. 关闭所有相关 gnome-terminal 窗口

## 部分技术细节实现

### 进程管理

- 用 **`pkill -f <脚本名>`** 按命令名匹配 Python 进程,不依赖 PID 文件(PID 文件容易和实际进程脱钩)
- 终止信号顺序:**SIGINT → SIGTERM → SIGKILL**,给数据保存留时间,超时再强杀

### 窗口管理

- 在每个 `gnome-terminal` 启动的 bash 命令行里注入 `TITLE_TAG=<窗口名>` 标记
- 关闭时用 `pkill -f "TITLE_TAG=xxx"` 精确命中那个 bash 进程
- 装了 `wmctrl` 的话有兜底:按窗口标题关闭

### 数据时间戳

各路数据流的发布频率不一致(相机 30Hz,关节/eef 60Hz),订阅器**不做时间同步**,每条消息按自带的 `header.stamp` 各自记录。**对齐留到训练时的后处理阶段**,这样:

- 不丢帧
- 训练时可以按需对齐(最近邻、插值、stack 多帧等)
- 调试更容易

## 常见问题

### Q1:按 [2] 提示「roscore 未运行」

先按 [1] 启动示教环境。

### Q2:按 [3] 后窗口没关闭

检查是否安装了 `wmctrl`:

```bash
sudo apt install wmctrl
```

没装也能跑,但窗口可能要等内部 bash 的 `read -n 1` 收到任意键才能关。装了就能自动关。

### Q3:数据没保存或保存的 pickle 很小

检查订阅器窗口的输出。SIGINT 触发后应该看到类似:

```
Received signal 2, saving data...
Saved to /home/.../raw_data/0007
  camera1 image: 245 frames
  ...
```

如果数字很小或为 0,说明订阅时根本没有数据 → 可能是相机没启动或机械臂没启动。

### Q4:连续按两次 [2]

脚本会检测 `data_record.py` 是否已在运行,有则提示「已有录制在运行,请先选 [3]」,不会启动重复进程。

### Q5:如何修改保存路径

编辑脚本顶部:

```bash
DATA_DIR="$HOME/workspace/raw_data"
```

改成你想要的目录。

## 文件结构

```
ARX_ARM/
├── data_collection/
│   ├── start_collection.sh         # 本启动脚本
│   ├── dual_arm_ctrl.py            # 机械臂示教 + 发布
│   ├── realsense_pub_node.py       # 相机发布
│   └── data_record.py              # 数据订阅 + pickle 保存
└── A5/
    └── bimanual/                   # ARX SDK
```
