# 数据验证(回放)

把录制好的 `state.pkl` 重放到物理臂上,用来**验证采集质量** —— 比 Rerun 可视化更严格,因为它会暴露出可视化看不出来的问题:关节跳变、夹爪信号不连续、demo 起点和臂当前位姿差太远等等。

回放不属于数据采集,所以独立放在 `data_replay/`,避免和 `data_collection/` 的脚本互相干扰(尤其是 `dual_arm_ctrl.py` 会把臂设成重力补偿模式,和回放需要的位置控制模式冲突)。

## 前置条件

1. **臂上电**、急停按钮在手边。
2. **CAN 接口 UP**。最省事的方式:
   ```bash
   ./dual_arm_sys.sh   # 选 [1] 启动示教数采
   ```
   `[1]` 会拉起 `can1` 和 `can3` 的 watchdog。**起完之后不要选 [2]**,否则 `dual_arm_ctrl.py` 会把臂设成重力补偿模式跟回放抢控制权。
3. **不需要** roscore、不需要相机节点 —— 回放走 SDK 直连,不经 ROS。`[1]` 顺带起的 roscore/相机不影响,无视即可。
4. **conda 环境**:`robo_ctrl`(有 `arx_r5_python` 绑定)。
5. **工作目录**:仓库根目录(`A5/` 在 PYTHONPATH 里要找得到),脚本里已经 `sys.path.insert` 过了,从哪里跑都行。

## 用法

### 第一次跑某条 episode — 强烈建议这样做

```bash
conda activate robo_ctrl
cd ~/workspace/ARX_A5_Dual_ARM

# 1. 先 dry-run 看数据范围合不合理
python data_replay/replay_episode.py ~/workspace/raw_data/egg_to_bowl/0000 --dry-run

# 2. 单臂、半速、长 warmup 试一次
python data_replay/replay_episode.py ~/workspace/raw_data/egg_to_bowl/0000 \
    --no-left --speed 0.5 --warmup-seconds 8

# 3. 同样参数换另一臂
python data_replay/replay_episode.py ~/workspace/raw_data/egg_to_bowl/0000 \
    --no-right --speed 0.5 --warmup-seconds 8

# 4. 都没问题再上双臂、原速
python data_replay/replay_episode.py ~/workspace/raw_data/egg_to_bowl/0000
```

### 日常验证(已熟悉这条 episode)

```bash
python data_replay/replay_episode.py ~/workspace/raw_data/egg_to_bowl/0042
```

### 参数

| flag | 默认 | 说明 |
|---|---|---|
| `episode_dir` | — | 必填,episode 目录路径 |
| `--speed` | 1.0 | 回放速度倍率。0.5 = 半速,2.0 = 双倍速 |
| `--warmup-seconds` | 5.0 | 从当前位姿插值到首帧的时间 |
| `--start` | 0.0 | 从 episode 第几秒开始(裁掉前面) |
| `--end` | (到结束) | 到 episode 第几秒结束 |
| `--no-left` / `--no-right` | off | 跳过该侧臂 |
| `--left-can` / `--right-can` | can1 / can3 | CAN 接口名 |
| `--urdf-name` | a5.urdf | URDF 文件名 |
| `--dry-run` | off | 只载入数据并打印摘要,不连臂 |

## 安全注意事项

### 首帧突跳是最大的坑

录制时臂在重力补偿模式,操作员从任意位姿开始拖动。回放时臂在位控模式,如果直接发 `joints[0]`,臂会以最大速度从当前位姿冲过去 —— **30 cm/100 ms 的运动量在 A5 上完全可能,会撞坏夹爪/桌上的东西/对面的臂**。

`--warmup-seconds 5` 默认在 5 秒内线性插值过去,新 episode 第一次跑用 8-10 秒更稳。

### 第一次跑必做

- [ ] 两臂周围 50 cm 内清空(纸杯、电缆、咖啡杯)
- [ ] 手放急停按钮上,不离开,直到 warmup 段过完看着臂稳住
- [ ] `--no-left` 或 `--no-right` 单臂先跑,确认数据没问题
- [ ] `--speed 0.5` 半速跑,留出反应时间

### Ctrl+C 不是急停

Ctrl+C 让脚本停止发送命令,**臂会保持在最后一个位置**(固件的位置 holding)。这不等于"安全停下" —— 如果臂正在朝桌子方向走,松开 Ctrl+C 时它会停在半空,不会自动回原位。**急停按钮才是急停**。

### 失败 episode 不要拿来首测

`_failed/f*/` 里的 episode 通常本身就有问题(夹爪没抓住、demo 走偏撞了东西、采集中途相机掉线等)。这些 episode 回放出来要么轨迹本身就乱,要么夹爪信号异常,不适合用来验证回放系统是否正常。验证回放系统先用有效 episode。

## 数据格式参考

`state.pkl` 结构:

```python
{
    'left_arm':  {'joints': np.ndarray (N, 7), 'timestamps': [int ms, ...]},
    'right_arm': {'joints': np.ndarray (N, 7), 'timestamps': [int ms, ...]},
}
```

7 列 = 6 个臂关节(弧度)+ 1 个夹爪开度。回放时:

```python
arm.set_joint_positions(row[:6])  # 6 个关节
arm.set_gripper_pos(row[6])       # 夹爪(负值代表合拢方向)
```

参考 `A5/transmission/arm_bridge.py:84-90` —— 这是项目里已有的同样切分方式。

时间戳 60 Hz,episode 里左右臂用同一 `rospy.Time.now()` 打的 stamp,所以 `left.timestamps[i] ≈ right.timestamps[i]`,行数也对齐。回放脚本拿 `left` 作为时间轴驱动(只有右臂时拿 `right`)。

## 故障排查

### 启动报 `No module named arx_r5_python`

没激活 `robo_ctrl` 环境。`conda activate robo_ctrl`。

### 启动报 `failed to connect to can1` 或臂"不动"

CAN 没起来。检查:
```bash
ip link show can1     # 看 UP/DOWN
pgrep -af arx_can1    # 看 watchdog 在不在
```
没起来就 `./dual_arm_sys.sh` 选 [1]。如果 [1] 也起不来看主 README 的 Q6 (sudo NOPASSWD)。

### 臂在 warmup 段就抖/卡

可能你之前跑过 `[2]` 让 `dual_arm_ctrl.py` 把臂切到重力补偿模式了。这个进程没退干净的话会和回放抢 CAN。检查:
```bash
pgrep -af dual_arm_ctrl
```
有就 kill 掉:
```bash
pkill -f dual_arm_ctrl.py
```

### Ctrl+C 之后臂还在动

理论上 SIGINT 处理器会立刻停发命令。如果还在动通常是固件还在执行最后一个排队的命令 —— 1 秒内会停。**如果超过 1 秒没停**,马上按急停。事后查 SDK 版本和这条 episode 的最后几帧。

### `joints` 列数不是 7

数据格式有问题。看一下:
```python
import pickle, numpy as np
with open('state.pkl','rb') as f: s = pickle.load(f)
print(np.asarray(s['left_arm']['joints']).shape)
```
如果是 `(N, 6)` 是旧格式没记夹爪,回放只能控臂关节不能控夹爪 —— 这个版本的脚本会拒绝跑,要手动加 padding 或改脚本去掉夹爪那行。
