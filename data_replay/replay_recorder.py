"""
Recorder used during replay+capture runs.

Subscribes to the same camera ROS topics as data_collection/data_record.py
(/<cam_key>_image, /<cam_key>_depth), but instead of subscribing to the joint
state topics it receives joint + EEF state via direct method calls from
replay_episode.py (which already owns the SingleArm SDK instances).

Output layout is byte-identical to a teaching episode produced by
data_collection/data_record.py — same files, same pickle schemas, same h5
dataset names — so any downstream loader / visualizer / verifier works on
replay-recorded episodes without modification.

Usage from replay_episode.py:
    rospy.init_node('replay_recorder', anonymous=True)
    rec = ReplayRecorder(save_path)
    rec.start()
    # ... replay loop ...
    rec.record_arm_state('right_arm', joints, eef, stamp_ms)
    # ...
    rec.finalize()
"""
import os
import pickle
from threading import Lock

import numpy as np
import cv2
import h5py
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


DEFAULT_CAM_KEYS = ('cam_top', 'cam_left_wrist', 'cam_right_wrist')


def stamp_to_ms(stamp) -> int:
    return int(stamp.secs * 1000 + stamp.nsecs // 1_000_000)


class ReplayRecorder:
    def __init__(self, save_path: str, cam_keys=DEFAULT_CAM_KEYS,
                 video_fps: float = 30.0, video_fourcc: str = 'mp4v'):
        self.save_path = save_path
        self.cam_keys = list(cam_keys)
        self.video_fps = video_fps
        self.video_fourcc = video_fourcc
        os.makedirs(self.save_path, exist_ok=True)

        self.bridge = CvBridge()
        self._finalized = False

        # RGB streaming writers + per-camera locks (mirrors data_record.py)
        self._rgb_writers = {k: None for k in self.cam_keys}
        self._rgb_timestamps = {k: [] for k in self.cam_keys}
        self._rgb_counts = {k: 0 for k in self.cam_keys}
        self._rgb_locks = {k: Lock() for k in self.cam_keys}

        # Depth h5 streams (resizable, lzf-compressed)
        self._depth_files = {k: None for k in self.cam_keys}
        self._depth_dsets = {k: None for k in self.cam_keys}
        self._depth_ts_dsets = {k: None for k in self.cam_keys}
        self._depth_counts = {k: 0 for k in self.cam_keys}
        self._depth_locks = {k: Lock() for k in self.cam_keys}

        # Arm state — fed via record_arm_state() from replay
        self.state_data_dict = {
            'left_arm':  {'joints': [], 'timestamps': []},
            'right_arm': {'joints': [], 'timestamps': []},
        }
        self.eef_data_dict = {
            'left_arm':  {'eef_pose': [], 'timestamps': []},
            'right_arm': {'eef_pose': [], 'timestamps': []},
        }
        self.current_data_dict = {
            'left_arm':  {'currents': [], 'timestamps': []},
            'right_arm': {'currents': [], 'timestamps': []},
        }
        self._state_lock = Lock()

        self._all_subs = []

    # ------------- camera callbacks (lifted from data_record.py) -------------

    def _make_image_cb(self, cam_key: str):
        def _cb(msg: Image):
            with self._rgb_locks[cam_key]:
                if self._finalized:
                    return
                img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                writer = self._rgb_writers[cam_key]
                if writer is None:
                    h, w = img.shape[:2]
                    path = os.path.join(self.save_path, f'{cam_key}_rgb.mp4')
                    fourcc = cv2.VideoWriter_fourcc(*self.video_fourcc)
                    writer = cv2.VideoWriter(path, fourcc, self.video_fps, (w, h))
                    if not writer.isOpened():
                        rospy.logerr("Failed to open VideoWriter for %s at %s", cam_key, path)
                        return
                    self._rgb_writers[cam_key] = writer
                writer.write(img)
                self._rgb_timestamps[cam_key].append(stamp_to_ms(msg.header.stamp))
                self._rgb_counts[cam_key] += 1
        return _cb

    def _make_depth_cb(self, cam_key: str):
        def _cb(msg: Image):
            with self._depth_locks[cam_key]:
                if self._finalized:
                    return
                dpt = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
                dpt_u16 = dpt.astype(np.uint16)
                h5f = self._depth_files[cam_key]
                if h5f is None:
                    h, w = dpt_u16.shape[:2]
                    path = os.path.join(self.save_path, f'{cam_key}_depth.h5')
                    h5f = h5py.File(path, 'w')
                    ds = h5f.create_dataset(
                        'depth_mm', shape=(0, h, w), maxshape=(None, h, w),
                        chunks=(1, h, w), dtype=np.uint16, compression='lzf',
                    )
                    ts_ds = h5f.create_dataset(
                        'timestamps_ms', shape=(0,), maxshape=(None,),
                        chunks=(256,), dtype=np.int64,
                    )
                    ds.attrs['unit'] = 'mm'
                    ds.attrs['camera'] = cam_key
                    self._depth_files[cam_key] = h5f
                    self._depth_dsets[cam_key] = ds
                    self._depth_ts_dsets[cam_key] = ts_ds
                ds = self._depth_dsets[cam_key]
                ts_ds = self._depth_ts_dsets[cam_key]
                n = self._depth_counts[cam_key]
                ds.resize(n + 1, axis=0)
                ds[n] = dpt_u16
                ts_ds.resize(n + 1, axis=0)
                ts_ds[n] = stamp_to_ms(msg.header.stamp)
                self._depth_counts[cam_key] = n + 1
        return _cb

    # ------------- arm state ingest (called from replay loop) -------------

    def record_arm_state(self, side: str, joints, eef_pose, stamp_ms: int,
                         currents=None):
        """Replay calls this each frame with actual SDK readings."""
        if self._finalized:
            return
        with self._state_lock:
            self.state_data_dict[side]['joints'].append(
                np.array(joints, dtype=np.float64))
            self.state_data_dict[side]['timestamps'].append(int(stamp_ms))
            self.eef_data_dict[side]['eef_pose'].append(
                np.array(eef_pose, dtype=np.float64))
            self.eef_data_dict[side]['timestamps'].append(int(stamp_ms))
            if currents is not None:
                self.current_data_dict[side]['currents'].append(
                    np.array(currents, dtype=np.float64))
                self.current_data_dict[side]['timestamps'].append(int(stamp_ms))

    # ------------- lifecycle -------------

    def start(self):
        for cam_key in self.cam_keys:
            self._all_subs.append(rospy.Subscriber(
                f'/{cam_key}_image', Image,
                self._make_image_cb(cam_key), queue_size=10,
            ))
            self._all_subs.append(rospy.Subscriber(
                f'/{cam_key}_depth', Image,
                self._make_depth_cb(cam_key), queue_size=10,
            ))
        rospy.loginfo("ReplayRecorder streaming to %s", self.save_path)

    def finalize(self):
        if self._finalized:
            return
        self._finalized = True
        for sub in self._all_subs:
            try:
                sub.unregister()
            except Exception:
                pass

        for cam_key, writer in list(self._rgb_writers.items()):
            with self._rgb_locks[cam_key]:
                if writer is not None:
                    writer.release()
                    self._rgb_writers[cam_key] = None

        for cam_key, h5f in list(self._depth_files.items()):
            with self._depth_locks[cam_key]:
                if h5f is not None:
                    h5f.flush()
                    h5f.close()
                    self._depth_files[cam_key] = None

        with open(os.path.join(self.save_path, 'image_timestamps.pkl'), 'wb') as f:
            pickle.dump(self._rgb_timestamps, f)
        with open(os.path.join(self.save_path, 'state.pkl'), 'wb') as f:
            pickle.dump(self.state_data_dict, f)
        with open(os.path.join(self.save_path, 'eef_pose.pkl'), 'wb') as f:
            pickle.dump(self.eef_data_dict, f)
        # Only write currents.pkl if at least one side has data
        if any(len(v['currents']) > 0 for v in self.current_data_dict.values()):
            with open(os.path.join(self.save_path, 'currents.pkl'), 'wb') as f:
                pickle.dump(self.current_data_dict, f)

        print(f'Recorded → {self.save_path}')
        for k in sorted(self._rgb_counts):
            print(f'  {k} RGB:   {self._rgb_counts[k]} frames')
        for k in sorted(self._depth_counts):
            print(f'  {k} depth: {self._depth_counts[k]} frames')
        for k, v in self.state_data_dict.items():
            print(f'  {k} joints: {len(v["joints"])} samples')
        for k, v in self.eef_data_dict.items():
            print(f'  {k} eef:    {len(v["eef_pose"])} samples')
        for k, v in self.current_data_dict.items():
            if len(v['currents']) > 0:
                print(f'  {k} currents: {len(v["currents"])} samples')
