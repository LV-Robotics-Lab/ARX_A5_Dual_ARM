#!/usr/bin/env python3

"""
Subscribe to all data topics and STREAM them to disk so memory stays bounded
no matter how long the recording is.

Layout written under <save_path>/:
    cameraN_rgb.mp4         BGR uint8, one mp4 per camera (cv2.VideoWriter)
    cameraN_depth.h5        chunked uint16 (mm) + timestamps_ms (int64)
    image_timestamps.pkl    {cameraN: [ms, ...]}  — RGB frame stamps
    state.pkl               joints + timestamps  (small, in-memory)
    eef_pose.pkl            eef pose + timestamps (small, in-memory)

Why streaming: the old version buffered every frame in RAM and dumped one big
pickle at SIGINT. At ~3 cams × (RGB + Depth) @ 30 Hz this grows ~200 MB/s, so a
multi-minute recording either OOM-killed (no save) or hit the shell's kill
timeout mid-pickle.dump (truncated file).
"""

import argparse
import os
import pickle
import signal
import threading

import cv2
import h5py
import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, JointState


def stamp_to_ms(stamp) -> int:
    """ROS Time -> milliseconds (int)."""
    return int(1000 * (stamp.secs + stamp.nsecs * 1e-9))


class Subscribers:
    def __init__(self, save_path: str, num_cameras: int = 3,
                 video_fps: float = 30.0, video_fourcc: str = 'mp4v'):
        rospy.init_node('all_topics_subscriber', anonymous=True)
        self.save_path = save_path
        os.makedirs(self.save_path, exist_ok=True)
        self.num_cameras = num_cameras
        self.video_fps = video_fps
        self.video_fourcc = video_fourcc
        self.bridge = CvBridge()
        self._finalized = False

        cam_keys = [f'camera{i}' for i in range(1, num_cameras + 1)]

        # Per-camera RGB streaming state. Writer is lazy-init'd on first frame
        # because we need height/width from the message.
        self._rgb_writers     = {k: None for k in cam_keys}
        self._rgb_timestamps  = {k: [] for k in cam_keys}
        self._rgb_counts      = {k: 0 for k in cam_keys}
        self._rgb_locks       = {k: threading.Lock() for k in cam_keys}

        # Per-camera depth h5 state. Same lazy-init pattern.
        self._depth_files     = {k: None for k in cam_keys}
        self._depth_dsets     = {k: None for k in cam_keys}
        self._depth_ts_dsets  = {k: None for k in cam_keys}
        self._depth_counts    = {k: 0 for k in cam_keys}
        self._depth_locks     = {k: threading.Lock() for k in cam_keys}

        # State / eef stay in memory — at ~60 Hz × 7 floats per arm this is
        # under a few MB even for hour-long recordings.
        self.state_data_dict = {
            'left_arm':  {'joints': [], 'timestamps': []},
            'right_arm': {'joints': [], 'timestamps': []},
        }
        self.eef_data_dict = {
            'left_arm':  {'eef_pose': [], 'timestamps': []},
            'right_arm': {'eef_pose': [], 'timestamps': []},
        }

        # Subscribers we'll create in run(); kept so finalize() can unregister.
        self._all_subs: list = []

        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    # ---------------- Camera callbacks ----------------

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
                # realsense_pub_node casts z16 (uint16 mm) to float32 without
                # rescaling, so values are already integer millimetres.
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

    # ---------------- Joint / EE callbacks (small, in-memory) ----------------

    def _left_joints_cb(self, msg: JointState):
        if self._finalized:
            return
        self.state_data_dict['left_arm']['joints'].append(
            np.array(msg.position, dtype=np.float64))
        self.state_data_dict['left_arm']['timestamps'].append(
            stamp_to_ms(msg.header.stamp))

    def _right_joints_cb(self, msg: JointState):
        if self._finalized:
            return
        self.state_data_dict['right_arm']['joints'].append(
            np.array(msg.position, dtype=np.float64))
        self.state_data_dict['right_arm']['timestamps'].append(
            stamp_to_ms(msg.header.stamp))

    @staticmethod
    def _pose_to_array(msg: PoseStamped) -> np.ndarray:
        return np.array([
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
            msg.pose.orientation.w, msg.pose.orientation.x,
            msg.pose.orientation.y, msg.pose.orientation.z,
        ], dtype=np.float64)

    def _left_eef_cb(self, msg: PoseStamped):
        if self._finalized:
            return
        self.eef_data_dict['left_arm']['eef_pose'].append(self._pose_to_array(msg))
        self.eef_data_dict['left_arm']['timestamps'].append(stamp_to_ms(msg.header.stamp))

    def _right_eef_cb(self, msg: PoseStamped):
        if self._finalized:
            return
        self.eef_data_dict['right_arm']['eef_pose'].append(self._pose_to_array(msg))
        self.eef_data_dict['right_arm']['timestamps'].append(stamp_to_ms(msg.header.stamp))

    # ---------------- Subscriptions ----------------

    def run(self):
        for i in range(1, self.num_cameras + 1):
            cam_key = f'camera{i}'
            self._all_subs.append(rospy.Subscriber(
                f'/camera_{i}_image', Image,
                self._make_image_cb(cam_key), queue_size=10,
            ))
            self._all_subs.append(rospy.Subscriber(
                f'/camera_{i}_depth', Image,
                self._make_depth_cb(cam_key), queue_size=10,
            ))

        self._all_subs.append(rospy.Subscriber(
            '/arx_left/joint_states', JointState, self._left_joints_cb, queue_size=20))
        self._all_subs.append(rospy.Subscriber(
            '/arx_right/joint_states', JointState, self._right_joints_cb, queue_size=20))
        self._all_subs.append(rospy.Subscriber(
            '/arx_left/eef_pose', PoseStamped, self._left_eef_cb, queue_size=20))
        self._all_subs.append(rospy.Subscriber(
            '/arx_right/eef_pose', PoseStamped, self._right_eef_cb, queue_size=20))

        rospy.loginfo("Streaming data to %s", self.save_path)

    # ---------------- Shutdown / finalize ----------------

    def _signal_handler(self, signum, frame):
        rospy.loginfo("Received signal %s, finalizing...", signum)
        self.finalize()
        rospy.signal_shutdown('Killed by user')

    def finalize(self):
        if self._finalized:
            return
        # Setting the flag first means any callback that grabs its lock after
        # this point will see _finalized=True and bail out.
        self._finalized = True

        # Unregister subscribers so no more callbacks fire.
        for sub in self._all_subs:
            try:
                sub.unregister()
            except Exception:
                pass

        # Drain & close per-camera writers. Acquiring each lock waits for any
        # in-flight callback that already passed the _finalized check.
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

        # Small metadata pickles.
        with open(os.path.join(self.save_path, 'image_timestamps.pkl'), 'wb') as f:
            pickle.dump(self._rgb_timestamps, f)
        with open(os.path.join(self.save_path, 'state.pkl'), 'wb') as f:
            pickle.dump(self.state_data_dict, f)
        with open(os.path.join(self.save_path, 'eef_pose.pkl'), 'wb') as f:
            pickle.dump(self.eef_data_dict, f)

        rospy.loginfo("Saved to %s", self.save_path)
        for k in sorted(self._rgb_counts):
            rospy.loginfo("  %s RGB:   %d frames", k, self._rgb_counts[k])
        for k in sorted(self._depth_counts):
            rospy.loginfo("  %s depth: %d frames", k, self._depth_counts[k])
        for k, v in self.state_data_dict.items():
            rospy.loginfo("  %s joints: %d samples", k, len(v['joints']))
        for k, v in self.eef_data_dict.items():
            rospy.loginfo("  %s eef:    %d samples", k, len(v['eef_pose']))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--root_dir", type=str, default='/home/robotics/raw_data/oo')
    parser.add_argument("--traj_number", type=int, default=255)
    parser.add_argument("--num_cameras", type=int, default=3, help="Number of cameras to subscribe")
    parser.add_argument("--video_fps", type=float, default=30.0, help="VideoWriter fps tag")
    parser.add_argument("--video_fourcc", type=str, default='mp4v',
                        help="VideoWriter fourcc (mp4v=lossy mp4, FFV1=lossless if available)")
    args = parser.parse_args()

    save_path = os.path.join(args.root_dir, str(args.traj_number).zfill(4))

    try:
        subscribers = Subscribers(
            save_path=save_path,
            num_cameras=args.num_cameras,
            video_fps=args.video_fps,
            video_fourcc=args.video_fourcc,
        )
        subscribers.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
