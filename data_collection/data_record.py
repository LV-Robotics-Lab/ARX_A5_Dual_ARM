#!/usr/bin/env python3

"""
Description: Subscribe to all data topics (cameras, dual-arm joints and EE pose)
and save them to pickle files when the node is killed.

Topics subscribed:
    /camera_{1,2,3}_image      (sensor_msgs/Image, bgr8)
    /camera_{1,2,3}_depth      (sensor_msgs/Image, 32FC1)
    /arx_left/joint_states     (sensor_msgs/JointState)
    /arx_right/joint_states    (sensor_msgs/JointState)
    /arx_left/eef_pose         (geometry_msgs/PoseStamped)
    /arx_right/eef_pose        (geometry_msgs/PoseStamped)
"""

import argparse
import os
import pickle
import signal
import time

import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, JointState


def stamp_to_ms(stamp) -> int:
    """ROS Time -> milliseconds (int)."""
    return int(1000 * (stamp.secs + stamp.nsecs * 1e-9))


class Subscribers:
    def __init__(self, save_path: str, num_cameras: int = 3):
        rospy.init_node('all_topics_subscriber', anonymous=True)
        self.save_path = save_path
        self.num_cameras = num_cameras
        self.bridge = CvBridge()
        self._saved = False  # guard against double-save

        # Dynamic camera dicts based on num_cameras
        self.cam_data_dict = {
            f'camera{i}': {'image': [], 'timestamps': []}
            for i in range(1, num_cameras + 1)
        }
        self.dpt_data_dict = {
            f'camera{i}': {'depth': [], 'timestamps': []}
            for i in range(1, num_cameras + 1)
        }
        self.state_data_dict = {
            'left_arm':  {'joints': [], 'timestamps': []},
            'right_arm': {'joints': [], 'timestamps': []},
        }
        self.eef_data_dict = {
            'left_arm':  {'eef_pose': [], 'timestamps': []},
            'right_arm': {'eef_pose': [], 'timestamps': []},
        }

        # Register signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

    # ---------------- Camera callbacks (factory) ----------------

    def _make_image_cb(self, cam_key: str):
        def _cb(msg: Image):
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.cam_data_dict[cam_key]['image'].append(img)
            self.cam_data_dict[cam_key]['timestamps'].append(stamp_to_ms(msg.header.stamp))
        return _cb

    def _make_depth_cb(self, cam_key: str):
        def _cb(msg: Image):
            dpt = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            self.dpt_data_dict[cam_key]['depth'].append(dpt)
            self.dpt_data_dict[cam_key]['timestamps'].append(stamp_to_ms(msg.header.stamp))
        return _cb

    # ---------------- Joint callbacks ----------------

    def left_joints_callback(self, msg: JointState):
        joints = np.array(msg.position, dtype=np.float64)
        self.state_data_dict['left_arm']['joints'].append(joints)
        self.state_data_dict['left_arm']['timestamps'].append(stamp_to_ms(msg.header.stamp))

    def right_joints_callback(self, msg: JointState):
        joints = np.array(msg.position, dtype=np.float64)
        self.state_data_dict['right_arm']['joints'].append(joints)
        self.state_data_dict['right_arm']['timestamps'].append(stamp_to_ms(msg.header.stamp))

    # ---------------- EE pose callbacks ----------------

    @staticmethod
    def _pose_to_array(msg: PoseStamped) -> np.ndarray:
        # Order: [x, y, z, qw, qx, qy, qz] (matches publisher convention)
        return np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            msg.pose.orientation.w,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
        ], dtype=np.float64)

    def left_eef_pose_callback(self, msg: PoseStamped):
        self.eef_data_dict['left_arm']['eef_pose'].append(self._pose_to_array(msg))
        self.eef_data_dict['left_arm']['timestamps'].append(stamp_to_ms(msg.header.stamp))

    def right_eef_pose_callback(self, msg: PoseStamped):
        self.eef_data_dict['right_arm']['eef_pose'].append(self._pose_to_array(msg))
        self.eef_data_dict['right_arm']['timestamps'].append(stamp_to_ms(msg.header.stamp))

    # ---------------- Subscriptions ----------------

    def run(self):
        # Cameras (image + depth) for each camera id
        self._cam_subs = []
        for i in range(1, self.num_cameras + 1):
            cam_key = f'camera{i}'
            self._cam_subs.append(rospy.Subscriber(
                f'/camera_{i}_image', Image,
                self._make_image_cb(cam_key), queue_size=10,
            ))
            self._cam_subs.append(rospy.Subscriber(
                f'/camera_{i}_depth', Image,
                self._make_depth_cb(cam_key), queue_size=10,
            ))

        # Dual-arm joint states
        self.sub_left_joints = rospy.Subscriber(
            '/arx_left/joint_states', JointState,
            self.left_joints_callback, queue_size=20,
        )
        self.sub_right_joints = rospy.Subscriber(
            '/arx_right/joint_states', JointState,
            self.right_joints_callback, queue_size=20,
        )

        # Dual-arm end-effector poses
        self.sub_left_eef = rospy.Subscriber(
            '/arx_left/eef_pose', PoseStamped,
            self.left_eef_pose_callback, queue_size=20,
        )
        self.sub_right_eef = rospy.Subscriber(
            '/arx_right/eef_pose', PoseStamped,
            self.right_eef_pose_callback, queue_size=20,
        )

        rospy.loginfo("Data Collection Started")

    # ---------------- Shutdown / save ----------------

    def signal_handler(self, signum, frame):
        rospy.loginfo("Received signal %s, saving data..." % signum)
        self.save_data()
        rospy.signal_shutdown('Killed by user')

    def save_data(self):
        if self._saved:
            return
        self._saved = True

        if self.save_path is None:
            rospy.logwarn("No save_path set, data will NOT be saved!")
            return

        os.makedirs(self.save_path, exist_ok=True)

        save_image_path = os.path.join(self.save_path, 'image.pkl')
        save_depth_path = os.path.join(self.save_path, 'depth.pkl')
        save_state_path = os.path.join(self.save_path, 'state.pkl')
        save_eef_path   = os.path.join(self.save_path, 'eef_pose.pkl')

        with open(save_image_path, 'wb') as f:
            pickle.dump(self.cam_data_dict, f)
        with open(save_depth_path, 'wb') as f:
            pickle.dump(self.dpt_data_dict, f)
        with open(save_state_path, 'wb') as f:
            pickle.dump(self.state_data_dict, f)
        with open(save_eef_path, 'wb') as f:
            pickle.dump(self.eef_data_dict, f)

        # Print a quick summary so you know what was captured
        rospy.loginfo("Saved to %s", self.save_path)
        for k, v in self.cam_data_dict.items():
            rospy.loginfo("  %s image: %d frames", k, len(v['image']))
        for k, v in self.dpt_data_dict.items():
            rospy.loginfo("  %s depth: %d frames", k, len(v['depth']))
        for k, v in self.state_data_dict.items():
            rospy.loginfo("  %s joints: %d samples", k, len(v['joints']))
        for k, v in self.eef_data_dict.items():
            rospy.loginfo("  %s eef:    %d samples", k, len(v['eef_pose']))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--root_dir", type=str, default='/home/robotics/raw_data/oo')
    parser.add_argument("--traj_number", type=int, default=255)
    parser.add_argument("--num_cameras", type=int, default=3, help="Number of cameras to subscribe")
    args = parser.parse_args()

    save_path = os.path.join(args.root_dir, str(args.traj_number).zfill(4))
    os.makedirs(save_path, exist_ok=True)

    try:
        subscribers = Subscribers(save_path=save_path, num_cameras=args.num_cameras)
        subscribers.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass