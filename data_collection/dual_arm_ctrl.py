#!/usr/bin/env python3

"""
Description: ARX A5 dual-arm teaching data collection + ROS topic publishing.
Drag both arms manually in gravity compensation mode, and publish joint states
and end-effector pose for each arm in real time.

Topics published:
    /arx_left/joint_states     (sensor_msgs/JointState)
    /arx_left/eef_pose         (geometry_msgs/PoseStamped)
    /arx_right/joint_states    (sensor_msgs/JointState)
    /arx_right/eef_pose        (geometry_msgs/PoseStamped)
"""


import os
import sys
import time
import rospy
import argparse
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from typing import Dict, Any, List
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from A5.bimanual import SingleArm


class ArmPublisher:
    """One ROS publisher pair (joint state + ee pose) for a single arm."""

    def __init__(self, arm: SingleArm, name: str, joint_names: List[str]):
        self.arm = arm
        self.name = name  # 'left' / 'right' — used as namespace and joint prefix

        # Prefix joint names with the arm name to avoid collisions in /joint_states viewers
        self.joint_names = [f'{name}_{j}' for j in joint_names]

        self.joint_pub = rospy.Publisher(
            f'/arx_{name}/joint_states', JointState, queue_size=20
        )
        self.eef_pub = rospy.Publisher(
            f'/arx_{name}/eef_pose', PoseStamped, queue_size=20
        )

        # Frame id namespaced per arm so TF stays unambiguous
        self.frame_id = f'{name}_base_link'

    def publish_once(self, stamp: rospy.Time):
        # Joint angles (radians)
        joint_positions = self.arm.get_joint_positions()

        # End-effector pose: [x, y, z, qw, qx, qy, qz]
        ee_pose = self.arm.get_ee_pose()

        # JointState
        joint_msg = JointState()
        joint_msg.header.stamp = stamp
        joint_msg.name = self.joint_names[:len(joint_positions)]
        joint_msg.position = list(joint_positions)
        joint_msg.velocity = [0.0] * len(joint_positions)
        joint_msg.effort = [0.0] * len(joint_positions)
        self.joint_pub.publish(joint_msg)

        # PoseStamped (ARX returns xyz + wxyz; assign Quaternion fields individually)
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.frame_id
        pose_msg.pose.position.x = ee_pose[0]
        pose_msg.pose.position.y = ee_pose[1]
        pose_msg.pose.position.z = ee_pose[2]
        pose_msg.pose.orientation.w = ee_pose[3]
        pose_msg.pose.orientation.x = ee_pose[4]
        pose_msg.pose.orientation.y = ee_pose[5]
        pose_msg.pose.orientation.z = ee_pose[6]
        self.eef_pub.publish(pose_msg)

        return joint_positions, ee_pose


class RecordDualArx:
    # Base joint names (must match a5.urdf). Will be prefixed per arm.
    BASE_JOINT_NAMES = [
        'joint1', 'joint2', 'joint3',
        'joint4', 'joint5', 'joint6',
        'joint7',
    ]

    def __init__(
        self,
        left_can_port: str,
        right_can_port: str,
        urdf_name: str,
        freq: int,
    ):
        self.freq = freq

        # Initialize both arms
        left_cfg: Dict[str, Any] = {
            "can_port": left_can_port,
            "urdf_name": urdf_name,
        }
        right_cfg: Dict[str, Any] = {
            "can_port": right_can_port,
            "urdf_name": urdf_name,
        }
        self.arm_left = SingleArm(left_cfg)
        self.arm_right = SingleArm(right_cfg)

        # Enter gravity compensation (teaching) mode for both arms
        self.arm_left.gravity_compensation()
        self.arm_right.gravity_compensation()

        # Initialize ROS node (single node publishes for both arms)
        rospy.init_node('arx_dual_recorder', anonymous=True)

        # Per-arm publisher wrappers
        self.pub_left = ArmPublisher(self.arm_left, 'left', self.BASE_JOINT_NAMES)
        self.pub_right = ArmPublisher(self.arm_right, 'right', self.BASE_JOINT_NAMES)

        time.sleep(0.5)

    def pub_data(self):
        print(f'Start publishing dual-arm trajectory data at {self.freq} Hz')
        print('Drag both arms manually for teaching. Press Ctrl+C to exit.')
        rate = rospy.Rate(self.freq)

        while not rospy.is_shutdown():
            try:
                # Use one shared timestamp so left/right messages are time-aligned
                stamp = rospy.Time.now()

                left_joints, left_pose = self.pub_left.publish_once(stamp)
                right_joints, right_pose = self.pub_right.publish_once(stamp)

                # Optional: print for debugging. Comment out for high frequency.
                print(f'[L] joints={left_joints}')
                print(f'[R] joints={right_joints}')

                rate.sleep()

            except Exception as e:
                print(f"Error while publishing data: {e}")
                time.sleep(0.1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--left_can',
        type=str,
        default='can1',
        help='CAN port for the LEFT arm, e.g. can0 / can1 / can3',
    )
    parser.add_argument(
        '--right_can',
        type=str,
        default='can3',
        help='CAN port for the RIGHT arm, e.g. can0 / can1 / can3',
    )
    parser.add_argument(
        '--urdf_name',
        type=str,
        default='a5.urdf',
        help='ARX URDF file name (shared by both arms)',
    )
    parser.add_argument(
        '--freq',
        type=int,
        default=60,
        help='Publishing frequency (Hz). Keep at 60 — ARX A5 firmware appears '
             'to need this rate to stay in gravity-comp mode. Camera/arm rate '
             'mismatch (cam 30Hz / arm 60Hz) is reconciled in LeRobot conversion '
             'by downsampling joint state to image timestamps.',
    )
    args = parser.parse_args()

    try:
        recorder = RecordDualArx(
            left_can_port=args.left_can,
            right_can_port=args.right_can,
            urdf_name=args.urdf_name,
            freq=args.freq,
        )
        recorder.pub_data()
    except rospy.ROSInterruptException:
        print("Interrupted by user")
    except Exception as e:
        print(f"Runtime error: {e}")