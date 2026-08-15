#!/usr/bin/env python3
"""ARX A5 dual-arm teaching and ROS state publishing.

This process changes both physical arms to gravity-compensation mode. It is
therefore fail-closed: all four execution confirmations are required before the
wrapper imports the vendor SDK or connects to CAN.
"""

import argparse
import time
from dataclasses import replace
from typing import List

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

from arx_wrapper import ArmEndpoint, ArxArm, ArxConfig, ArxDualArm, MotionGate


class ArmPublisher:
    """One ROS publisher pair (joint state + end-effector pose) for one arm."""

    def __init__(self, arm: ArxArm, name: str, joint_names: List[str]):
        self.arm = arm
        self.name = name
        self.joint_names = [f"{name}_{joint}" for joint in joint_names]
        self.joint_pub = rospy.Publisher(
            f"/arx_{name}/joint_states", JointState, queue_size=20
        )
        self.eef_pub = rospy.Publisher(
            f"/arx_{name}/eef_pose", PoseStamped, queue_size=20
        )
        self.frame_id = f"{name}_base_link"

    def publish_once(self, stamp: rospy.Time):
        joint_positions = self.arm.get_joint_positions()
        ee_pose = self.arm.get_ee_pose()

        joint_msg = JointState()
        joint_msg.header.stamp = stamp
        joint_msg.name = self.joint_names[: len(joint_positions)]
        joint_msg.position = list(joint_positions)
        joint_msg.velocity = [0.0] * len(joint_positions)
        joint_msg.effort = [0.0] * len(joint_positions)
        self.joint_pub.publish(joint_msg)

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
    BASE_JOINT_NAMES = [
        "joint1",
        "joint2",
        "joint3",
        "joint4",
        "joint5",
        "joint6",
        "joint7",
    ]

    def __init__(self, pair: ArxDualArm, *, freq: float, verbose: bool = False):
        self.freq = freq
        self.verbose = verbose
        rospy.init_node("arx_dual_recorder", anonymous=True)
        self.pub_left = ArmPublisher(pair.left, "left", self.BASE_JOINT_NAMES)
        self.pub_right = ArmPublisher(pair.right, "right", self.BASE_JOINT_NAMES)
        time.sleep(0.5)

    def pub_data(self) -> None:
        print(f"Start publishing dual-arm trajectory data at {self.freq:g} Hz")
        print("Both arms are in teaching mode. Press Ctrl+C to exit.")
        rate = rospy.Rate(self.freq)

        while not rospy.is_shutdown():
            try:
                stamp = rospy.Time.now()
                left_joints, _ = self.pub_left.publish_once(stamp)
                right_joints, _ = self.pub_right.publish_once(stamp)
                if self.verbose:
                    print(f"[L] joints={left_joints}")
                    print(f"[R] joints={right_joints}")
                rate.sleep()
            except Exception as error:
                print(f"Error while publishing data: {error}")
                time.sleep(0.1)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--left-can", "--left_can", dest="left_can", default=None)
    parser.add_argument("--right-can", "--right_can", dest="right_can", default=None)
    parser.add_argument("--urdf-name", "--urdf_name", dest="urdf_name", default=None)
    parser.add_argument("--freq", type=float, default=None)
    parser.add_argument("--verbose", action="store_true", help="print every joint sample")
    parser.add_argument("--execute", action="store_true", help="allow real hardware state changes")
    parser.add_argument("--clearance-confirmed", action="store_true")
    parser.add_argument("--estop-ready", action="store_true")
    parser.add_argument("--exclusive-control-confirmed", action="store_true")
    return parser


def main() -> int:
    args = _parser().parse_args()
    base = ArxConfig.from_env()
    urdf_name = args.urdf_name or base.arms[0].urdf_name
    config = replace(
        base,
        arms=(
            ArmEndpoint("left", args.left_can or base.arm("left").can_interface, urdf_name),
            ArmEndpoint("right", args.right_can or base.arm("right").can_interface, urdf_name),
        ),
    )
    gate = MotionGate(
        execute=args.execute,
        clearance_confirmed=args.clearance_confirmed,
        estop_ready=args.estop_ready,
        control_source_exclusive=args.exclusive_control_confirmed,
    )
    gate.require_motion()

    with ArxDualArm(config=config, gate=gate) as pair:
        pair.enter_teaching_mode()
        recorder = RecordDualArx(
            pair,
            freq=args.freq or config.arm_publish_hz,
            verbose=args.verbose,
        )
        recorder.pub_data()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except rospy.ROSInterruptException:
        print("Interrupted by ROS shutdown")
