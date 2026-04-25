#!/usr/bin/env python3

import csv
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

POSE_DIR = Path.home() / "pupper_pose_tuner" / "poses"

JOINT_NAMES = [
    "leg_front_r_1", "leg_front_r_2", "leg_front_r_3",
    "leg_front_l_1", "leg_front_l_2", "leg_front_l_3",
    "leg_back_r_1", "leg_back_r_2", "leg_back_r_3",
    "leg_back_l_1", "leg_back_l_2", "leg_back_l_3",
]

class PosePlayer(Node):
    def __init__(self, pose_name):
        super().__init__("pose_player")
        self.pose_name = pose_name
        self.current_positions = None

        self.position_pub = self.create_publisher(
            Float64MultiArray,
            "/forward_position_controller/commands",
            10,
        )
        self.kp_pub = self.create_publisher(
            Float64MultiArray,
            "/forward_kp_controller/commands",
            10,
        )
        self.kd_pub = self.create_publisher(
            Float64MultiArray,
            "/forward_kd_controller/commands",
            10,
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
        )

    def joint_state_callback(self, msg):
        joint_map = dict(zip(msg.name, msg.position))
        if all(name in joint_map for name in JOINT_NAMES):
            self.current_positions = [joint_map[name] for name in JOINT_NAMES]

    def load_pose(self):
        path = POSE_DIR / f"{self.pose_name}.csv"
        if not path.exists():
            raise FileNotFoundError(f"Pose file not found: {path}")

        with open(path, "r") as f:
            rows = list(csv.reader(f))

        target = [float(x) for x in rows[1]]
        if len(target) != 12:
            raise ValueError(f"Expected 12 joint values, got {len(target)}")

        return target

    def publish(self, positions, kp=4.0, kd=0.25):
        pos_msg = Float64MultiArray()
        pos_msg.data = positions
        self.position_pub.publish(pos_msg)

        kp_msg = Float64MultiArray()
        kp_msg.data = [kp] * 12
        self.kp_pub.publish(kp_msg)

        kd_msg = Float64MultiArray()
        kd_msg.data = [kd] * 12
        self.kd_pub.publish(kd_msg)

    def wait_for_joint_state(self):
        self.get_logger().info("Waiting for /joint_states...")
        start = time.time()

        while rclpy.ok() and self.current_positions is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > 5:
                raise TimeoutError("No /joint_states after 5 seconds")

        self.get_logger().info("Got /joint_states.")

    def move_to_pose(self, duration=2.0, steps=100):
        target = self.load_pose()
        self.wait_for_joint_state()

        start_pose = self.current_positions.copy()
        self.get_logger().info(f"Moving to {self.pose_name}")

        for i in range(steps + 1):
            alpha = i / steps
            cmd = [
                start_pose[j] * (1 - alpha) + target[j] * alpha
                for j in range(12)
            ]
            self.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.001)
            time.sleep(duration / steps)

        self.get_logger().info(f"Holding {self.pose_name}")

        for _ in range(100):
            self.publish(target)
            time.sleep(0.02)

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 play_pose.py sit|stand|lay")
        return

    rclpy.init()
    node = PosePlayer(sys.argv[1])

    try:
        node.move_to_pose()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
