#!/usr/bin/env python3

import csv
import time
from pathlib import Path

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Float64MultiArray
from controller_manager_msgs.srv import SwitchController


JOINT_NAMES = [
    "leg_front_r_1",
    "leg_front_r_2",
    "leg_front_r_3",
    "leg_front_l_1",
    "leg_front_l_2",
    "leg_front_l_3",
    "leg_back_r_1",
    "leg_back_r_2",
    "leg_back_r_3",
    "leg_back_l_1",
    "leg_back_l_2",
    "leg_back_l_3",
]

# From Pupper's neural_controller config default_joint_pos.
DEFAULT_POSE = [
    0.26, 0.0, -0.52,
   -0.26, 0.0,  0.52,
    0.26, 0.0, -0.52,
   -0.26, 0.0,  0.52,
]

# Conservative starting limits.
# If this blocks the pose you need, loosen slowly after testing.
MIN_ANGLE = -1.6
MAX_ANGLE = 1.6


class PS4PoseTuner(Node):
    def __init__(self):
        super().__init__("ps4_pose_tuner")

        self.declare_parameter("pose_name", "sit")
        self.declare_parameter("save_dir", str(Path.home() / "pupper_pose_tuner" / "poses"))
        self.declare_parameter("step", 0.03)
        self.declare_parameter("kp", 3.0)
        self.declare_parameter("kd", 0.20)
        self.declare_parameter("frames_to_save", 40)

        self.pose_name = self.get_parameter("pose_name").value
        self.save_dir = Path(self.get_parameter("save_dir").value)
        self.step = float(self.get_parameter("step").value)
        self.kp = float(self.get_parameter("kp").value)
        self.kd = float(self.get_parameter("kd").value)
        self.frames_to_save = int(self.get_parameter("frames_to_save").value)

        self.save_dir.mkdir(parents=True, exist_ok=True)

        self.selected_joint = 0
        self.target_positions = DEFAULT_POSE.copy()
        self.current_positions = None
        self.initialized_from_robot = False

        self.last_joy = None
        self.prev_buttons = []
        self.last_nudge_time = 0.0
        self.nudge_period = 0.05

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

        self.joy_sub = self.create_subscription(
            Joy,
            "/joy",
            self.joy_callback,
            10,
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
        )

        self.switch_client = self.create_client(
            SwitchController,
            "/controller_manager/switch_controller",
        )

        self.timer = self.create_timer(0.02, self.control_loop)

        self.switch_to_forward_mode()

        self.get_logger().info("PS4 pose tuner started.")
        self.print_controls()

    def print_controls(self):
        self.get_logger().info("")
        self.get_logger().info("Controls:")
        self.get_logger().info("  L1 / R1       : select previous/next joint")
        self.get_logger().info("  D-pad up/down : increase/decrease selected joint angle")
        self.get_logger().info("  X             : save pose CSV")
        self.get_logger().info("  Circle        : reset to default pose")
        self.get_logger().info("  Triangle      : print current pose")
        self.get_logger().info("")
        self.print_selected_joint()

    def switch_to_forward_mode(self):
        if not self.switch_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn("Controller switch service not available yet.")
            return

        req = SwitchController.Request()
        req.activate_controllers = [
            "forward_position_controller",
            "forward_kp_controller",
            "forward_kd_controller",
        ]
        req.deactivate_controllers = [
            "neural_controller",
            "neural_controller_three_legged",
        ]
        req.strictness = SwitchController.Request.BEST_EFFORT
        req.activate_asap = True
        req.timeout = rclpy.duration.Duration(seconds=3.0).to_msg()

        future = self.switch_client.call_async(req)
        future.add_done_callback(self.switch_done_callback)

    def switch_done_callback(self, future):
        try:
            result = future.result()
            if result.ok:
                self.get_logger().info("Switched to forward command controllers.")
            else:
                self.get_logger().warn("Controller switch returned not OK.")
        except Exception as e:
            self.get_logger().error(f"Controller switch failed: {e}")

    def joint_state_callback(self, msg: JointState):
        joint_map = dict(zip(msg.name, msg.position))

        if all(name in joint_map for name in JOINT_NAMES):
            self.current_positions = [joint_map[name] for name in JOINT_NAMES]

            # First time only: start from actual robot pose, not hardcoded pose.
            if not self.initialized_from_robot:
                self.target_positions = self.current_positions.copy()
                self.initialized_from_robot = True
                self.get_logger().info("Initialized target pose from /joint_states.")
                self.print_selected_joint()

    def button_edge(self, buttons, index):
        if index >= len(buttons):
            return False

        old = self.prev_buttons[index] if index < len(self.prev_buttons) else 0
        new = buttons[index]
        return old == 0 and new == 1

    def joy_callback(self, msg: Joy):
        self.last_joy = msg
        buttons = msg.buttons

        # PS4 common mapping:
        # X = 0, Circle = 1, Triangle = 3, L1 = 4, R1 = 5
        # If your mapping is different, check with: ros2 topic echo /joy

        if self.button_edge(buttons, 4):  # L1
            self.selected_joint = (self.selected_joint - 1) % len(JOINT_NAMES)
            self.print_selected_joint()

        if self.button_edge(buttons, 5):  # R1
            self.selected_joint = (self.selected_joint + 1) % len(JOINT_NAMES)
            self.print_selected_joint()

        if self.button_edge(buttons, 0):  # X
            self.save_pose_csv()

        if self.button_edge(buttons, 1):  # Circle
            self.target_positions = DEFAULT_POSE.copy()
            self.get_logger().warn("Reset target pose to DEFAULT_POSE.")

        if self.button_edge(buttons, 3):  # Triangle
            self.print_pose()

        self.prev_buttons = list(buttons)

    def control_loop(self):
        if not self.initialized_from_robot:
            return

        self.handle_dpad_nudge()
        self.publish_motor_commands()

    def handle_dpad_nudge(self):
        if self.last_joy is None:
            return

        axes = self.last_joy.axes

        # Common PS4 ROS mapping: D-pad vertical is axes[7].
        # Up is usually +1, down is usually -1.
        if len(axes) <= 7:
            return

        dpad_vertical = axes[7]
        now = time.time()

        if abs(dpad_vertical) < 0.5:
            return

        if now - self.last_nudge_time < self.nudge_period:
            return

        delta = self.step if dpad_vertical > 0 else -self.step

        old_angle = self.target_positions[self.selected_joint]
        new_angle = old_angle + delta
        new_angle = max(MIN_ANGLE, min(MAX_ANGLE, new_angle))

        self.target_positions[self.selected_joint] = new_angle
        self.last_nudge_time = now

        joint_name = JOINT_NAMES[self.selected_joint]
        self.get_logger().info(
            f"{self.selected_joint:02d} {joint_name}: {old_angle:.3f} -> {new_angle:.3f} rad"
        )

    def publish_motor_commands(self):
        pos_msg = Float64MultiArray()
        pos_msg.data = list(self.target_positions)
        self.position_pub.publish(pos_msg)

        kp_msg = Float64MultiArray()
        kp_msg.data = [self.kp] * len(JOINT_NAMES)
        self.kp_pub.publish(kp_msg)

        kd_msg = Float64MultiArray()
        kd_msg.data = [self.kd] * len(JOINT_NAMES)
        self.kd_pub.publish(kd_msg)

    def save_pose_csv(self):
        path = self.save_dir / f"{self.pose_name}.csv"

        with open(path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(JOINT_NAMES)

            # Save repeated frames so animation_controller can play it as a short hold.
            for _ in range(self.frames_to_save):
                writer.writerow([f"{x:.6f}" for x in self.target_positions])

        self.get_logger().info(f"Saved pose to: {path}")

    def print_selected_joint(self):
        joint_name = JOINT_NAMES[self.selected_joint]
        angle = self.target_positions[self.selected_joint]
        self.get_logger().info(
            f"Selected joint {self.selected_joint:02d}: {joint_name} = {angle:.3f} rad"
        )

    def print_pose(self):
        self.get_logger().info("Current target pose:")
        for name, angle in zip(JOINT_NAMES, self.target_positions):
            self.get_logger().info(f"  {name}: {angle:.6f}")


def main(args=None):
    rclpy.init(args=args)
    node = PS4PoseTuner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
