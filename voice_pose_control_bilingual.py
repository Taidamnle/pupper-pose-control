#!/usr/bin/env python3

"""
Bilingual Voice Pose Control for Pupper v3

English commands:
    "sit down"   -> sit_v2.csv
    "stand up"  -> final_stand_v2.csv
    "lay down"  -> lay_v2.csv

Chinese commands:
    "坐下"        -> sit_v2.csv
    "站起来" / "起立" / "站立" -> final_stand_v2.csv
    "趴下" / "躺下" / "卧倒"   -> lay_v2.csv

Requirements:
    pip3 install vosk sounddevice

Model folders expected:
    ~/pupper_robot_testing/model_en
    ~/pupper_robot_testing/model_cn

Important:
    Do not run ps4_pose_tuner.py, play_pose.py, or play_motion_csv.py at the same time.
"""

import csv
import json
import queue
import re
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

try:
    import sounddevice as sd
    from vosk import Model, KaldiRecognizer
except ImportError:
    print("Missing dependency. Install with:")
    print("  pip3 install vosk sounddevice")
    sys.exit(1)


POSE_DIR = Path.home() / "pupper_pose_tuner" / "poses"

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

# Final pose mapping.
POSE_SIT = "sit_v2"
POSE_STAND = "final_stand_v2"
POSE_LAY = "lay_v2"

ENGLISH_PHRASES = {
    "sit down": POSE_SIT,
    "sit": POSE_SIT,
    "set down": POSE_SIT,
    "set": POSE_SIT,

    "stand up": POSE_STAND,
    "stand": POSE_STAND,
    "up": POSE_STAND,

    "lay down": POSE_LAY,
    "lie down": POSE_LAY,
    "lay": POSE_LAY,
    "lie": POSE_LAY,
    "down": POSE_LAY,
}

CHINESE_PHRASES = {
    "坐下": POSE_SIT,
    "坐": POSE_SIT,

    "站起来": POSE_STAND,
    "站起": POSE_STAND,
    "起立": POSE_STAND,
    "站立": POSE_STAND,
    "站": POSE_STAND,

    "趴下": POSE_LAY,
    "躺下": POSE_LAY,
    "卧倒": POSE_LAY,
    "趴": POSE_LAY,
    "躺": POSE_LAY,
}


class VoicePoseController(Node):
    def __init__(self):
        super().__init__("voice_pose_controller_bilingual")

        self.current_positions = None
        self.current_pose_name = "unknown"
        self.is_moving = False

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

    def wait_for_joint_state(self):
        self.get_logger().info("Waiting for /joint_states...")
        start = time.time()

        while rclpy.ok() and self.current_positions is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > 5:
                raise TimeoutError("No /joint_states received after 5 seconds")

        self.get_logger().info("Got /joint_states.")

    def load_pose(self, pose_name):
        path = POSE_DIR / f"{pose_name}.csv"

        if not path.exists():
            raise FileNotFoundError(f"Pose file not found: {path}")

        with open(path, "r") as f:
            rows = list(csv.reader(f))

        if len(rows) < 2:
            raise ValueError(f"Pose file has no data row: {path}")

        target = [float(x) for x in rows[1]]

        if len(target) != len(JOINT_NAMES):
            raise ValueError(f"Expected 12 joint values, got {len(target)}")

        return target

    def publish(self, positions, kp=4.0, kd=0.25):
        pos_msg = Float64MultiArray()
        pos_msg.data = list(positions)
        self.position_pub.publish(pos_msg)

        kp_msg = Float64MultiArray()
        kp_msg.data = [kp] * len(JOINT_NAMES)
        self.kp_pub.publish(kp_msg)

        kd_msg = Float64MultiArray()
        kd_msg.data = [kd] * len(JOINT_NAMES)
        self.kd_pub.publish(kd_msg)

    def move_to_pose(self, pose_name, duration=2.0, steps=100, hold_seconds=1.0):
        if self.is_moving:
            self.get_logger().warn("Already moving. Ignoring command.")
            return

        self.is_moving = True

        try:
            target = self.load_pose(pose_name)

            if self.current_positions is None:
                self.wait_for_joint_state()

            start_pose = self.current_positions.copy()

            self.get_logger().info(f"Voice command moving to: {pose_name}")

            for i in range(steps + 1):
                alpha = i / steps
                cmd = [
                    start_pose[j] * (1 - alpha) + target[j] * alpha
                    for j in range(len(JOINT_NAMES))
                ]

                self.publish(cmd)
                rclpy.spin_once(self, timeout_sec=0.001)
                time.sleep(duration / steps)

            hold_steps = int(hold_seconds / 0.02)
            for _ in range(hold_steps):
                self.publish(target)
                rclpy.spin_once(self, timeout_sec=0.001)
                time.sleep(0.02)

            self.current_pose_name = pose_name
            self.get_logger().info(f"Holding: {pose_name}")

        finally:
            self.is_moving = False


def find_model_dir(folder_name):
    candidates = [
        Path.home() / "pupper_robot_testing" / folder_name,
        Path.cwd() / folder_name,
    ]

    for candidate in candidates:
        if candidate.exists() and candidate.is_dir():
            return candidate

    raise FileNotFoundError(
        f"Could not find {folder_name}. Expected it in ~/pupper_robot_testing/{folder_name}"
    )


def normalize_english(text):
    text = text.lower().strip()
    text = re.sub(r"[^a-z ]+", " ", text)
    text = re.sub(r"\s+", " ", text)
    return text


def normalize_chinese(text):
    # Vosk Chinese may output spaces. Remove them for matching.
    return text.replace(" ", "").strip()


def parse_english(text):
    text = normalize_english(text)

    # Prioritize two-word phrases so "sit down" beats "down".
    for phrase in ["sit down", "set down", "stand up", "lay down", "lie down"]:
        if phrase in text:
            return phrase, ENGLISH_PHRASES[phrase]

    for phrase, pose_name in ENGLISH_PHRASES.items():
        if phrase in text.split():
            return phrase, pose_name

    return None, None


def parse_chinese(text):
    text = normalize_chinese(text)

    for phrase, pose_name in CHINESE_PHRASES.items():
        if phrase in text:
            return phrase, pose_name

    return None, None


def should_exit(text_en, text_cn):
    text_en = normalize_english(text_en)
    text_cn = normalize_chinese(text_cn)

    if "stop" in text_en.split() or "exit" in text_en.split() or "quit" in text_en.split():
        return True

    if "停止" in text_cn or "退出" in text_cn or "停下" in text_cn:
        return True

    return False


def main():
    rclpy.init()
    node = VoicePoseController()

    audio_queue = queue.Queue()

    model_en_dir = find_model_dir("model_en")
    model_cn_dir = find_model_dir("model_cn")

    print(f"Using English model: {model_en_dir}")
    print(f"Using Chinese model: {model_cn_dir}")

    model_en = Model(str(model_en_dir))
    model_cn = Model(str(model_cn_dir))

    grammar_en = json.dumps([
        "sit down", "sit", "set down", "set",
        "stand up", "stand", "up",
        "lay down", "lie down", "lay", "lie", "down",
        "stop", "exit", "quit",
        "[unk]",
    ])

    grammar_cn = json.dumps([
        "坐下", "坐",
        "站起来", "站起", "起立", "站立", "站",
        "趴下", "躺下", "卧倒", "趴", "躺",
        "停止", "退出", "停下",
        "[unk]",
    ], ensure_ascii=False)

    sample_rate = 16000
    recognizer_en = KaldiRecognizer(model_en, sample_rate, grammar_en)
    recognizer_cn = KaldiRecognizer(model_cn, sample_rate, grammar_cn)

    def audio_callback(indata, frames, time_info, status):
        if status:
            print(status, file=sys.stderr)
        audio_queue.put(bytes(indata))

    print("")
    print("Bilingual voice control ready.")
    print("English: sit down / stand up / lay down")
    print("Chinese: 坐下 / 站起来 / 趴下")
    print("Say stop / exit / 停止 / 退出 to quit.")
    print("")

    node.wait_for_joint_state()

    try:
        with sd.RawInputStream(
            samplerate=sample_rate,
            blocksize=8000,
            dtype="int16",
            channels=1,
            callback=audio_callback,
        ):
            while rclpy.ok():
                data = audio_queue.get()

                rclpy.spin_once(node, timeout_sec=0.001)

                got_en = recognizer_en.AcceptWaveform(data)
                got_cn = recognizer_cn.AcceptWaveform(data)

                text_en = ""
                text_cn = ""

                if got_en:
                    text_en = json.loads(recognizer_en.Result()).get("text", "").strip()

                if got_cn:
                    text_cn = json.loads(recognizer_cn.Result()).get("text", "").strip()

                if not text_en and not text_cn:
                    continue

                print(f"Heard EN: {text_en}")
                print(f"Heard CN: {text_cn}")

                if should_exit(text_en, text_cn):
                    print("Exiting voice control.")
                    break

                command_word, pose_name = parse_english(text_en)

                if pose_name is None:
                    command_word, pose_name = parse_chinese(text_cn)

                if pose_name is None:
                    continue

                print(f"Command: {command_word} -> {pose_name}")

                try:
                    node.move_to_pose(pose_name)
                except Exception as e:
                    print(f"Failed moving to {pose_name}: {e}")

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
