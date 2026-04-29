# Pupper Voice Pose Control

This project controls a Pupper v3 robot using saved pose CSV files and bilingual voice commands.

The robot can respond to:

- English: `sit down`, `stand up`, `lay down`
- Chinese: `坐下`, `站起来`, `趴下`

The final voice-control script directly maps each command to a saved pose:

- `sit down` / `坐下` → `sit_v2.csv`
- `stand up` / `站起来` → `final_stand_v2.csv`
- `lay down` / `趴下` → `lay_v2.csv`

---

## Project Structure

```text
pupper-pose-control/
├── model_en/
├── model_cn/
├── poses/
│   ├── sit_v2.csv
│   ├── final_stand_v2.csv
│   └── lay_v2.csv
├── play_pose.py
├── ps4_pose_tuner.py
├── voice_pose_control_bilingual.py
└── README.md
```

---

## Files

### `play_pose.py`

Loads one saved pose CSV and moves the robot smoothly into that pose.

Example:

```bash
python3 play_pose.py sit_v2
python3 play_pose.py final_stand_v2
python3 play_pose.py lay_v2
```

### `ps4_pose_tuner.py`

Uses a PS4 controller to tune joint positions and save the robot pose as a CSV file.

Example:

```bash
python3 ps4_pose_tuner.py --ros-args -p pose_name:=sit_v2 -p step:=0.03
```

### `voice_pose_control_bilingual.py`

Runs bilingual voice control using Vosk speech recognition.

Example:

```bash
python3 voice_pose_control_bilingual.py
```

---

## Required Robot Setup

Robot IP used during testing:

```text
10.20.13.119
```

The robot should already have the Pupper v3 ROS2 workspace:

```bash
~/pupperv3-monorepo/ros2_ws/
```

The custom files should be copied into:

```bash
~/pupper_robot_testing/
```

Pose CSV files should be copied into:

```bash
~/pupper_pose_tuner/poses/
```

---

# Setup Steps

## 1. Clone or download this repo

On your computer, download or clone this repo.

If using Git:

```bash
git clone <your-repo-url>
cd pupper-pose-control
```

---

## 2. Copy files to the robot

From Windows PowerShell, inside the repo folder:

```powershell
scp .\play_pose.py pi@10.20.13.119:/home/pi/pupper_robot_testing/
scp .\ps4_pose_tuner.py pi@10.20.13.119:/home/pi/pupper_robot_testing/
scp .\voice_pose_control_bilingual.py pi@10.20.13.119:/home/pi/pupper_robot_testing/
```

Copy the Vosk voice models:

```powershell
scp -r .\model_en pi@10.20.13.119:/home/pi/pupper_robot_testing/
scp -r .\model_cn pi@10.20.13.119:/home/pi/pupper_robot_testing/
```

Copy the final pose files:

```powershell
scp .\poses\sit_v2.csv pi@10.20.13.119:/home/pi/pupper_pose_tuner/poses/
scp .\poses\final_stand_v2.csv pi@10.20.13.119:/home/pi/pupper_pose_tuner/poses/
scp .\poses\lay_v2.csv pi@10.20.13.119:/home/pi/pupper_pose_tuner/poses/
```

If the destination folders do not exist on the robot, SSH into the robot and run:

```bash
mkdir -p ~/pupper_robot_testing
mkdir -p ~/pupper_pose_tuner/poses
```

---

## 3. SSH into the robot

From your computer:

```bash
ssh pi@10.20.13.119
```

You should see:

```text
pi@pupper:~ $
```

---

## 4. Install voice dependencies

On the robot:

```bash
pip3 install vosk sounddevice
```

If already installed, the terminal will say `Requirement already satisfied`.

---

## 5. Start the robot stack

Use **Terminal 1**.

SSH into the robot:

```bash
ssh pi@10.20.13.119
```

Then run:

```bash
source ~/pupperv3-monorepo/ros2_ws/install/local_setup.bash
ros2 launch neural_controller launch.py teleop:=False bag_recorder:=False
```

Leave this terminal running.

Do not run other commands in this terminal.

---

## 6. Load forward controllers

Open **Terminal 2**.

SSH into the robot again:

```bash
ssh pi@10.20.13.119
```

Then run:

```bash
source ~/pupperv3-monorepo/ros2_ws/install/local_setup.bash
```

Load the forward controllers:

```bash
ros2 control load_controller --set-state active forward_position_controller
ros2 control load_controller --set-state active forward_kp_controller
ros2 control load_controller --set-state active forward_kd_controller
```

Check controllers:

```bash
ros2 control list_controllers
```

You should see:

```text
forward_position_controller    active
forward_kp_controller          active
forward_kd_controller          active
joint_state_broadcaster        active
imu_sensor_broadcaster         active
```

---

## 7. Test pose playback

In Terminal 2:

```bash
cd ~/pupper_robot_testing
source ~/pupperv3-monorepo/ros2_ws/install/local_setup.bash
```

Stop any old control scripts:

```bash
pkill -f ps4_pose_tuner.py
pkill -f play_pose.py
pkill -f voice_pose_control_bilingual.py
```

Test the three final poses:

```bash
python3 play_pose.py sit_v2
```

```bash
python3 play_pose.py final_stand_v2
```

```bash
python3 play_pose.py lay_v2
```

If all three poses work, voice control is ready.

---

## 8. Run bilingual voice control

In Terminal 2:

```bash
cd ~/pupper_robot_testing
source ~/pupperv3-monorepo/ros2_ws/install/local_setup.bash

pkill -f ps4_pose_tuner.py
pkill -f play_pose.py
pkill -f voice_pose_control_bilingual.py

python3 voice_pose_control_bilingual.py
```

Expected output:

```text
Bilingual voice control ready.
English: sit down / stand up / lay down
Chinese: 坐下 / 站起来 / 趴下
Got /joint_states.
```

Now say:

```text
sit down
stand up
lay down
坐下
站起来
趴下
```

Say `stop`, `exit`, `停止`, or `退出` to quit.

---

# Tuning Workflow

Use `ps4_pose_tuner.py` if a pose needs to be adjusted.

Example:

```bash
cd ~/pupper_robot_testing
source ~/pupperv3-monorepo/ros2_ws/install/local_setup.bash

python3 ps4_pose_tuner.py --ros-args -p pose_name:=sit_v2 -p step:=0.03
```

Suggested tuning names:

```text
sit_v2
final_stand_v2
lay_v2
```

Do not overwrite a working pose without backing it up first.

Backup example:

```bash
cp ~/pupper_pose_tuner/poses/sit_v2.csv ~/pupper_pose_tuner/poses/sit_v2_backup.csv
```

---

# Safety Notes

Always test poses with the robot supported first.

Do not run these at the same time:

```text
ps4_pose_tuner.py
play_pose.py
voice_pose_control_bilingual.py
```

They all publish to the same robot command topics and will fight for control.

Before testing, run:

```bash
pkill -f ps4_pose_tuner.py
pkill -f play_pose.py
pkill -f voice_pose_control_bilingual.py
```

---

# Troubleshooting

## Problem: `No /joint_states received after 5 seconds`

The robot stack is not running or the joint state broadcaster is not active.

Check:

```bash
ros2 topic echo /joint_states --once
```

If nothing prints, restart Terminal 1:

```bash
source ~/pupperv3-monorepo/ros2_ws/install/local_setup.bash
ros2 launch neural_controller launch.py teleop:=False bag_recorder:=False
```

Then reload controllers in Terminal 2.

---

## Problem: Voice model not found

The script expects:

```text
~/pupper_robot_testing/model_en
~/pupper_robot_testing/model_cn
```

Check:

```bash
ls ~/pupper_robot_testing
```

If missing, copy them from your computer:

```powershell
scp -r .\model_en pi@10.20.13.119:/home/pi/pupper_robot_testing/
scp -r .\model_cn pi@10.20.13.119:/home/pi/pupper_robot_testing/
```

---

## Problem: FastDDS / RTPS shared memory error

You may see:

```text
RTPS_TRANSPORT_SHM Error
```

If the robot still moves, ignore it.

If ROS behaves weirdly, clean stale FastDDS shared-memory files:

```bash
pkill -f ps4_pose_tuner.py
pkill -f play_pose.py
pkill -f voice_pose_control_bilingual.py

sudo rm -f /dev/shm/fastrtps_*
sudo rm -f /dev/shm/fastrtps_port*
```

Then restart the robot stack and reload controllers.

---

# Normal Demo Checklist

Every demo session:

```text
1. Power on robot.
2. SSH into robot in Terminal 1.
3. Launch neural_controller.
4. SSH into robot in Terminal 2.
5. Load forward controllers.
6. Test one pose.
7. Run voice control.
```

Terminal 1:

```bash
source ~/pupperv3-monorepo/ros2_ws/install/local_setup.bash
ros2 launch neural_controller launch.py teleop:=False bag_recorder:=False
```

Terminal 2:

```bash
source ~/pupperv3-monorepo/ros2_ws/install/local_setup.bash

ros2 control load_controller --set-state active forward_position_controller
ros2 control load_controller --set-state active forward_kp_controller
ros2 control load_controller --set-state active forward_kd_controller

cd ~/pupper_robot_testing
python3 voice_pose_control_bilingual.py
```
