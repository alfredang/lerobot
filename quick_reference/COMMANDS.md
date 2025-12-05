# Quick Command Reference

All commands copy-paste ready.

---

## 🔐 Access

### SSH into Jetson
```bash
ssh orin_nano@192.168.28.23
# Password: jetson192024
```

### Power Off Jetson
```bash
sudo poweroff
```

---

## 🔍 Check Devices
```bash
# Check cameras
ls -l /dev/video*

# Check USB devices
ls /dev/ttyUSB* /dev/ttyACM*

# Detailed camera info
v4l2-ctl --list-devices
```

---

## 🔓 Permissions (After Each Reboot)
```bash
# All at once
sudo chmod 666 /dev/video0 /dev/video2 /dev/ttyUSB0 /dev/ttyACM0

# Or individually
sudo chmod 666 /dev/ttyACM0
sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/video0
sudo chmod 666 /dev/video2
```

---

## 🤖 Mode 1: Teleoperation

### Jetson Terminal
```bash
ssh orin_nano@192.168.28.23
cd ~/lerobot_lekiwi && conda activate lerobot
sudo chmod 666 /dev/video0 /dev/video2 /dev/ttyACM0

python lerobot/scripts/control_robot.py \
  --robot.type=lekiwi \
  --robot.cameras='{}' \
  --control.type=remote_robot
```

### Laptop Terminal (Direct, NOT SSH)
```bash
cd ~ && conda activate lerobot
sudo chmod 666 /dev/ttyACM0
python run_lekiwi_teleop.py
```

---

## 📸 Mode 2: Vision-Only

### Jetson Terminal 1
```bash
ssh orin_nano@192.168.28.23
cd ~/lerobot_lekiwi && conda activate lerobot
sudo chmod 666 /dev/video0 /dev/video2 /dev/ttyACM0

python lerobot/scripts/control_robot.py \
  --robot.type=lekiwi \
  --robot.cameras='{}' \
  --control.type=remote_robot
```

### Jetson Terminal 2 (New SSH)
```bash
ssh orin_nano@192.168.28.23
cd ~/lerobot_lekiwi && conda activate lerobot
python robot_vision_server.py
```

### Laptop
```bash
cd ~ && conda activate lerobot
sudo chmod 666 /dev/ttyACM0

# Set API key (replace with yours)
export OPENAI_API_KEY="sk-proj-YOUR_KEY"

# Run controller
python chatgpt_lekiwi_final.py
```

### View Cameras
Open browser: **http://192.168.28.23:5001/view**

---

## 🗺️ Mode 3: SLAM 

### Jetson Terminal 1 - Robot Control
```bash
ssh orin_nano@192.168.28.23
cd ~/lerobot_lekiwi && conda activate lerobot
sudo chmod 666 /dev/video0 /dev/video2 /dev/ttyUSB0 /dev/ttyACM0

python lerobot/scripts/control_robot.py \
  --robot.type=lekiwi \
  --robot.cameras='{}' \
  --control.type=remote_robot
```

### Jetson Terminal 2 - Vision
```bash
ssh orin_nano@192.168.28.23
cd ~/lerobot_lekiwi && conda activate lerobot
python robot_vision_server.py
```

### Jetson Terminal 3 - RPLidar
```bash
ssh orin_nano@192.168.28.23
cd ~/ros2_ws_rplidar
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch rplidar_ros view_rplidar_a1_launch.py
```

### Jetson Terminal 4 - SLAM Server
```bash
ssh orin_nano@192.168.28.23
cd ~/lerobot_lekiwi && conda activate lerobot
python slam_map_server.py
```

### Laptop - ChatGPT + SLAM
```bash
cd ~ && conda activate lerobot
sudo chmod 666 /dev/ttyACM0

export OPENAI_API_KEY="sk-proj-YOUR_KEY"
python chatgpt_lekiwi_slam.py
```

---

## 🔧 Calibration

### Calibrate Follower (Jetson - Do First)
```bash
ssh orin_nano@192.168.28.23
cd ~/lerobot_lekiwi && conda activate lerobot
sudo chmod 666 /dev/ttyACM0

python lerobot/scripts/control_robot.py \
  --robot.type=lekiwi \
  --robot.cameras='{}' \
  --control.type=calibrate \
  --control.arms='["main_follower"]'
```

### Calibrate Leader (Laptop - Do Second)
```bash
cd ~/lerobot_working_version_backup
conda activate lerobot
sudo chmod 666 /dev/ttyACM0

python lerobot/scripts/control_robot.py \
  --robot.type=lekiwi \
  --robot.cameras='{}' \
  --control.type=calibrate \
  --control.arms='["main_leader"]'
```

---

## 🔑 OpenAI API Key

### Set API Key
```bash
export OPENAI_API_KEY="sk-proj-YOUR_KEY_HERE"

# Verify
echo $OPENAI_API_KEY
```

### Make Permanent
```bash
echo 'export OPENAI_API_KEY="sk-proj-YOUR_KEY"' >> ~/.bashrc
source ~/.bashrc
```

---

## 📝 Daily Workflow

1. **Power on Jetson**
2. **Start robot control (Jetson T1)**
3. **Start vision server (Jetson T2)**
4. **Set API key and run ChatGPT controller (Laptop)**
5. **Open browser** to view cameras
6. **Give commands:** "pick up the green block"
7. **Shutdown:** Ctrl+C all terminals, `sudo poweroff` on Jetson

---

**Quick access link:** Bookmark this page!
