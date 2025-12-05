# Installation Guide

Complete setup for LeKiwi ChatGPT robot system.

---

## System Information

**Jetson Orin Nano:**
- IP: `192.168.28.23`
- Username: `orin_nano`
- Password: `jetson192024`
- Working dir: `~/lerobot_lekiwi`

**Laptop:**
- Working dir: `~`
- Leader arm via USB

---

## Prerequisites Checklist

- [ ] Conda installed with Python 3.10+
- [ ] LeRobot framework (working version)
- [ ] All USB devices connected
- [ ] OpenAI API key with $5+ credit
- [ ] ROS2 Humble (SLAM mode only)

---

## Installation Steps

### 1. Check Existing Setup

**On Jetson:**
```bash
ssh orin_nano@192.168.28.23
# Password: jetson192024

# Check conda
conda --version

# Check LeRobot
cd ~/lerobot_lekiwi
conda activate lerobot
python -c "import lerobot; print('OK')"
```

**On Laptop:**
```bash
# Check conda
conda --version

# Check LeRobot
cd ~/lerobot_working_version_backup
conda activate lerobot
python -c "import lerobot; print('OK')"
```

### 2. Install Missing Packages

**Jetson:**
```bash
ssh orin_nano@192.168.28.23
conda activate lerobot

pip install flask opencv-python
```

**Laptop:**
```bash
conda activate lerobot
pip install openai requests pillow pyzmq flask
```

### 3. Set Up Calibration Files

**Laptop:**
```bash
mkdir -p ~/.cache/calibration/lekiwi
cp ~/lekiwi-chatgpt-robot/calibration/lekiwi/main_leader.json ~/.cache/calibration/lekiwi/

# Verify
ls ~/.cache/calibration/lekiwi/
```

**Jetson:**
```bash
# Should already exist at:
ls ~/lerobot_lekiwi/.cache/calibration/lekiwi/main_follower.json
```

### 4. Configure OpenAI API
```bash
# Get key from: https://platform.openai.com/api-keys
# Add billing: https://platform.openai.com/settings/organization/billing

# Set permanently
echo 'export OPENAI_API_KEY="sk-proj-YOUR_KEY"' >> ~/.bashrc
source ~/.bashrc

# Verify
echo $OPENAI_API_KEY
```

⚠️ **Important:** Minimum $5 credit required to use API

### 5. Test Installation

**Test robot control:**
```bash
ssh orin_nano@192.168.28.23
cd ~/lerobot_lekiwi && conda activate lerobot
sudo chmod 666 /dev/ttyACM0

python lerobot/scripts/control_robot.py \
  --robot.type=lekiwi \
  --robot.cameras='{}' \
  --control.type=remote_robot
```

Should see: "LeKiwi robot server started..."

Press Ctrl+C to stop.

**Test vision server:**
```bash
ssh orin_nano@192.168.28.23
cd ~/lerobot_lekiwi && conda activate lerobot
python robot_vision_server.py
```

Open browser: http://192.168.28.23:5001/view

Should see both cameras.

---

## Troubleshooting

**"Permission denied /dev/ttyACM0"**
```bash
sudo chmod 666 /dev/ttyACM0
```

**"ModuleNotFoundError: lerobot"**
```bash
conda activate lerobot
pip install -e ~/lerobot_lekiwi
```

**"Connection refused to Jetson"**
```bash
ping 192.168.28.23
# Should get replies
```

---

## Next Steps

- [Quick Commands](../quick_reference/COMMANDS.md)
- [Troubleshooting](TROUBLESHOOTING.md)
- Test teleoperation mode

---

**Installation complete!**
