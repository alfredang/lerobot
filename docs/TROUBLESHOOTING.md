# Troubleshooting Guide

Complete solutions for common issues with the LeKiwi ChatGPT robot system.

---

## Table of Contents

1. [Permission Errors](#1-permission-errors)
2. [Connection Issues](#2-connection-issues)
3. [Motor Problems](#3-motor-problems)
4. [Camera Issues](#4-camera-issues)
5. [ChatGPT API Issues](#5-chatgpt-api-issues)
6. [SLAM Issues](#6-slam-issues)
7. [Device Not Found](#7-device-not-found)
8. [Performance Issues](#9-performance-issues)
9. [Common Error Messages](#10-common-error-messages)

---

## 1. Permission Errors

### Error: "Permission denied: /dev/ttyACM0"

**Cause:** USB device permissions not set

**Solution:**
```bash
sudo chmod 666 /dev/ttyACM0
```

**Make Permanent:**
```bash
sudo usermod -a -G dialout $USER
# Logout and login again
```

---

### Error: "Permission denied: /dev/video0"

**Cause:** Camera device permissions not set

**Solution:**
```bash
sudo chmod 666 /dev/video0 /dev/video2
```

**Make Permanent:**
```bash
sudo usermod -a -G video $USER
# Logout and login again
```

---

### Error: "Permission denied: /dev/ttyUSB0"

**Cause:** RPLidar device permissions not set

**Solution:**
```bash
sudo chmod 666 /dev/ttyUSB0
```

**Make Permanent:**
```bash
sudo usermod -a -G dialout $USER
# Logout and login again
```

---

## 2. Connection Issues

### Can't SSH to Jetson

**Symptoms:**
```
ssh: connect to host 192.168.28.23 port 22: Connection refused
```

**Solutions:**

**Check 1: Verify Jetson is on network**
```bash
ping 192.168.28.23
# Should get replies
```

**Check 2: Verify SSH is running**
```bash
# On Jetson (if you have direct access)
sudo systemctl status ssh
sudo systemctl start ssh
```

**Check 3: Check firewall**
```bash
# On Jetson
sudo ufw status
sudo ufw allow 22
```

---

### ZMQ Connection Timeout

**Error:**
```
ZMQ connection timeout on port 5555
```

**Solutions:**

**Check 1: Is control_robot.py running on Jetson?**
```bash
ssh orin_nano@192.168.28.23
ps aux | grep control_robot
```

**Check 2: Is port blocked?**
```bash
# On Jetson
sudo netstat -tulpn | grep 5555
```

**Check 3: Firewall blocking port?**
```bash
# On Jetson
sudo ufw allow 5555
```

---

### Camera Server Not Responding

**Error:**
```
Connection refused on http://192.168.28.23:5001
```

**Solutions:**

**Check 1: Is vision server running?**
```bash
ssh orin_nano@192.168.28.23
ps aux | grep robot_vision_server
```

**Check 2: Restart vision server**
```bash
ssh orin_nano@192.168.28.23
cd ~/lerobot_lekiwi && conda activate lerobot
python robot_vision_server.py
```

**Check 3: Check firewall**
```bash
sudo ufw allow 5001
```

---

## 3. Motor Problems

### "Motors stopped" Message Constantly

**This is NORMAL** when no commands are being sent.

Only worry if motors stop during active movement.

**If motors stop unexpectedly:**
- Check USB connection to arm
- Verify `/dev/ttyACM0` exists
- Check power supply to motors

---

### Robot Moves in Wrong Direction

**Already fixed in scripts** (x/y axes swapped in code).

**If still occurring:**
- Check leader arm is connected properly
- Verify calibration files are loaded
- Test with teleoperation mode first

---

### Arm Jumps to Wrong Position

**Cause:** Missing or incorrect calibration

**Solution:**

**Check calibration files exist:**
```bash
# Leader (laptop)
ls ~/.cache/calibration/lekiwi/main_leader.json

# Follower (Jetson)
ssh orin_nano@192.168.28.23
ls ~/lerobot_lekiwi/.cache/calibration/lekiwi/main_follower.json
```

**Recalibrate if needed:**
```bash
# Follower first
python lerobot/scripts/control_robot.py \
  --robot.type=lekiwi \
  --robot.cameras='{}' \
  --control.type=calibrate \
  --control.arms='["main_follower"]'

# Leader second
python lerobot/scripts/control_robot.py \
  --robot.type=lekiwi \
  --robot.cameras='{}' \
  --control.type=calibrate \
  --control.arms='["main_leader"]'
```

---

### Motors Not Responding

**Solutions:**

**Check 1: Verify motor power**
- Are motors powered on?
- Check power supply connections

**Check 2: Check USB connection**
```bash
ls /dev/ttyACM*
# Should show /dev/ttyACM0
```

**Check 3: Test communication**
```bash
# Install minicom
sudo apt install minicom

# Test serial port
sudo minicom -D /dev/ttyACM0 -b 1000000
# Press Ctrl+A, then X to exit
```

---

## 4. Camera Issues

### Cameras Not Showing

**Check 1: Verify camera devices**
```bash
ls -l /dev/video*
# Should show /dev/video0 and /dev/video2
```

**Check 2: Test with v4l2**
```bash
v4l2-ctl --list-devices
# Should show both cameras
```

**Check 3: Test capture**
```bash
ffmpeg -f v4l2 -i /dev/video0 -frames:v 1 test.jpg
# Should create test.jpg
```

---

### Black Image / No Feed

**Solutions:**

**Check 1: USB connection**
- Unplug and replug camera USB
- Try different USB port

**Check 2: Restart vision server**
```bash
# Kill existing server
pkill -f robot_vision_server

# Restart
python robot_vision_server.py
```

**Check 3: Check camera permissions**
```bash
sudo chmod 666 /dev/video0 /dev/video2
```

---

### Wrong Camera Assigned

**If front/wrist cameras are swapped:**

**Find correct device IDs:**
```bash
v4l2-ctl --list-devices
```

**Edit script to swap:**
```python
# In robot_vision_server.py, swap these:
self.front_camera = cv2.VideoCapture(0)  # Change number
self.wrist_camera = cv2.VideoCapture(2)  # Change number
```

---

## 5. ChatGPT API Issues

### "Authentication error"

**Check API key is set:**
```bash
echo $OPENAI_API_KEY
# Should show your key starting with sk-proj-
```

**If empty, set it:**
```bash
export OPENAI_API_KEY="sk-proj-YOUR_KEY_HERE"

# Make permanent:
echo 'export OPENAI_API_KEY="sk-proj-YOUR_KEY"' >> ~/.bashrc
source ~/.bashrc
```

---

### "Insufficient credits" / "Quota exceeded"

**Cause:** Not enough OpenAI credit

**Solution:**
1. Go to: https://platform.openai.com/settings/organization/billing
2. Add more credit (minimum $5)
3. Wait 5-10 minutes for activation

---

### "Rate limit exceeded"

**Cause:** Too many API requests

**Solution:**
- Wait 1 minute between tasks
- Reduce `max_steps` in script
- Use longer `time.sleep()` between requests

---

### Invalid JSON Response

**Already handled in script** with fallback parsing.

**If still failing:**
- Check temperature setting (should be 0.3)
- Verify prompt is clear
- Try simpler commands first

---

## 6. SLAM Issues

### "No laser scan received"

**Check 1: LiDAR connected**
```bash
ls /dev/ttyUSB*
# Should show /dev/ttyUSB0
```

**Check 2: LiDAR spinning**
- You should hear motor spinning
- Green light should be on

**Check 3: Permissions**
```bash
sudo chmod 666 /dev/ttyUSB0
```

**Check 4: ROS2 node running**
```bash
ros2 node list
# Should show /rplidar_node or /sllidar_node
```

---

### Map Drifts Over Time

**Normal behavior** for odometry-based SLAM.

**Mitigations:**
- Reduce robot speeds
- Return to start position periodically
- Add more visual features to environment
- Use loop closure (in SLAM Toolbox settings)

---

### SLAM Server Not Responding

**Check:**
```bash
curl http://192.168.28.23:5002/status
# Should return JSON
```

**Restart:**
```bash
ssh orin_nano@192.168.28.23
cd ~/lerobot_lekiwi && conda activate lerobot
python slam_map_server.py
```

---

## 7. Device Not Found

### Check All Connected Devices
```bash
# On Jetson
ls -l /dev/video* /dev/ttyUSB* /dev/ttyACM*

# Expected:
# /dev/video0  - Front camera
# /dev/video2  - Wrist camera
# /dev/ttyUSB0 - RPLidar
# /dev/ttyACM0 - Follower arm + wheels
```
```bash
# On Laptop
ls -l /dev/ttyACM*

# Expected:
# /dev/ttyACM0 - Leader arm
```

---

### Device Path Changed

**If `/dev/video0` became `/dev/video1`:**

**Option 1: Swap USB ports back**

**Option 2: Update script**
```python
# In robot_vision_server.py
self.front_camera = cv2.VideoCapture(1)  # Changed from 0
```

---

## 8. Performance Issues

### Slow Response Time

**Normal:** 2-5 seconds per decision

**If > 10 seconds:**
- Check internet speed
- Reduce image size in script
- Use lower resolution cameras

---

### High CPU Usage

**On Jetson:**
- Normal during camera capture
- Close unused applications
- Increase swap if needed

**On Laptop:**
- Normal during ChatGPT calls
- Close other browsers/apps

---

### Robot Stuttering

**Causes:**
- Network latency
- CPU overload
- USB power issues

**Solutions:**
- Use wired ethernet instead of WiFi
- Reduce control loop frequency
- Use powered USB hub

---

## 9. Common Error Messages

### "ModuleNotFoundError: No module named 'lerobot'"

**Solution:**
```bash
conda activate lerobot
cd ~/lerobot_lekiwi
pip install -e .
```

---

### "ImportError: libGL.so.1"

**Solution:**
```bash
sudo apt install libgl1-mesa-glx
```

---

### "OSError: [Errno 24] Too many open files"

**Solution:**
```bash
ulimit -n 4096
```

**Make permanent:**
```bash
echo "* soft nofile 4096" | sudo tee -a /etc/security/limits.conf
echo "* hard nofile 4096" | sudo tee -a /etc/security/limits.conf
```

---

### "calibration file not found"

**Solution:**
```bash
# Copy calibration files
mkdir -p ~/.cache/calibration/lekiwi
cp calibration/lekiwi/*.json ~/.cache/calibration/lekiwi/
```

---

## Getting Help

If issue persists after trying troubleshooting:

1. Check [Quick Commands](../quick_reference/COMMANDS.md)
2. Review [Installation Guide](INSTALLATION.md)

**Most issues are permission-related or connection-related. Always check devices and permissions first!**
