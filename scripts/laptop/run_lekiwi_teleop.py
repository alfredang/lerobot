#!/usr/bin/env python3
import sys
import termios
import tty
import select
import time
import json
import zmq
from lerobot.common.robot_devices.robots.configs import LeKiwiRobotConfig
from lerobot.common.robot_devices.robots.mobile_manipulator import MobileManipulator

def get_key():
    """Non-blocking keyboard read"""
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None

# Setup
config = LeKiwiRobotConfig()
config.ip = "192.168.28.23"
config.cameras = {}

robot = MobileManipulator(config)
robot.connect()

# Put terminal in raw mode
old_settings = termios.tcgetattr(sys.stdin)
try:
    tty.setcbreak(sys.stdin.fileno())
    
    print("Connected! Use WASD to move, ZX to rotate, Q to quit")
    print("R/F for speed up/down")
    
    speed_index = 0
    speed_levels = [
        {"xy": 0.1, "theta": 30},
        {"xy": 0.2, "theta": 60},
        {"xy": 0.3, "theta": 90},
    ]
    
    while True:
        key = get_key()
        
        # Update speed index
        if key == 'r':
            speed_index = min(speed_index + 1, 2)
            print(f"\nSpeed: {speed_index}")
        elif key == 'f':
            speed_index = max(speed_index - 1, 0)
            print(f"\nSpeed: {speed_index}")
        elif key == 'q':
            print("\nQuitting...")
            break
        
        # Calculate velocities based on current keys
        xy_speed = speed_levels[speed_index]["xy"]
        theta_speed = speed_levels[speed_index]["theta"]
        
        x_cmd = 0.0
        y_cmd = 0.0
        theta_cmd = 0.0
        
        if key == 'w':
            y_cmd = xy_speed
        elif key == 's':
            y_cmd = -xy_speed
        elif key == 'a':
            x_cmd = xy_speed
        elif key == 'd':
            x_cmd = -xy_speed
        elif key == 'z':
            theta_cmd = theta_speed
        elif key == 'x':
            theta_cmd = -theta_speed
        
        # Only send command if there's movement
        if x_cmd != 0 or y_cmd != 0 or theta_cmd != 0:
            print(f"\rMoving: x={x_cmd:.2f} y={y_cmd:.2f} θ={theta_cmd:.1f}  ", end='', flush=True)
            
            # Get arm positions
            arm_positions = []
            for name in robot.leader_arms:
                pos = robot.leader_arms[name].read("Present_Position")
                arm_positions.extend(pos.tolist())
            
            # Calculate wheel commands
            wheel_commands = robot.body_to_wheel_raw(x_cmd, y_cmd, theta_cmd)
            
            # Send command
            message = {"raw_velocity": wheel_commands, "arm_positions": arm_positions}
            robot.cmd_socket.send_string(json.dumps(message))
        else:
            # Send stop command
            arm_positions = []
            for name in robot.leader_arms:
                pos = robot.leader_arms[name].read("Present_Position")
                arm_positions.extend(pos.tolist())
            
            message = {
                "raw_velocity": {"left_wheel": 0, "back_wheel": 0, "right_wheel": 0},
                "arm_positions": arm_positions
            }
            robot.cmd_socket.send_string(json.dumps(message))
        
        time.sleep(0.03)  # ~30Hz

finally:
    # Restore terminal
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    robot.disconnect()
    print("\nDisconnected")
