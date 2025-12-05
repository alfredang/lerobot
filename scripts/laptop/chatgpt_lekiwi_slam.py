#!/usr/bin/env python3
"""
ChatGPT Lekiwi Controller with SLAM
Full autonomous control with map awareness

Usage:
    export OPENAI_API_KEY="sk-proj-..."
    python chatgpt_lekiwi_slam.py
"""

import openai
import requests
import json
import time
import os
import signal
import sys
from lerobot.common.robot_devices.robots.configs import LeKiwiRobotConfig
from lerobot.common.robot_devices.robots.mobile_manipulator import MobileManipulator

# Configuration
JETSON_IP = "192.168.28.23"
VISION_PORT = 5001   # Camera server
SLAM_PORT = 5002     # SLAM data server

openai.api_key = os.getenv("OPENAI_API_KEY")

emergency_stop_robot = None

def signal_handler(sig, frame):
    """Emergency stop on Ctrl+C"""
    print('\n🛑 EMERGENCY STOP')
    global emergency_stop_robot
    if emergency_stop_robot:
        try:
            arm_pos = []
            for name in emergency_stop_robot.leader_arms:
                pos = emergency_stop_robot.leader_arms[name].read("Present_Position")
                arm_pos.extend(pos.tolist())
            
            stop = {
                "raw_velocity": {"left_wheel": 0, "back_wheel": 0, "right_wheel": 0},
                "arm_positions": arm_pos
            }
            for _ in range(10):
                emergency_stop_robot.cmd_socket.send_string(json.dumps(stop))
                time.sleep(0.01)
            print('✅ Stopped')
        except:
            pass
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

SYSTEM_PROMPT = """You are controlling a mobile manipulator robot with SLAM-based navigation.

ROBOT CAPABILITIES:
- Omnidirectional wheeled base (can move in any direction + rotate)
- 6-DOF robotic arm with gripper
- SLAM system that provides:
  * 2D occupancy grid map showing obstacles and free space
  * Precise robot position (x, y in meters, theta in degrees)
  * Memory of known object locations
- 2 cameras: front view + wrist view

YOU RECEIVE 3 IMAGES:
1. Front camera - Shows objects and environment ahead
2. Wrist camera - Close-up view of gripper area
3. Map - Top-down view of explored area
   - White = free space
   - Black = obstacles (walls, furniture)
   - Gray = unexplored
   - Red dot with arrow = your position and orientation
   - Green dots = known objects

CURRENT POSE DATA:
You receive exact position: x=<X>m, y=<Y>m, theta=<THETA>°

ACTIONS:

1. move_base(x_velocity, y_velocity, theta_velocity, duration)
   - x_velocity: forward(+)/backward(-), -0.2 to 0.2 m/s
   - y_velocity: left(+)/right(-), -0.2 to 0.2 m/s
   - theta_velocity: rotate CCW(+)/CW(-), -45 to 45 deg/s
   - duration: 2-5 seconds

2. move_arm(target_description, duration)
   - target_description: ONE of these:
     * "raise_high" - For placing on shelf
     * "lower_to_ground" - For picking from floor
     * "neutral_position" - Medium height
     * "retract_up" - Pull back and up
     * "extend_forward" - Reach forward
   - duration: 2-3 seconds

3. gripper(close)
   - close: true to grab, false to release

4. save_object_location(name)
   - Saves current position as location of named object
   - Use after identifying an object's location
   - Example: After seeing "green_block", save it

5. wait(seconds)

6. done

RESPONSE FORMAT (JSON only):
{
  "observation": "What I see in all 3 views + current position",
  "reasoning": "My plan based on map and vision",
  "action": "move_base|move_arm|gripper|save_object_location|wait|done",
  "parameters": {...},
  "next_step": "What comes next"
}

SLAM-ENHANCED STRATEGIES:

EXPLORATION WITH MAP:
- Look at map to identify unexplored areas (gray regions)
- Navigate to gray areas to expand map
- Use map to avoid retracing paths
- Rotate in place to scan with sensors
- Remember approximate distances from map

OBJECT LOCATION WITH MEMORY:
Step 1: Explore and identify object in camera
Step 2: save_object_location("green_block") when close
Step 3: Continue exploration or proceed to next task
Later: "Navigate toward saved 'green_block' location at (x, y)"

PICKING WITH POSITION AWARENESS:
Step 1: Check map - am I near the object? 
Step 2: Use camera to visually confirm object
Step 3: Fine-tune position using small movements
Step 4: Lower arm, grab, lift

PLACING ON SHELF:
Step 1: Find shelf in camera view
Step 2: save_object_location("shelf") when identified
Step 3: Navigate close to shelf position
Step 4: Raise arm, release object

RETURNING HOME:
Step 1: Check current position on map
Step 2: Navigate back using map (find clear path)
Step 3: Use landmarks in camera to confirm location

IMPORTANT RULES:
- ALWAYS describe all 3 views in observation
- Use map to estimate distances (1 grid square = 1 meter)
- Save object locations when you find important objects
- Use position data to estimate how far to move
- Break complex tasks into 12-20 small steps
- Rotate frequently to scan environment
"""


class SlamEnabledController:
    def __init__(self):
        global emergency_stop_robot
        
        print("🧠 ChatGPT Lekiwi with SLAM")
        print("=" * 60)
        
        # Connect to robot
        config = LeKiwiRobotConfig()
        config.ip = JETSON_IP
        config.cameras = {}
        
        self.robot = MobileManipulator(config)
        self.robot.connect()
        emergency_stop_robot = self.robot
        
        # Save home position
        self.home_arm_positions = self.get_leader_positions()
        
        # Arm presets (in raw motor units)
        self.arm_presets = {
            "home": self.home_arm_positions[:6],
            "neutral_position": [2048, 2200, 1800, 2048, 2048, 1500],
            "lower_to_ground": [2048, 1400, 2600, 2048, 2048, 1500],
            "raise_high": [2048, 2800, 1200, 2048, 2048, 1500],
            "retract_up": [2048, 2600, 1400, 2048, 2048, 1500],
            "extend_forward": [2048, 2000, 2200, 2048, 2048, 1500]
        }
        
        # Current baseline for arm
        self.baseline_arm_positions = self.home_arm_positions
        
        # Test SLAM connection
        print("📡 Testing SLAM server...")
        try:
            r = requests.get(f"http://{JETSON_IP}:{SLAM_PORT}/status", timeout=5)
            if r.status_code == 200:
                print("✅ SLAM server connected")
            else:
                print(f"⚠️ SLAM server status: {r.status_code}")
        except Exception as e:
            print(f"⚠️ SLAM server not available: {e}")
            print("   Continuing without SLAM (vision only mode)")
        
        print("✅ Robot connected")
        print(f"📹 Cameras: http://{JETSON_IP}:{VISION_PORT}/view")
        print("=" * 60)
    
    def get_leader_positions(self):
        """Get current leader arm positions"""
        arm_pos = []
        for name in self.robot.leader_arms:
            pos = self.robot.leader_arms[name].read("Present_Position")
            arm_pos.extend(pos.tolist())
        return arm_pos
    
    def capture_images(self):
        """Get camera images"""
        r = requests.get(f"http://{JETSON_IP}:{VISION_PORT}/capture", timeout=5)
        d = r.json()
        return d['front'], d['wrist']
    
    def capture_slam_data(self):
        """Get SLAM data (map + pose + objects)"""
        try:
            r = requests.get(f"http://{JETSON_IP}:{SLAM_PORT}/slam_data", timeout=5)
            return r.json()
        except:
            # Return dummy data if SLAM not available
            return {
                'map': None,
                'pose': {'x': 0, 'y': 0, 'theta': 0},
                'objects': {},
                'pose_status': 'unavailable'
            }
    
    def save_object_location(self, name):
        """Save current position as object location"""
        try:
            r = requests.post(
                f"http://{JETSON_IP}:{SLAM_PORT}/save_object",
                json={"name": name, "use_current_position": True},
                timeout=5
            )
            if r.status_code == 200:
                data = r.json()
                print(f"✅ Saved '{name}' at ({data['object']['x']:.2f}, {data['object']['y']:.2f})")
                return True
            else:
                print(f"⚠️ Failed to save object: {r.status_code}")
                return False
        except Exception as e:
            print(f"⚠️ Error saving object: {e}")
            return False
    
    def send_to_chatgpt(self, msg, front, wrist, slam_data):
        """Send all data to ChatGPT"""
        from openai import OpenAI
        client = OpenAI(api_key=openai.api_key)
        
        # Build context message
        pose = slam_data['pose']
        objects = slam_data.get('objects', {})
        
        context = f"""
CURRENT STATUS:
===============
Position: x={pose['x']:.2f}m, y={pose['y']:.2f}m, theta={pose['theta']:.1f}°
Known objects: {len(objects)}
"""
        
        if objects:
            context += "\nSaved locations:\n"
            for obj_name, obj_data in objects.items():
                context += f"  - {obj_name}: ({obj_data['x']:.2f}, {obj_data['y']:.2f})\n"
        
        context += f"\nTask: {msg}\n"
        
        # Build message content
        content = [
            {"type": "text", "text": context},
            {"type": "image_url", "image_url": {
                "url": f"data:image/jpeg;base64,{front}",
                "detail": "low"
            }},
            {"type": "image_url", "image_url": {
                "url": f"data:image/jpeg;base64,{wrist}",
                "detail": "low"
            }}
        ]
        
        # Add map if available
        if slam_data.get('map'):
            content.append({
                "type": "image_url",
                "image_url": {
                    "url": f"data:image/jpeg;base64,{slam_data['map']}",
                    "detail": "low"
                }
            })
        
        messages = [
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user", "content": content}
        ]
        
        try:
            r = client.chat.completions.create(
                model="gpt-4o",
                messages=messages,
                max_tokens=600,
                temperature=0.3
            )
            
            text = r.choices[0].message.content.strip()
            
            # Clean JSON
            if "```json" in text:
                text = text.split("```json")[1].split("```")[0].strip()
            elif "```" in text:
                text = text.split("```")[1].split("```")[0].strip()
            
            if "{" in text:
                text = text[text.find("{"):]
            if "}" in text:
                text = text[:text.rfind("}")+1]
            
            return json.loads(text)
        except Exception as e:
            print(f"❌ ChatGPT error: {e}")
            return None
    
    def move_base(self, x, y, theta, dur):
        """Move robot base"""
        print(f"🚗 Base: x={x:.2f} y={y:.2f} θ={theta:.1f}° for {dur}s")
        
        for i in range(int(dur * 30)):
            arm = self.baseline_arm_positions
            wheel = self.robot.body_to_wheel_raw(y, x, theta)
            self.robot.cmd_socket.send_string(json.dumps({
                "raw_velocity": wheel,
                "arm_positions": arm
            }))
            if i % 30 == 0:
                print(f"  {i//30+1}s", end='', flush=True)
            time.sleep(1/30)
        
        print()
        self.robot.cmd_socket.send_string(json.dumps({
            "raw_velocity": {"left_wheel": 0, "back_wheel": 0, "right_wheel": 0},
            "arm_positions": self.baseline_arm_positions
        }))
        print("✅ Stopped")
    
    def move_arm(self, target_desc, dur):
        """Move arm to preset"""
        print(f"🦾 Arm → {target_desc}")
        
        if target_desc not in self.arm_presets:
            print(f"⚠️ Unknown preset: {target_desc}")
            target_desc = "neutral_position"
        
        current = self.baseline_arm_positions[:6]
        target = self.arm_presets[target_desc]
        
        steps = int(dur * 30)
        for i in range(steps):
            progress = (i + 1) / steps
            interpolated = [
                int(current[j] + (target[j] - current[j]) * progress)
                for j in range(6)
            ]
            
            self.robot.cmd_socket.send_string(json.dumps({
                "raw_velocity": {"left_wheel": 0, "back_wheel": 0, "right_wheel": 0},
                "arm_positions": interpolated
            }))
            
            if i % 30 == 0:
                print(f"  {i//30+1}s", end='', flush=True)
            
            time.sleep(1/30)
        
        print()
        self.baseline_arm_positions[:6] = target
        print("✅ Arm moved")
    
    def gripper(self, close):
        """Control gripper"""
        print(f"✋ {'Closing' if close else 'Opening'} gripper...")
        
        target = 3000 if close else 1000
        
        for i in range(40):
            arm = self.baseline_arm_positions.copy()
            current = arm[5]
            progress = (i + 1) / 40
            arm[5] = int(current + (target - current) * progress)
            
            self.robot.cmd_socket.send_string(json.dumps({
                "raw_velocity": {"left_wheel": 0, "back_wheel": 0, "right_wheel": 0},
                "arm_positions": arm
            }))
            time.sleep(0.05)
        
        self.baseline_arm_positions[5] = target
        print("✅ Gripper done")
    
    def execute_action(self, plan):
        """Execute action from ChatGPT plan"""
        action = plan.get('action')
        params = plan.get('parameters', {})
        
        print(f"\n{'='*60}")
        print(f"👁️  OBSERVATION:")
        print(f"    {plan.get('observation', 'N/A')}")
        print(f"\n🧠 REASONING:")
        print(f"    {plan.get('reasoning', 'N/A')}")
        print(f"\n🎬 ACTION: {action.upper()}")
        print(f"📋 NEXT: {plan.get('next_step', 'N/A')}")
        print(f"{'='*60}\n")
        
        try:
            if action == 'move_base':
                self.move_base(
                    params.get('x_velocity', 0),
                    params.get('y_velocity', 0),
                    params.get('theta_velocity', 0),
                    params.get('duration', 2.0)
                )
                return True
            
            elif action == 'move_arm':
                self.move_arm(
                    params.get('target_description', 'neutral_position'),
                    params.get('duration', 2.0)
                )
                return True
            
            elif action == 'gripper':
                self.gripper(params.get('close', True))
                return True
            
            elif action == 'save_object_location':
                name = params.get('name', 'unknown_object')
                self.save_object_location(name)
                return True
            
            elif action == 'wait':
                secs = params.get('seconds', 1)
                print(f"⏳ Waiting {secs}s...")
                time.sleep(secs)
                return True
            
            elif action == 'done':
                print(f"\n{'='*60}")
                print("✅ TASK COMPLETED!")
                print(f"{'='*60}\n")
                return False
            
            else:
                print(f"⚠️ Unknown action: {action}")
                return True
                
        except Exception as e:
            print(f"❌ Execution error: {e}")
            return True
    
    def run_task(self, cmd):
        """Execute complete task"""
        print(f"\n{'='*60}")
        print(f"🎯 TASK: {cmd}")
        print(f"{'='*60}\n")
        
        # Determine max steps
        cmd_lower = cmd.lower()
        if any(w in cmd_lower for w in ['explore', 'scan', 'search', 'map']):
            max_steps = 20
        elif 'pick' in cmd_lower and 'place' in cmd_lower:
            max_steps = 25
        elif any(w in cmd_lower for w in ['pick', 'grab', 'grasp']):
            max_steps = 15
        else:
            max_steps = 10
        
        print(f"📊 Max steps: {max_steps}\n")
        
        for step in range(1, max_steps + 1):
            print(f"\n{'─'*60}")
            print(f"STEP {step}/{max_steps}")
            print(f"{'─'*60}")
            
            try:
                # Capture all data
                print("📸 Capturing cameras...")
                front, wrist = self.capture_images()
                
                print("🗺️  Fetching SLAM data...")
                slam_data = self.capture_slam_data()
                
                # Show current position
                pose = slam_data['pose']
                print(f"📍 Position: x={pose['x']:.2f}m, y={pose['y']:.2f}m, θ={pose['theta']:.1f}°")
                
                # Ask ChatGPT
                print("🤔 Asking ChatGPT...")
                prompt = f"Step {step}/{max_steps}. Task: '{cmd}'. What should I do next?"
                
                plan = self.send_to_chatgpt(prompt, front, wrist, slam_data)
                
                if not plan:
                    print("❌ No valid plan")
                    time.sleep(1)
                    continue
                
                # Execute
                should_continue = self.execute_action(plan)
                
                if not should_continue:
                    break
                
                time.sleep(0.5)
                
            except KeyboardInterrupt:
                raise
            except Exception as e:
                print(f"❌ Step error: {e}")
                continue
        
        if step >= max_steps:
            print(f"\n⚠️ Reached max steps ({max_steps})")
    
    def interactive(self):
        """Interactive mode"""
        print("\n🎮 SLAM-ENABLED INTERACTIVE MODE")
        print("="*60)
        print("\nCommands:")
        print("  • 'explore the area and build a map'")
        print("  • 'find the green block'")
        print("  • 'pick up the green block and place it on the shelf'")
        print("  • 'return to home position'")
        print("  • 'quit'")
        print(f"\n📹 Cameras: http://{JETSON_IP}:{VISION_PORT}/view")
        print("="*60)
        
        while True:
            try:
                cmd = input("\n🎤 Command: ").strip()
                
                if not cmd:
                    continue
                
                if cmd.lower() in ['quit', 'q', 'exit']:
                    print("👋 Shutting down...")
                    break
                
                self.run_task(cmd)
                
            except KeyboardInterrupt:
                signal_handler(None, None)
        
        self.robot.disconnect()


if __name__ == "__main__":
    if not openai.api_key:
        print("❌ Set OPENAI_API_KEY environment variable")
        print("\n   export OPENAI_API_KEY='sk-proj-...'")
        exit(1)
    
    try:
        controller = SlamEnabledController()
        controller.interactive()
    except Exception as e:
        print(f"❌ Fatal error: {e}")
        import traceback
        traceback.print_exc()