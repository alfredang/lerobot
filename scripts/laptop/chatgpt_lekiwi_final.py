#!/usr/bin/env python3
"""
ChatGPT Lekiwi - FINAL VERSION
Full autonomous control with no hardcoding
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

JETSON_IP = "192.168.28.23"
VISION_PORT = 5001  

openai.api_key = os.getenv("OPENAI_API_KEY")

emergency_stop_robot = None

def signal_handler(sig, frame):
    print('\n🛑 EMERGENCY STOP')
    global emergency_stop_robot
    if emergency_stop_robot:
        try:
            arm_pos = []
            for name in emergency_stop_robot.leader_arms:
                pos = emergency_stop_robot.leader_arms[name].read("Present_Position")
                arm_pos.extend(pos.tolist())
            
            stop = {"raw_velocity": {"left_wheel": 0, "back_wheel": 0, "right_wheel": 0}, "arm_positions": arm_pos}
            for _ in range(10):
                emergency_stop_robot.cmd_socket.send_string(json.dumps(stop))
                time.sleep(0.01)
        except:
            pass
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

SYSTEM_PROMPT = """You are controlling a mobile manipulator robot autonomously using ONLY vision. NO pre-programmed positions exist.

ROBOT CAPABILITIES:
- Omnidirectional wheeled base (can move forward/back, strafe left/right, rotate)
- 6-DOF arm with gripper
- 2 cameras: front camera (shows environment) + wrist camera (close-up view)

AVAILABLE ACTIONS:

1. move_base(x_velocity, y_velocity, theta_velocity, duration)
   - x_velocity: forward(+) or backward(-), range: -0.2 to 0.2 m/s
   - y_velocity: left(+) or right(-), range: -0.2 to 0.2 m/s
   - theta_velocity: rotate counterclockwise(+) or clockwise(-), range: -45 to 45 deg/s
   - duration: 2-5 seconds
   Example: {"x_velocity": 0.15, "y_velocity": 0, "theta_velocity": 0, "duration": 3}

2. move_arm(target_description, duration)
   - target_description: ONE of these exact strings:
     * "raise_high" - Arm extends upward (for placing on shelf)
     * "lower_to_ground" - Arm lowers to ground level (for picking objects)
     * "neutral_position" - Arm at medium height, slightly forward
     * "retract_up" - Pull arm back and up (after grasping)
     * "extend_forward" - Reach forward at current height
   - duration: 2-3 seconds
   Example: {"target_description": "lower_to_ground", "duration": 2}

3. gripper(close)
   - close: true to close gripper (grab), false to open gripper (release)
   Example: {"close": true}

4. wait(seconds)
   - Pause for observation
   Example: {"seconds": 1}

5. done
   - Task is complete

RESPONSE FORMAT (JSON only, no markdown):
{
  "observation": "Detailed description of what you see in BOTH camera views",
  "reasoning": "Your specific plan for THIS step only",
  "action": "move_base|move_arm|gripper|wait|done",
  "parameters": {...},
  "next_step": "Brief description of what comes after this step"
}

TASK STRATEGIES:

EXPLORATION:
- Rotate in place to scan area (theta_velocity: 20-30)
- Move forward, then rotate
- Strafe sideways to see different angles
- Look for objects and shelves
- Execute 8-12 varied movements

PICKING UP OBJECT:
Step 1: Observe object location, rotate base to face it
Step 2: Move forward to approach object (x_velocity: 0.1-0.15)
Step 3: Fine-tune position with small movements
Step 4: move_arm("lower_to_ground") to lower arm
Step 5: Small base adjustments to align gripper
Step 6: gripper(close=true) to grab
Step 7: move_arm("retract_up") to lift object
Step 8: done (or continue to placement)

PLACING ON SHELF:
Step 1: Identify shelf location in camera
Step 2: Navigate to shelf (move_base)
Step 3: Rotate to face shelf directly
Step 4: move_arm("raise_high") to shelf height
Step 5: Move forward closer to shelf
Step 6: gripper(close=false) to release
Step 7: move_arm("neutral_position") to retract
Step 8: done (or return home)

RETURNING HOME:
Step 1: move_arm("neutral_position")
Step 2: Rotate to face starting area
Step 3: Navigate back using landmarks in camera
Step 4: done

IMPORTANT RULES:
- ALWAYS describe what you see in both camera views
- Use small velocities initially (0.1-0.15 m/s)
- Rotate frequently during exploration (theta_velocity: 20-40)
- Break complex tasks into 10-20 small steps
- Adjust based on what you observe
- If you can't see the object, explore more
- Use the wrist camera to verify alignment when picking
"""


class FinalAutonomousController:
    def __init__(self):
        global emergency_stop_robot
        
        print("🧠 ChatGPT Lekiwi - FINAL AUTONOMOUS VERSION")
        print("=" * 60)
        
        config = LeKiwiRobotConfig()
        config.ip = JETSON_IP
        config.cameras = {}
        
        self.robot = MobileManipulator(config)
        self.robot.connect()
        emergency_stop_robot = self.robot
        
        # Save HOME position (current leader arm position)
        self.home_arm_positions = self.get_leader_positions()
        print(f"🏠 Home position saved")
        
        # Define arm presets in RAW UNITS (not degrees)
        self.arm_presets = {
            "home": self.home_arm_positions[:6],
            "neutral_position": [2048, 2200, 1800, 2048, 2048, 1500],
            "lower_to_ground": [2048, 1400, 2600, 2048, 2048, 1500],
            "raise_high": [2048, 2800, 1200, 2048, 2048, 1500],
            "retract_up": [2048, 2600, 1400, 2048, 2048, 1500],
            "extend_forward": [2048, 2000, 2200, 2048, 2048, 1500]
        }
        
        print("✅ Connected")
        print(f"📹 Camera view: http://{JETSON_IP}:{VISION_PORT}/view")
        print("=" * 60)
    
    def get_leader_positions(self):
        """Get current arm positions in raw units"""
        arm_pos = []
        for name in self.robot.leader_arms:
            pos = self.robot.leader_arms[name].read("Present_Position")
            arm_pos.extend(pos.tolist())
        return arm_pos
    
    def capture_images(self):
        r = requests.get(f"http://{JETSON_IP}:{VISION_PORT}/capture", timeout=5)
        d = r.json()
        return d['front'], d['wrist']
    
    def send_to_chatgpt(self, msg, front, wrist):
        from openai import OpenAI
        client = OpenAI(api_key=openai.api_key)
        
        messages = [
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user", "content": [
                {"type": "text", "text": msg},
                {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{front}", "detail": "low"}},
                {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{wrist}", "detail": "low"}}
            ]}
        ]
        
        try:
            r = client.chat.completions.create(
                model="gpt-4o",
                messages=messages,
                max_tokens=600,
                temperature=0.3
            )
            
            text = r.choices[0].message.content.strip()
            
            # Remove markdown
            if "```json" in text:
                text = text.split("```json")[1].split("```")[0].strip()
            elif "```" in text:
                text = text.split("```")[1].split("```")[0].strip()
            
            # Remove any text before first {
            if "{" in text:
                text = text[text.find("{"):]
            if "}" in text:
                text = text[:text.rfind("}")+1]
            
            return json.loads(text)
        except json.JSONDecodeError as e:
            print(f"❌ JSON error: {e}")
            print(f"Response: {text[:200]}")
            return None
        except Exception as e:
            print(f"❌ Error: {e}")
            return None
    
    def move_base(self, x, y, theta, dur):
        print(f"🚗 Base: x={x:.2f} y={y:.2f} θ={theta:.1f}° for {dur}s")
        
        for i in range(int(dur * 30)):
            arm = self.get_leader_positions()
            wheel = self.robot.body_to_wheel_raw(y, x, theta)  # Swapped x,y
            self.robot.cmd_socket.send_string(json.dumps({
                "raw_velocity": wheel,
                "arm_positions": arm
            }))
            if i % 30 == 0:
                print(f"  {i//30+1}s", end='', flush=True)
            time.sleep(1/30)
        
        print()
        arm = self.get_leader_positions()
        self.robot.cmd_socket.send_string(json.dumps({
            "raw_velocity": {"left_wheel": 0, "back_wheel": 0, "right_wheel": 0},
            "arm_positions": arm
        }))
        print("✅ Stopped")
    
    def move_arm(self, target_desc, dur):
        """Move arm to preset position"""
        print(f"🦾 Arm → {target_desc}")
        
        if target_desc not in self.arm_presets:
            print(f"⚠️ Unknown preset: {target_desc}, using neutral")
            target_desc = "neutral_position"
        
        current = self.get_leader_positions()[:6]
        target = self.arm_presets[target_desc]
        
        print(f"   From: {[int(c) for c in current]}")
        print(f"   To:   {[int(t) for t in target]}")
        
        # Smooth interpolation
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
        print("✅ Arm moved")
    
    def gripper(self, close):
        print(f"✋ {'Closing' if close else 'Opening'} gripper...")
        
        target = 3000 if close else 1000
        
        for i in range(40):
            arm = self.get_leader_positions()
            current_grip = arm[5]
            progress = (i + 1) / 40
            new_grip = int(current_grip + (target - current_grip) * progress)
            arm[5] = new_grip
            
            self.robot.cmd_socket.send_string(json.dumps({
                "raw_velocity": {"left_wheel": 0, "back_wheel": 0, "right_wheel": 0},
                "arm_positions": arm
            }))
            time.sleep(0.05)
        
        print("✅ Gripper done")
    
    def run(self, cmd):
        print(f"\n{'='*60}")
        print(f"🎯 TASK: {cmd}")
        print(f"{'='*60}\n")
        
        # Determine max steps based on task complexity
        cmd_lower = cmd.lower()
        if 'explore' in cmd_lower or 'scan' in cmd_lower or 'search' in cmd_lower:
            max_steps = 15
        elif 'pick' in cmd_lower and 'place' in cmd_lower:
            max_steps = 20  # Pick + place needs more steps
        elif 'pick' in cmd_lower or 'grab' in cmd_lower:
            max_steps = 12
        elif 'return' in cmd_lower or 'home' in cmd_lower:
            max_steps = 8
        else:
            max_steps = 10
        
        print(f"📊 Max steps: {max_steps}\n")
        
        for step in range(1, max_steps + 1):
            print(f"\n{'─'*60}")
            print(f"STEP {step}/{max_steps}")
            print(f"{'─'*60}")
            
            print("📸 Capturing images...")
            try:
                front, wrist = self.capture_images()
            except Exception as e:
                print(f"❌ Camera error: {e}")
                break
            
            print("🤔 Asking ChatGPT...")
            prompt = f"Step {step} of {max_steps}. Task: '{cmd}'. Based on what you see, what should I do next?"
            
            plan = self.send_to_chatgpt(prompt, front, wrist)
            
            if not plan:
                print("❌ No valid plan, trying next step...")
                time.sleep(1)
                continue
            
            print(f"\n👁️  OBSERVATION:")
            print(f"    {plan.get('observation', 'N/A')}")
            print(f"\n🧠 REASONING:")
            print(f"    {plan.get('reasoning', 'N/A')}")
            print(f"\n🎬 ACTION: {plan.get('action', 'unknown').upper()}")
            print(f"📋 NEXT: {plan.get('next_step', 'N/A')}\n")
            
            action = plan.get('action')
            params = plan.get('parameters', {})
            
            try:
                if action == 'move_base':
                    self.move_base(
                        params.get('x_velocity', 0),
                        params.get('y_velocity', 0),
                        params.get('theta_velocity', 0),
                        params.get('duration', 2.0)
                    )
                
                elif action == 'move_arm':
                    self.move_arm(
                        params.get('target_description', 'neutral_position'),
                        params.get('duration', 2.0)
                    )
                
                elif action == 'gripper':
                    self.gripper(params.get('close', True))
                
                elif action == 'wait':
                    secs = params.get('seconds', 1)
                    print(f"⏳ Waiting {secs}s...")
                    time.sleep(secs)
                
                elif action == 'done':
                    print(f"\n{'='*60}")
                    print("✅ TASK COMPLETED!")
                    print(f"{'='*60}\n")
                    break
                
                else:
                    print(f"⚠️ Unknown action: {action}")
                
            except Exception as e:
                print(f"❌ Execution error: {e}")
                continue
            
            time.sleep(0.5)  # Brief pause between steps
        
        if step >= max_steps:
            print(f"\n⚠️ Reached maximum steps ({max_steps})")
    
    def interactive(self):
        print("\n🎮 INTERACTIVE MODE - FULL AUTONOMY")
        print("="*60)
        print("\nExample commands:")
        print("  • 'explore the area and find objects'")
        print("  • 'pick up the green block'")
        print("  • 'pick up the green block and place it on the shelf'")
        print("  • 'pick up the green block, place it on the top shelf, then return home'")
        print("  • 'move arm to neutral position'")
        print("  • 'quit'")
        print(f"\n📹 Live cameras: http://{JETSON_IP}:{VISION_PORT}/view")
        print("="*60)
        
        while True:
            try:
                cmd = input("\n🎤 Command: ").strip()
                
                if not cmd:
                    continue
                
                if cmd.lower() in ['quit', 'q', 'exit']:
                    print("👋 Shutting down...")
                    break
                
                self.run(cmd)
                
            except KeyboardInterrupt:
                signal_handler(None, None)
        
        self.robot.disconnect()


if __name__ == "__main__":
    if not openai.api_key:
        print("❌ Set OPENAI_API_KEY environment variable")
        exit(1)
    
    try:
        controller = FinalAutonomousController()
        controller.interactive()
    except Exception as e:
        print(f"❌ Fatal error: {e}")
        import traceback
        traceback.print_exc()