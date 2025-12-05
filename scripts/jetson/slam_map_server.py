#!/usr/bin/env python3
"""
SLAM Map Server - Runs on Jetson Orin Nano
Bridges ROS2 SLAM data (map + pose) to HTTP REST API

Provides:
- Current robot position (x, y, theta)
- Occupancy grid map as image
- Known object locations (memory)

Usage:
    python slam_map_server.py
    
Endpoints:
    GET /slam_data - Returns {"map": "<base64>", "pose": {...}, "objects": {...}}
    POST /save_object - Save object location
    GET /status - Health check
"""

from flask import Flask, jsonify, request
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import TransformListener, Buffer
import numpy as np
import cv2
import base64
import threading
import time
import math

app = Flask(__name__)

class SlamDataServer(Node):
    def __init__(self):
        super().__init__('slam_data_server')
        
        self.get_logger().info('SLAM Data Server initializing...')
        
        # QoS profile for SLAM topics
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribe to map (occupancy grid)
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            qos
        )
        
        # Subscribe to robot pose
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/pose',
            self.pose_callback,
            qos
        )
        
        # TF2 buffer for transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Current state
        self.current_map_image = None
        self.current_map_data = None
        self.map_metadata = None
        self.current_pose = {
            "x": 0.0,
            "y": 0.0,
            "theta": 0.0,
            "confidence": 0.0,
            "timestamp": 0
        }
        
        # Object memory (locations of found objects)
        self.known_objects = {}
        
        self.get_logger().info('SLAM Data Server ready!')
    
    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to euler angles (roll, pitch, yaw)"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def pose_callback(self, msg):
        """Update current robot pose"""
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        
        # Convert quaternion to yaw angle
        _, _, yaw = self.quaternion_to_euler(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        )
        
        # Extract covariance for confidence estimate
        covariance = msg.pose.covariance
        position_variance = covariance[0] + covariance[7]  # x + y variance
        confidence = 1.0 / (1.0 + position_variance) if position_variance > 0 else 1.0
        
        self.current_pose = {
            "x": float(position.x),
            "y": float(position.y),
            "theta": float(math.degrees(yaw)),  # Convert to degrees
            "confidence": float(confidence),
            "timestamp": time.time()
        }
        
        self.get_logger().info(
            f'Pose: x={position.x:.2f}m, y={position.y:.2f}m, θ={math.degrees(yaw):.1f}°',
            throttle_duration_sec=2.0
        )
    
    def map_callback(self, msg):
        """Process occupancy grid map"""
        try:
            # Extract map metadata
            self.map_metadata = {
                "width": msg.info.width,
                "height": msg.info.height,
                "resolution": msg.info.resolution,  # meters per pixel
                "origin_x": msg.info.origin.position.x,
                "origin_y": msg.info.origin.position.y
            }
            
            # Convert occupancy grid to numpy array
            width = msg.info.width
            height = msg.info.height
            data = np.array(msg.data, dtype=np.int8).reshape((height, width))
            
            # Store raw data for coordinate conversions
            self.current_map_data = data
            
            # Create visualization image
            # Occupancy values: 0 = free, 100 = occupied, -1 = unknown
            img = np.zeros((height, width, 3), dtype=np.uint8)
            
            # Color mapping
            img[data == -1] = [128, 128, 128]     # Unknown = gray
            img[data == 0] = [255, 255, 255]      # Free = white
            img[(data > 0) & (data <= 100)] = [0, 0, 0]  # Occupied = black
            
            # Draw robot position on map
            if self.current_pose["x"] != 0 or self.current_pose["y"] != 0:
                robot_pixel = self.world_to_pixel(
                    self.current_pose["x"],
                    self.current_pose["y"]
                )
                
                if robot_pixel:
                    px, py = robot_pixel
                    
                    # Draw robot as red circle
                    cv2.circle(img, (px, py), 5, (0, 0, 255), -1)
                    
                    # Draw orientation arrow
                    theta_rad = math.radians(self.current_pose["theta"])
                    arrow_length = 15
                    end_x = int(px + arrow_length * math.cos(theta_rad))
                    end_y = int(py + arrow_length * math.sin(theta_rad))
                    cv2.arrowedLine(img, (px, py), (end_x, end_y), (255, 0, 0), 2)
            
            # Draw known objects
            for obj_name, obj_data in self.known_objects.items():
                obj_pixel = self.world_to_pixel(obj_data["x"], obj_data["y"])
                if obj_pixel:
                    px, py = obj_pixel
                    # Draw object as green circle
                    cv2.circle(img, (px, py), 4, (0, 255, 0), -1)
                    # Add label
                    cv2.putText(img, obj_name, (px + 8, py - 8),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
            
            # Add grid lines for scale
            grid_spacing = int(1.0 / msg.info.resolution)  # 1 meter grid
            for i in range(0, width, grid_spacing):
                cv2.line(img, (i, 0), (i, height), (200, 200, 200), 1)
            for i in range(0, height, grid_spacing):
                cv2.line(img, (0, i), (width, i), (200, 200, 200), 1)
            
            # Flip image (ROS maps have origin at bottom-left, images at top-left)
            img = cv2.flip(img, 0)
            
            # Encode as JPEG then base64
            _, buffer = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 85])
            self.current_map_image = base64.b64encode(buffer).decode('utf-8')
            
            self.get_logger().info(
                f'Map updated: {width}x{height}, res={msg.info.resolution:.3f}m/px',
                throttle_duration_sec=5.0
            )
            
        except Exception as e:
            self.get_logger().error(f'Map processing error: {str(e)}')
    
    def world_to_pixel(self, world_x, world_y):
        """Convert world coordinates (meters) to pixel coordinates"""
        if self.map_metadata is None:
            return None
        
        # Transform to map frame
        pixel_x = int((world_x - self.map_metadata["origin_x"]) / self.map_metadata["resolution"])
        pixel_y = int((world_y - self.map_metadata["origin_y"]) / self.map_metadata["resolution"])
        
        # Flip y (image coordinates are flipped)
        pixel_y = self.map_metadata["height"] - pixel_y
        
        # Bounds check
        if 0 <= pixel_x < self.map_metadata["width"] and 0 <= pixel_y < self.map_metadata["height"]:
            return (pixel_x, pixel_y)
        return None
    
    def pixel_to_world(self, pixel_x, pixel_y):
        """Convert pixel coordinates to world coordinates (meters)"""
        if self.map_metadata is None:
            return None
        
        # Flip y
        pixel_y = self.map_metadata["height"] - pixel_y
        
        world_x = pixel_x * self.map_metadata["resolution"] + self.map_metadata["origin_x"]
        world_y = pixel_y * self.map_metadata["resolution"] + self.map_metadata["origin_y"]
        
        return (world_x, world_y)
    
    def save_object_location(self, name, x, y):
        """Save a known object location"""
        self.known_objects[name] = {
            "x": x,
            "y": y,
            "timestamp": time.time()
        }
        self.get_logger().info(f'Saved object "{name}" at ({x:.2f}, {y:.2f})')

# Global instance
slam_server = None

@app.route('/slam_data', methods=['GET'])
def get_slam_data():
    """Get current SLAM data (map + pose + objects)"""
    global slam_server
    
    if slam_server is None:
        return jsonify({'error': 'SLAM server not initialized'}), 503
    
    # Check if pose is recent (within last 5 seconds)
    pose_age = time.time() - slam_server.current_pose.get("timestamp", 0)
    pose_status = "fresh" if pose_age < 5.0 else "stale"
    
    return jsonify({
        'map': slam_server.current_map_image,
        'map_metadata': slam_server.map_metadata,
        'pose': slam_server.current_pose,
        'pose_status': pose_status,
        'pose_age_seconds': pose_age,
        'objects': slam_server.known_objects,
        'timestamp': time.time()
    })

@app.route('/save_object', methods=['POST'])
def save_object():
    """Save an object location
    
    POST body:
    {
        "name": "green_block",
        "x": 2.5,
        "y": 1.3
    }
    
    Or use current robot position:
    {
        "name": "green_block",
        "use_current_position": true
    }
    """
    global slam_server
    
    if slam_server is None:
        return jsonify({'error': 'SLAM server not initialized'}), 503
    
    data = request.json
    name = data.get('name')
    
    if not name:
        return jsonify({'error': 'Missing object name'}), 400
    
    if data.get('use_current_position'):
        x = slam_server.current_pose['x']
        y = slam_server.current_pose['y']
    else:
        x = data.get('x')
        y = data.get('y')
        
        if x is None or y is None:
            return jsonify({'error': 'Missing x or y coordinates'}), 400
    
    slam_server.save_object_location(name, x, y)
    
    return jsonify({
        'success': True,
        'object': {
            'name': name,
            'x': x,
            'y': y
        }
    })

@app.route('/objects', methods=['GET'])
def get_objects():
    """Get all known object locations"""
    global slam_server
    
    if slam_server is None:
        return jsonify({'error': 'SLAM server not initialized'}), 503
    
    return jsonify({
        'objects': slam_server.known_objects,
        'count': len(slam_server.known_objects)
    })

@app.route('/status', methods=['GET'])
def status():
    """Health check"""
    global slam_server
    
    if slam_server is None:
        return jsonify({
            'ready': False,
            'error': 'SLAM server not initialized'
        }), 503
    
    pose_age = time.time() - slam_server.current_pose.get("timestamp", 0)
    has_map = slam_server.current_map_image is not None
    
    return jsonify({
        'ready': True,
        'has_map': has_map,
        'pose_status': 'fresh' if pose_age < 5.0 else 'stale',
        'pose_age_seconds': pose_age,
        'known_objects_count': len(slam_server.known_objects)
    })

def ros_spin_thread():
    """ROS2 spin in background thread"""
    global slam_server
    
    try:
        rclpy.init()
        slam_server = SlamDataServer()
        rclpy.spin(slam_server)
    except KeyboardInterrupt:
        pass
    finally:
        if slam_server is not None:
            slam_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    print("=" * 60)
    print("🗺️  SLAM Map Server Starting...")
    print("=" * 60)
    print()
    print("Endpoints:")
    print("  GET  /slam_data     - Get map + pose + objects")
    print("  POST /save_object   - Save object location")
    print("  GET  /objects       - List known objects")
    print("  GET  /status        - Health check")
    print()
    print("Starting on http://0.0.0.0:5002")
    print("=" * 60)
    print()
    
    # Start ROS2 in background thread
    ros_thread = threading.Thread(target=ros_spin_thread, daemon=True)
    ros_thread.start()
    
    # Give ROS2 time to initialize
    time.sleep(2)
    
    # Start Flask server in main thread
    app.run(host='0.0.0.0', port=5002, debug=False, threaded=True)