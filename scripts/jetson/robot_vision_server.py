#!/usr/bin/env python3
"""
Vision Server for Jetson - Complete with web viewer
"""

from flask import Flask, jsonify
import cv2
import base64
import numpy as np

app = Flask(__name__)

class CameraServer:
    def __init__(self):
        print("📷 Initializing cameras...")
        self.cap_front = cv2.VideoCapture(0)
        self.cap_wrist = cv2.VideoCapture(2)
        
        for cap in [self.cap_front, self.cap_wrist]:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        print("✅ Cameras ready")
    
    def capture_and_encode(self):
        ret1, front_img = self.cap_front.read()
        ret2, wrist_img = self.cap_wrist.read()
        
        if not ret1:
            front_img = np.zeros((480, 640, 3), dtype=np.uint8)
        if not ret2:
            wrist_img = np.zeros((480, 640, 3), dtype=np.uint8)
        
        _, front_buffer = cv2.imencode('.jpg', front_img)
        _, wrist_buffer = cv2.imencode('.jpg', wrist_img)
        
        front_b64 = base64.b64encode(front_buffer).decode('utf-8')
        wrist_b64 = base64.b64encode(wrist_buffer).decode('utf-8')
        
        return front_b64, wrist_b64

camera_server = CameraServer()

@app.route('/capture', methods=['GET'])
def capture():
    front_b64, wrist_b64 = camera_server.capture_and_encode()
    return jsonify({'front': front_b64, 'wrist': wrist_b64, 'status': 'ok'})

@app.route('/status', methods=['GET'])
def status():
    return jsonify({'ready': True})

@app.route('/view', methods=['GET'])
def view_cameras():
    """Live camera viewer"""
    html = """
<!DOCTYPE html>
<html>
<head>
    <title>Lekiwi Vision</title>
    <style>
        body { 
            background: #1a1a1a; 
            color: #0f0; 
            font-family: 'Courier New', monospace;
            margin: 0;
            padding: 20px;
        }
        .container { 
            max-width: 1400px; 
            margin: 0 auto; 
        }
        h1 { 
            text-align: center; 
            color: #0f0;
            text-shadow: 0 0 10px #0f0;
        }
        .cameras { 
            display: flex; 
            gap: 20px; 
            justify-content: center;
            flex-wrap: wrap;
        }
        .camera-box {
            border: 2px solid #0f0;
            padding: 10px;
            background: #000;
        }
        img { 
            width: 640px; 
            height: 480px; 
            display: block;
        }
        h2 { 
            color: #0f0; 
            margin: 10px 0;
            text-align: center;
        }
        .status {
            text-align: center;
            margin: 20px 0;
            font-size: 14px;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>🤖 LEKIWI ROBOT VISION SYSTEM 🤖</h1>
        <div class="status" id="status">Loading...</div>
        <div class="cameras">
            <div class="camera-box">
                <h2>📹 FRONT CAMERA</h2>
                <img id="front" src="" alt="Front Camera" />
            </div>
            <div class="camera-box">
                <h2>📹 WRIST CAMERA</h2>
                <img id="wrist" src="" alt="Wrist Camera" />
            </div>
        </div>
    </div>
    <script>
        let frameCount = 0;
        
        async function updateImages() {
            try {
                const response = await fetch('/capture');
                const data = await response.json();
                
                document.getElementById('front').src = 'data:image/jpeg;base64,' + data.front;
                document.getElementById('wrist').src = 'data:image/jpeg;base64,' + data.wrist;
                
                frameCount++;
                document.getElementById('status').textContent = 
                    '✅ LIVE - Frame: ' + frameCount + ' - Updated: ' + new Date().toLocaleTimeString();
            } catch (error) {
                document.getElementById('status').textContent = '❌ ERROR: ' + error.message;
            }
        }
        
        // Update every 500ms (2 FPS)
        setInterval(updateImages, 500);
        updateImages();
    </script>
</body>
</html>
    """
    return html

if __name__ == '__main__':
    print("🚀 Vision server starting on port 5001...")
    print("=" * 50)
    print("📹 Camera view: http://192.168.28.23:5001/view")
    print("=" * 50)
    app.run(host='0.0.0.0', port=5001, debug=False)