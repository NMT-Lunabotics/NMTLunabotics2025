#!/usr/bin/env python3
from flask import Flask, Response, render_template
import cv2

app = Flask(__name__)

# --- Camera configuration: define per camera ID ---
CAMERA_CONFIG = {
    0: {
        'resolution': (288, 160),
        'fps': 5
    },
    4: {
        'resolution': (320, 240),
        'fps': 5
    }
}

# --- Camera stream generator ---
def gen_frames(camera_index):
    config = CAMERA_CONFIG.get(camera_index, {'resolution': (640, 480), 'fps': 10})
    width, height = config['resolution']
    fps = config['fps']

    cap = cv2.VideoCapture(camera_index)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)

    while True:
        success, frame = cap.read()
        if not success:
            break
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Convert to grayscale
        # ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 10])
        # if not ret:
        #     continue
        # frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

# --- Route to render index.html ---
@app.route('/')
def index():
    camera_ids = list(CAMERA_CONFIG.keys())
    cameras = [{'id': cam_id} for cam_id in camera_ids]
    return render_template('index.html', cameras=cameras)

# --- Route to serve video feed ---
@app.route('/video/<int:camera_id>')
def video_feed(camera_id):
    return Response(gen_frames(camera_id),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
