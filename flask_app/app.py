#!/usr/bin/env python3
from flask import Flask, Response, render_template
import cv2
import sys


num_arg = len(sys.argv)

rotate_array = [[180],[0],[180]]  # Initialize array with 3 0 values
lines_draw = [[0.25],[0],[0]]  # Initialize array with 3 0 values

if num_arg > 1:
    # First argument is an array of camera indices
    rotate_array = [int(x) for x in sys.argv[1].strip('[]').split(',')]
    
    if num_arg > 2:
        lines_draw = [[float(x)] for x in sys.argv[2].strip('[]').split(',')]
        
               
app = Flask(__name__)

CAMERA_CONFIG = {
    0: {
        'resolution': (640, 320),
        'fps': 10,
        'flip': False,
        'rotate': rotate_array[0],  # 0, 90, 180, or 270
        'lines': lines_draw[0] # Percent of height for reference line
    },
    4: {
        'resolution': (640, 320),
        'fps': 10,
        'flip': False,
        'rotate': rotate_array[1],  # 0, 90, 180, or 270
        'lines': lines_draw[1] # Percent of height for reference line
    },
    
    8: {
        'resolution': (640, 320),
        'fps': 10,
        'flip': False,
        'rotate': rotate_array[2],  # 0, 90, 180, or 270
        'lines': lines_draw[2] # Percent of height for reference line
    }
}

def gen_frames(camera_index):
    config = CAMERA_CONFIG.get(camera_index, {'resolution': (640, 480), 'fps': 10, 'flip': False, 'rotate': 0})
    width, height = config['resolution']
    fps = config['fps']
    flip = config.get('flip', False)
    rotate = config.get('rotate', 0)

    cap = cv2.VideoCapture(camera_index)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)

    while True:
        success, frame = cap.read()
        if not success:
            break

        if flip:
            frame = cv2.flip(frame, 1)  # Horizontal flip

        if rotate == 90:
            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        elif rotate == 180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        elif rotate == 270:
            frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

        # Draw reference lines
        for line in config['lines']:
            y = int(frame.shape[0] * (1 - line))
            cv2.line(frame, (0, y), (frame.shape[1], y), (255, 0, 0), 2)

        # Convert to grayscale
        # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
        if not ret:
            continue
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

@app.route('/')
def index():
    camera_ids = list(CAMERA_CONFIG.keys())
    cameras = [{'id': cam_id} for cam_id in camera_ids]
    return render_template('index.html', cameras=cameras)

@app.route('/video/<int:camera_id>')
def video_feed(camera_id):
    return Response(gen_frames(camera_id),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
