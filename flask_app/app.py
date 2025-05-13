#!/usr/bin/env python3
from flask import Flask, Response, render_template
import cv2

app = Flask(__name__)

# --- Camera stream generator ---
def gen_frames(camera_index):
    cap = cv2.VideoCapture(camera_index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    cap.set(cv2.CAP_PROP_FPS, 10)

    # Define the codec and create a VideoWriter object for H.264
    # Removed unused fourcc variable

    while True:
        success, frame = cap.read()
        if not success:
            break
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Convert to grayscale

        # Encode the frame using JPEG
        ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 15])
        if not ret:
            continue
        frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

# --- Route to render index.html ---
@app.route('/')
def index():
    # List of cameras: /dev/video0
    camera_ids = [0, 4]
    cameras = [{'id': cam_id} for cam_id in camera_ids]
    return render_template('index.html', cameras=cameras)

# --- Route to serve video feed ---
@app.route('/video/<int:camera_id>')
def video_feed(camera_id):
    return Response(gen_frames(camera_id),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, threaded=True)
