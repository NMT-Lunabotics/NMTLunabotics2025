#!/usr/bin/env python3
from flask import Flask, render_template

app = Flask(__name__)

# Define the external MJPEG stream URLs (adjust ports as needed)
CAMERA_STREAMS = {
    0: 'http://localhost:8080/?action=stream',
    4: 'http://localhost:8081/?action=stream'
}

@app.route('/')
def index():
    cameras = [{'id': cam_id, 'url': url} for cam_id, url in CAMERA_STREAMS.items()]
    return render_template('index.html', cameras=cameras)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
