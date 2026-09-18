import cv2
from flask import Flask, Response

def main():

    app = Flask(__name__)
    
    cam = cv2.VideoCapture(0)  # Initialize the camera (0 is usually the default camera)

    def generate_frames():
        while True:
            ret, frame = cam.read()  # Read a frame from the camera
            if not ret:
                print("Failed to grab frame")
                break

            # Encode the frame in JPEG format
            ret, buffer = cv2.imencode('.jpg', frame)
            if not ret:
                print("Failed to encode frame")
                break

            # Convert the buffer to bytes and yield it as a response
            frame_bytes = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

    @app.route('/video_feed')
    def video_feed():
        return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

    app.run(host='0.0.0.0', port=5000)

