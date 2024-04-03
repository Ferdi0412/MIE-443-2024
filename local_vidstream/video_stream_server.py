from typing import Generator
from flask import Flask, Response, request, abort
import cv2

###################
### LIVE STREAM ###
###################

def open_camera( camera_number: int ) -> cv2.VideoCapture | None:
    """Returns an open VideoCapture if possible, otherwise None."""
    camera = cv2.VideoCapture( camera_number )
    if not camera.isOpened():
        camera.release()
        return None
    return camera



def generate_stream_frames( camera: cv2.VideoCapture ) -> Generator[bytes, None, None]:
    """Yields frames of image."""
    while True:
        success, frame = camera.read()
        if not success:
            break
        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()
        yield frame
    camera.release()



def browser_format_frames( frame_generator: Generator[bytes, None, None] ) -> Generator[bytes, None, None]:
    """Re-formats an image frame for a web-browser."""
    for frame in frame_generator:
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')



########################
### SERVER RESOURCES ###
########################
def get_index( camera_number: int ) -> str:
    """Returns "homepage" to display video..."""
    camera_url = f"/cam?camera_id={camera_number}"
    return f"""
<html>
<head>
    <title>Video Stream</title>
</head>
<body>
    <h1>Video Stream</h1>
    <a href="{camera_url}">Camera {camera_number}</a>
    <img src="{camera_url}" width="640" height="480">
</body>
</html>
"""



########################
### SERVER ENDPOINTS ###
########################
app = Flask(__name__)


@app.route('/')
def home_page():
    camera_number = request.args.get('camera_id', default=0, type=int)
    return get_index( camera_number )



@app.route('/cam')
def serve_camera():
    camera_number = request.args.get('camera_id', default=0, type=int)
    user_agent    = request.headers.get('User-Agent', '')
    camera        = open_camera( camera_number )
    if camera is None:
        abort(500, description=f"Camera {camera_number} could not be opened!")
    # if 'Mozilla' in user_agent: # User is a browser
    return Response(browser_format_frames(generate_stream_frames( camera )),
                    mimetype='multipart/x-mixed-replace; boundary=frame')



####################
### START SERVER ###
####################
if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True, threaded=True)
