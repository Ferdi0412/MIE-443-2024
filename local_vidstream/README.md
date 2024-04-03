# Local Video Stream
Because we are using the Oracle VirtualBox VM, accessing webcams in the VM is difficult. As such, here is a python code to serve a video stream to access within the VM.

## Important Note
If any given connection closes, the video stream will stop until the next connection is established....

## Install requirements
```bash
pip install -r requirements.txt
```

## Run code
```bash
python video_stream_server.py
```
