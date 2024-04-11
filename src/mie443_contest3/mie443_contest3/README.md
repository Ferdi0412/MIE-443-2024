# Starting all the necessary files:
There are several nodes being used here, and you need **6** terminals to run this contest package...

## Start Robot/Gazebo
```bash
cd ~/catkin_ws

### GAZEBO
bash start-gazeo.sh

### Physical Turtlebot
bash start-robot.sh
```

## Start soundplay_node
```bash
cd ~/catkin_ws

bash start-soundplay.sh
```

## Start Follower Node
```bash
cd ~/catkin_ws

bash start-follower.sh
```


### Start Camera
In order to get access to kinect images WITHOUT running the follower node:
```bash
cd ~/catkin_ws

bash start-kinect.sh
```

### Start Webcam
To subscribe to the webcam on a real linux machine:
```bash
cd ~/catkin_ws

bash start-webcam.sh -w 0
```

<br>[OR] To get access to webcam from real laptop running a VM (assuming you have access to the same local network):

```bash
### On main computer
cd .../catkin_ws/local_vidstream

pip install requirements.txt

python video_stream_server.py

### In VM
cd ~/catkin_ws

bash start-webcam.sh -w http://<main computer IP address>:5000/cam?camera_id=0
```

## Run Image Displayer Node
```bash
rosrun mie443_contest3 image_server
```

## Run Face Detection Node
```bash
rosrun mie443_contest3 face_server
```

## Run Contest3 Node
```bash
rosrun mie443_contest3 contest3
```
