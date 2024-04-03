# MIE-443-2024
Project for Thursday Team 1 for MIE443 in Winter 2023/2024. This assumes you are using Ubuntu 16.04, and ROS-Kinetic, with course-provided dependencies installed.

# Contest 3
This is split between [contest 3](src/mie443_contest3/mie443_contest3) and [turtlebot follower](src/turtlebot_follower/turtlebot_follower).

## Startup
```shell
## For all terminals:
cd ~/catkin_ws # Make sure all commands are run from root of this repository

##################
### SIMULATION ###
##################
## Terminal 0 [OPTIONAL] - Run on local computer to display webcam over VirtualBox VM
pip install -r local_vidstream/requirements.txt
python local_vidstream/video_stream_server.py

## Terminal 1
bash start-gazebo.sh # Add "-h" for help with options

## Terminal 2
bash start-soundplay.sh

## Terminal 3
rosrun mie443_contest3 image_server

## Terminal 4 [OPTIONAL] - Run to have access to webcam
bash start-webcam.sh <source>
# For Linux machines - Use "0" instead of <source>
# For VM with Teminal 0 option - Use "http://localhost:5000/cam?camera_id=0" instead of <source>

## Terminal 5
rosrun mie443_contest3 priv_test # Run this to run the test node - swap 'priv_test' with 'contest3' to run actual contest3 file

###############
### CONTEST ###
###############
## Terminal 1
bash start-robot.sh

## Terminal 2
bash start-soundplay.sh

## Terminal 3
bash start-follower.sh

## Terminal 4
rosrun mie443_contest3 image_server

## Terminal 5
rosrun mie443_contest3 contest3
```

# Contest 2
For [contest 2](src/mie443_contest2/mie443_contest2).

## Startup
```shell
## For all terminals:
cd ~/catkin_ws # Make sure all commands are run from root of this repository

##################
### SIMULATION ###
##################
## Terminal 1
bash start-gazebo.sh # Add "-h" for help with options

## Terminal 2
bash start-amcl.sh # Add "-h" for help with options

## Terminal 3 [OPTIONAL]
bash start-rviz.sh

## Terminal 4 [OPTIONAL]
rosrun mie443_contest2 webcam_publisher 0 # Run this if you need the webcam_publisher node

## Terminal 5
rosrun mie443_contest2 priv_test # Run this to run the test node

###############
### CONTEST ###
###############
## Terminal 1
bash start-robot.sh

## Terminal 2
bash start-amcl.sh --map_file <path_to_map>.yaml # Replace <path_to_map> with the path to the desired map file

## Terminal 3
bash start-rviz.sh

## Terminal 4 [TBD if this is actually needed...]
rosrun mie443_contest2 webcam_publisher 0

## Terminal 5
rosrun mie443_contest2 mie443_contest2
```

# Contest 1
For [contest 1](src/mie443_contest1/mie443_contest1/src/README.md).

## Startup
```shell
## For all terminals:
cd ~/catkin_ws

##################
### SIMULATION ###
##################
## Terminal 1
bash start-gazebo.sh --world 1 --contest 1

## Terminal 2
bash start-gmapping.sh

## Terminal 3 [OPTIONAL]
bash start-rviz.sh

## Terminal 4
rosrun mie443_contest1 contest1

###############
### CONTEST ###
###############
## Terminal 1
bash start-robot.sh

## Terminal 2
bash start-gmapping.sh

## Termianl 3
bash start-rviz.sh

## Terminal 4
rosrun mie443_contest1 contest1

## Terminal 5 [AFTER contest1 ends]
rosrun map_server map_saver -f <file_name>
```

# ROS Commands
To move the robot in gazebo, you can use the following:
```shell
roslaunch turtlebot_teleop keyboard_teleop.launch
```

To check what topic are currently *active*:
```shell
rostopic list
```

To listen to a topic:
```shell
rostopic echo /{topic}
```

To print info about a topic:
```shell
rostopic info /{topic}
```

To check what nodes are currently running:
```shell
rosnode list
```

To print info about a node:
```shell
rostopic info /{node_name}
```

# Workspace Setup
Treat this as the ***catkin_ws*** directory from class. When first installed, ensure that the root directory is set up, and run catkin_make. This setup requires Ubuntu 16.04 Xenial Xerus, and the setup steps installs ROS Kinetic.

## Cloning repository
To clone the repository, follow the following steps:
```shell
##############
### 1. SSH ###
##############
# SSH is required to connect to GitHub
ssh-keygen -t ed25519     # Follow on-screen prompts, using default path, to generate SSH keys
cat ~/.ssh/id_ed25519.pub # Print to terminal the public SSH key, this should be copied to your GitHub settings

##############
### 2. GIT ###
##############
# You will need to ensure the `git` command is available
git --version                                                   # Check that `git` is available
git config --global user.name "Insert your name here"
git config --global user.email "insert.your.email@here"
git config --list                                               # Ensure the config is correct
mkdir ~/catkin_ws                                               # Make a folder to clone the repository to
git clone git@github.com:Ferdi0412/MIE-443-2024.git ~/catkin_ws # Clone to your new folder
cd ~/catkin_ws                                                  # Go to your new clone of this repository
```

## Setup/requirements
You will need to have used the course-provided setup file to install dependencies.
```shell
##################################
### 3. COURSE-DEPENDENCY FIXES ###
##################################
# Run course dependencies
bash turtlebot_script.sh

# If the course-provided script does not fully work, run these 4 lines, then re-try
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F42ED6FBAB17C654
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key EB3E94ADBE1229CF
sudo apt update
sudo apt-get update

###########################
### 4. REPOSITORY SETUP ###
###########################
# After cloning the repository for the first time, run the following shell file
cd ~/catkin_ws
bash setup.sh
```

## Gazebo fix
When running gazebo, there may be some issues. Try the following to fix it.
```shell
#####################
### 5. GAZEBO FIX ###
#####################
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt update
sudo apt upgrade
```

## Catkin Make
To build the project, run the following command.
```shell
######################
### 6. CATKIN_MAKE ###
######################
cd ~/catkin_ws
catkin_make
```

## Reset build
Upon seemingly un-fixable issues when compiling, you may try the following.
```shell
######################
### 7. RESET BUILD ###
######################
cd ~/catkin_make
bash reset-build.sh
```

## Git operations
To work with git, get familiar with the following operations. **NOTE:** That you should always ensure all builds correctly with `catkin_make` before commiting/pushing something.
```shell
#########################
### 8. GIT OPERATIONS ###
#########################
## To update your local clone of the repository:
git pull

## To check what changes you have made locally
git status

## To compare your version with GitHub version (eg. if you are unsure of other's changes)
git fetch
git status

## To commit file src/README.md to local revision history
git add src/README.md
git commit -m "Short Message"                    # For short message only
git commit -m "Short Message" -m  "Long Message" # For short AND long message

## To push local revision history to remote
git push
```
