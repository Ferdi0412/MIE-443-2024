# MIE-443-2024
Project for Thursday Team 1 for MIE443 in Winter 2023/2024. This assumes you are using Ubuntu 16.04, and ROS-Kinetic, with course-provided dependencies installed.

# Contest 2
For [contest 2](src/mie443_contest2/mie443_contest2).

## Startup
To startup, you can use GAZEBO to simulate robot motion...

# Contest 1
For [contest 1](src/mie443_contest1/mie443_contest1/src/README.md).

## Startup
If you want to simulate, us GAZEBO, otherwise use ROBOT to connect to the physical turtlebot.
```shell
# Build            -> Before running
cd ~/catkin_ws
catkin_make
# GAZEBO           -> Terminal 1
roslaunch mie443_contest1 turtlebot_world.launch world:=1
# ROBOT            -> Terminal 1
roslaunch turtlebot_bringup minimal.launch
# Script           -> Terminal 2
rosrun mie443_contest1 contest1
# GMAPPING         -> Termianl 3
roslaunch mie443_contest1 gmapping.launch
# RVIZ [OPTIONAL ] -> Terminal 4
roslaunch turtlebot_rviz_launchers view_navigation.launch
# SAVE MAP         -> After running
rosrun map_server map_saver -f your_map_name
```

## Team1Robot
This is a class for controlling the robot movements, and will wrap the topic calls, for simplicity sake. For details, checkout the [contest 1 README](src/mie443_contest1/mie443_contest1/src/README.md).

## ROS Commands
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
Treat this as the ***catkin_ws*** directory from class. When first installed, ensure that the root directory is set up, and run catkin_make.

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
