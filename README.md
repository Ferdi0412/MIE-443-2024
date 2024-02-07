# MIE-443-2024
Project for Thursday Team 1 for MIE443 in Winter 2023/2024. This assumes you are using Ubuntu 16.04, and ROS-Kinetic, with course-provided dependencies installed.

## Deadlines
| Date         | Item |
| :--:         | :--: |
| Feb 6 (tues) | Individual navigation/exploration |
| Feb 7 (wed)  | Extra lab                         |

## Initial Navigation Strategies
- Wall-following
- Open-spaces
- Biased random-walk

# Contest 1
For [contest 1](src/mie443_contest1/mie443_contest1/src/README.md), we will start off evaluating wall-following, open-space and biased random-walk navigation strategies.
~~If time permits, we will look into frontier-search to find unexplored areas from the **gmapping** node.~~

## Startup
If you want to simulate, us GAZEBO, otherwise use ROBOT to connect to the physical turtlebot.
```shell
# Build            -> Before running
cd ~/catkin_ws
catkin_make
# GAZEBO           -> Terminal 1
roslaunch mie443_contest1 turtlebot_world.launch world:=1
# ROBOT            -> Terminal 1
...
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
This is a class for controlling the robot movements, and will wrap the topic calls, for simplicity sake.

```c++
ros::NodeHandle nh;
Team1Robot robot(nh);

// Move forwards, and check stuff...
robot.moveForwards( );

// Use a do-while instead of while loop, such that robot.cycleROS() is run at least once, so that .isMoving updates
do {
    robot.cycleROS();
    // Check states of everything
} while ( robot.isMoving() );

// Rotate by 45 degrees, and wait until it stops moving
robot.rotate( 45 );

robot.waitMovement();
```

# Workspace Setup
Treat this as the ***catkin_ws*** directory from class. When first installed, ensure that the root directory is set up, and run catkin_make.
```shell
## == DEPENDENCY SETUP ==
## First install all required software through the script provided by the course
## In case of any errors, run the following lines
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F42ED6FBAB17C654
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key EB3E94ADBE1229CF
## You may also need to run:
sudo apt update
sudo apt-get update
## For more details, the following link may help: https://askubuntu.com/questions/140246/how-do-i-resolve-unmet-dependencies-after-adding-a-ppa

## == GAZEBO FIX ==
## In case of issues running Gazebo with the setup/dependencies provided in the course, run the following (taken from course PIAZZA post note@19)
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sduo apt update
sudo apt upgrade


## == DIRECTORY SETUP ==
## If the repository folder not created, start from here...
cd ~
mkdir catkin_ws
cd catkin_ws
REPOSITORY_ROOT="~/catkin_ws" # NOTE that anywhere you see $REPOSITORY_ROOT, it can be replaced with the actual directory of your clone of this repository...

## == DIRECTORY CLEANUP ==
## If you need to clone the git repository, your repository directory must be empty
## If there are files here, you can use the following:
sudo rm -rf $REPOSITORY_ROOT # -rf means RECURSIVE, FORCED -> all sub-files and directories are deleted
mkdir $REPOSITORY_ROOT       # Re-create the directory without any files

## == SSH KEYGEN ==
## If you need to setup git, you need an SSH certificate, follow the following to create one:
ssh-keygen -t ed25519
## Follow setup steps
## Make note of where file is stored
## Typically under directory /home/<user>/.ssh/id_ed25519
cat ~/.ssh/id_ed25519.pub # Print the public certificate key
## Add the SSH key to your github "SSH and GPG keys" under settings

## == GIT SETUP ==
## If you have not setup git, start from here...
cd $REPOSITORY_ROOT
## Check that git is installed
git --version                                         # Should print something like "git version x.x.x"
## Configure git settings
git config --global user.name "My Name"
git config --global user.email "my.name@gmail.com"
## Check config is correctly set
git config --list
git clone git@github.com:Ferdi0412/MIE-443-2024.git . # Clone the repository to current directory (should be $REPOSITORY_ROOT)
cd src

## == GIT OPERATIONS ==
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

## == CATKIN SETUP ==
## Before running catkin_make, ensure the source is initialized
cd $REPOSITORY_ROOT/src
catkin_init_workspace

## == CATKIN BUILD ==
## To build the project, run the following:
cd $REPOSITORY_ROOT
catkin_make

## == SOURCE PROJECT ==
## To be able to launch project programs, you may have to do the following:
cd $REPOSITORY_ROOT
source devel/setup.sh

## == RESET BUILD ==
## In case of strange errors, this may sometimes fix them:
cd $REPOSITORY_ROOT
sudo rm -rf build     # Remove all build files created by catkin_make
sudo rm -rf devel     # Remove all devel files created by catkin_make
catkin_make           # Re-build projecct
source devel/setup.sh # re-source
```
