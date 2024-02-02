# MIE-443-2024
Project for Thursday Team 1 for MIE443 in Winter 2023/2024. This assumes you are using Ubuntu 16.04, and ROS-Kinetic, with course-provided dependencies installed.

## Setup
Treat this as the ***catkin_ws*** directory from class. When first installed, ensure that the root directory is set up, and run catkin_make.
```shell
## == DIRECTORY SETUP ==
## If the repository folder not created, start from here...
cd ~
mkdir catkin_ws
cd catkin_ws
REPOSITORY_ROOT="~/catkin_ws" # NOTE that anywhere you see $REPOSITORY_ROOT, it can be replaced with the actual directory of your clone of this repository...

## == SSH KEYGEN ==
## If you need to setup git, you need an SSH certificate, follow the following to create one:
ssh-keygen -t ed25519
## Follow setup steps
## Make note of where file is stored
## Typically under directory /home/<user>/.ssh/id_ed25519
cd ~/.ssh          # If in default directory
cat id_ed25519.pub # Print the public certificate key
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
catkin_init_workspace

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

# Contest 1
For contest 1, we will start off using a wall-following strategy. If time permits, we will look into frontier-search to find unexplored areas from the **gmapping** node.

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

## Wall-following
Add details here...
