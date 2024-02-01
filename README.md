# MIE-443-2024
Project for Thursday Team 1 for MIE443 in Winter 2023/2024

## Setup
Treat this as the ***catkin_ws*** directory from class. When first installed, ensure that the root directory is set up, and run catkin_make.
```shell
REPOSITORY_ROOT="~/catkin_ws"
cd $REPOSITORY_ROOT/
catkin_make
```

To be able to launch the project, you may have to run the following:
```shell
cd $REPOSITORY_ROOT/
source devel/setup.sh
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
