# robot.cpp
This file creates the Robot class, under the Team1 namespace.

## Include
To include the robot.cpp file, and expose the robot class, you need to include the file.

```C++
// Assuming the class is in the same directory as the main .cpp file:
#include "robot.cpp"
```

## Initialization/Constructor

```C++
// You need a node handle to handle topic publishing/subscribing
ros::NodeHandle nh;

// You also need a rate object for blocking calls/sleep method
ros::Rate loop_rate( 2 );

// You will pass these, and a reference to the ros::spinOnce function to the robot instance
Team1::Robot robot( node_handle, loop_rate, &(ros::spinOnce) );
```

## Getting  values
The values are exposed through methods.

### SpinOnce
In order to ensure the values are up-to-date, use the spinOnceROS function to handle subscriptions and propogate changes to topics.

```C++
robot.spinOnceROS();
```

### Position
These are the odometry positions (/odom topic).

```C++
double x_position, y_position, orientation_angle;

x_position = robot.getX();
y_position = robot.getY();

oritientation_angle = robot.getTheta();
```

### Velocities
These are (mostly) the velocities from the odometry (/odom) topic.

```C++
double x_vel, y_vel, rotation_speed;

x_vel = robot.getVelX();
y_vel = robot.getVelY();

rotation_speed = robot.getVelTheta();
```

### Bumpers
These return true when a bumper is pressed, otherwise they return false.

```C++
bool left, right, center;

left = robot.getBumperLeft();
right = robot.getBumperRight();
center = robot.getBumperCenter();

if ( robot.getBumperAny() )
    cout << "At least one of the bumpers are pressed!";
```

## Controlling the robot
There are several functions intended for controlling the movement of the robot.

