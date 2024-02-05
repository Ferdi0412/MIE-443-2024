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

// You must pass a node_handle and set the frequency (here 50 Hz)
Team1::Robot robot( node_handle, 50 );
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
// Odometry/global settings
double x_vel = robot.getVelX();
double y_vel = robot.getVelY();

double rotation_speed = robot.getVelTheta();

// Current robot actual velocities (from robot state)
double fwd_vel   = robot.getVelFwdAct();
double clock_vel = robot.getVelClockAct();

// Current code velocities (set-points from this object)
double fwd_vel   = robot.getVelFwdSet();
double clock_vel = robot.getVelClockSet();
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

### Lasers

```C++
TBD ...
```

## Control motion
There are several functions intended for controlling the movement of the robot.

### Linear
```C++
// To move forwards at 0.2 [m/s] for 2 seconds
robot.jogForwards(0.2);
robot.sleepFor(2);
robot.stopMotion();

// To move forwards only if the robot is not currently pressing against a wall
try {
    robot.jogForwardsSafe(0.2);
} catch {
    cout << "Cannot move forwards, as the bumper is against a wall!";
}

// To move forwards for a set distance of 2 [m] at a speed of 0.2 [m/s]
try {
    robot.moveForwards( 0.2, 2 );
} catch {
    cout << "A wall was encountered before 2 meters was covered.";
}

/* To move forwards for 2 [m] or until a wall is encountered...

1. Check if robot bumper is triggered
2. Get starting position
3. Start forwards travel
4. Implement interrim logic
5. Stop motion once 2 meters has been reached
*/

double target_distance = 2;

if ( !robot.getBumperAny() ) {
    // Start position
    double x_start, y_start;
    x_start = robot.getX();
    y_start = robot.getY();

    // Forwards travel
    robot.jogForwards(0.2);

    while ( (spinOnce(), robot.distanceToPoint(x_start, y_start) < target_distance) ) {
        if ( robot.getBumperAny() ) break; // Break from while loop if any bumper is pressed

        // Add additional logic...

        robot.sleepOnce(); // Sleep for rest of cycle...
    }

    // Once distance is travelled, stop motion
    stopMotion();

}
```

### Rotation
```C++
// To rotate at 0.2 [rad/s] for 2 seconds
robot.jogClockwise(0.2);
robot.sleepFor(2);
robot.stopMotion();

// To rotate only if the front of robot is not currently against a wall
try {
    robot.jogClockwiseSafe(0.2);
} catch {
    cout << "The front of the robot is touching a wall in the direction of rotation!";
}

// TODO: Add a distance of rotation...
```

## Potentially interesting
### Nodes
1. /bumper2pointcloud

### Topics
1. /camera/depth/points
2. /camera/depth/image_raw
