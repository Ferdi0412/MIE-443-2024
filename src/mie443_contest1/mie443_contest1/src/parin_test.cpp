#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include "robot.cpp"

// Constants
#define LINEAR_SPEED 0.2
#define ANGULAR_SPEED 0.5
#define WALL_DISTANCE_THRESHOLD 0.5 // Adjust according to your environment
#define MAP_COVERAGE_THRESHOLD 0.8  // Adjust according to your requirement

// Global variables
static ros::Publisher velocity_publisher;
static ros::Subscriber laser_subscriber;
static double front_distance = 0.0;
static bool obstacle_detected = false;
static bool wall_following_mode = false;

// Function declarations
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
void moveRobot(double linear_speed, double angular_speed);
void stopRobot();
void wallFollowing();
bool isObstacleInFront();

int main(int argc, char **argv) {
    // Initialize ROS node
    ros::init(argc, argv, "wall_following");
    ros::NodeHandle nh;

    // Create robot object
    Team1::Robot robot(nh, 2);

    // Setup publishers and subscribers
    velocity_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    laser_subscriber = nh.subscribe("/scan", 10, laserCallback);

    // Main loop
    while (ros::ok()) {
        if (isObstacleInFront()) {
            // If obstacle detected in front, start wall-following mode
            wall_following_mode = true;
        }

        if (wall_following_mode) {
            wallFollowing();
        } else {
            // Move forward
            moveRobot(LINEAR_SPEED, 0.0);
        }

        ros::spinOnce();
    }

    return 0;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    // Update front distance from laser scan
    front_distance = msg->ranges[msg->ranges.size() / 2];
}

void moveRobot(double linear_speed, double angular_speed) {
    // Publish velocity command to move the robot
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = linear_speed;
    vel_msg.angular.z = angular_speed;
    velocity_publisher.publish(vel_msg);
}

void stopRobot() {
    // Stop the robot's motion
    moveRobot(0.0, 0.0);
}

void wallFollowing() {
    if (front_distance > WALL_DISTANCE_THRESHOLD) {
        // If no obstacle in front, move forward while following the wall
        moveRobot(LINEAR_SPEED, -ANGULAR_SPEED); // Turn right
    } else {
        // If obstacle in front, turn left to avoid it
        moveRobot(LINEAR_SPEED, ANGULAR_SPEED); // Turn left
    }
}

bool isObstacleInFront() {
    // Check if there is an obstacle within the threshold distance in front of the robot
    return front_distance < WALL_DISTANCE_THRESHOLD;
}
