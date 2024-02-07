#include <vector>
#include <cmath>
#include <chrono>
#include <stdint.h>
#include <stdio.h>

#include <ros/console.h>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

#include "robot.cpp"

// Define constants
#define LINEAR_SPEED 0.2
#define ANGULAR_SPEED 0.5
#define WALL_DISTANCE_THRESHOLD 1.0 // Adjust according to your environment
#define MAP_COVERAGE_THRESHOLD 0.8  // Adjust according to your requirement

// Global variables
static std::chrono::time_point<std::chrono::system_clock> program_start;
static const unsigned long long program_duration = 10; // Adjust as needed

// Function declarations
void moveAndScan_example(Team1::Robot robot, double distance);
void wallFollowing(Team1::Robot &robot);
void stopRobot(Team1::Robot &robot);
uint16_t secondsElapsed(void);
void exitIfTimeRunOut(void);
void exitIfTimeRunOut(unsigned int exit_code);

int main(int argc, char **argv) {
    // Setup ROS node
    ROS_INFO("Setting up...");

    ros::init(argc, argv, "contest1");
    ros::NodeHandle nh;
    ros::Rate loop_rate(2);

    Team1::Robot robot(nh, 2);
    ros::Duration(0.5).sleep(); // Ensure initialization
    robot.spinOnce();
    program_start = std::chrono::system_clock::now();

    // Main loop
    while (ros::ok() && secondsElapsed() <= program_duration) {
        wallFollowing(robot);
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Program end
    stopRobot(robot);
    ROS_INFO("Program ended.");
}

void wallFollowing(Team1::Robot &robot) {
    double front_distance = robot.getRanges()[robot.getNLasers() / 2]; // Front distance from laser scan

    // Check if there's an obstacle in front
    if (front_distance < WALL_DISTANCE_THRESHOLD) {
        // Turn right to follow the wall
        robot.setMotion(LINEAR_SPEED, -ANGULAR_SPEED);
    } else {
        // Move forward while following the wall
        robot.setMotion(LINEAR_SPEED, 0);
    }
}

void stopRobot(Team1::Robot &robot) {
    // Stop the robot's motion
    robot.stopMotion();
}

uint16_t secondsElapsed(void) {
    return std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - program_start).count();
}

void exitIfTimeRunOut(void) {
    if (secondsElapsed() > program_duration)
        exit(0);
}

void exitIfTimeRunOut(unsigned int exit_code) {
    if (secondsElapsed() > program_duration)
        exit(exit_code);
}
