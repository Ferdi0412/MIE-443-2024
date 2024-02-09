// STD imports
#include <vector>
#include <cmath>
#include <chrono>
#include <stdint.h>
#include <stdio.h>

// ROS imports
#include <ros/console.h>
#include "ros/ros.h"

// Team1::Robot import
#include "robot.cpp"

int main(int argc, char **argv) {
    // Initialize ROS node
    ros::init(argc, argv, "exploration_node");
    ros::NodeHandle nh;

    // Create a Robot object
    Team1::Robot robot(nh, 10); // Spin frequency of 10 Hz

    try {
        // Start exploration
        while (ros::ok()) {
            // Move forwards
            robot.jogForwardsSafe(0.2); // Adjust speed as needed

            // Check for obstacles
            if (robot.getVelFwdAct() > 0 && robot.getNLasers() > 0) {
                // Get laser scan data
                const std::vector<float>& ranges = robot.getRanges();

                // Check for obstacles in front
                bool obstacle_detected = false;
                for (float range : ranges) {
                    if (range < 1.0) { // Adjust threshold as needed
                        obstacle_detected = true;
                        break;
                    }
                }
// fuck

                // If obstacle detected, rotate to avoid it
                if (obstacle_detected) {
                    robot.rotateClockwiseBy(30, 90); // Rotate 90 degrees clockwise
                }
            }

            // Sleep for a short duration before next iteration
            robot.sleepFor(0.1); // Adjust as needed
            // Team1::checkBumpers();
        }
    } catch (const BumperException& e) {
        // Handle bumper collision
        ROS_WARN("Bumper collision detected. Stopping exploration.");
    }

    // Stop motion before exiting
    robot.stopMotion();

    return 0;
}

