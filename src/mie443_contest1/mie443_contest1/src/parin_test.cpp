// STD imports
#include <vector>
#include <cmath>
#include <chrono>
#include <stdint.h>
#include <stdio.h>

// ROS imports
#include <ros/console.h>
#include "ros/ros.h"

#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>

// Team1::Robot import
#include "robot.cpp"

// Temp imports
#include <cmath>
#include <iostream>
#include <vector>


/**
 * ==============================
 * === FUNCTIONS DECLARATIONS ===
 * ==============================
*/

/**
 * secondsElapsed
 *
 * @returns number of seconds from program_start
*/
uint16_t secondsElapsed(void);

/**
 * printVectorFloats prints each float in a vector
*/
void printVectorFloats( const std::vector<float>& the_vector );

/**
 * =====================
 * === GLOBAL params ===
 * =====================
*/
static std::chrono::time_point<std::chrono::system_clock> program_start;

static const unsigned long long program_duration = 10;

#define SPEED_HIGH 0.2
// #define ROT_HIGH 0.2


/**
 * ============
 * === MAIN ===
 * ============
 *
 * @param argc number of params used in starting program (ignore but keep)
 * @param argv string params used when starting program  (ignore but keep)
*/


// Function to determine if a value is within a given range
bool withinRange(float value, float min, float max) {
    return (value >= min && value <= max);
}

// Function to determine if there is an obstacle in a specific range of laser scans
bool obstacleDetected(const std::vector<float>& ranges, int start_index, int end_index, float min_range, float max_range) {
    for (int i = start_index; i <= end_index; ++i) {
        if (withinRange(ranges[i], min_range, max_range)) {
            return true;
        }
    }
    return false;
}

// Wall following algorithm
void wallFollow(Team1::Robot& robot) {
    float desired_distance = 0.5; // Desired distance from the wall
    float min_follow_distance = 0.2; // Minimum distance to follow the wall
    float max_follow_distance = 0.8; // Maximum distance to follow the wall

    while (ros::ok()) {
        // Get laser scan data
        const std::vector<float>& ranges = robot.getRanges();
        int n_lasers = robot.getNLasers();

        // Check if there's an obstacle on the right
        bool obstacle_right = obstacleDetected(ranges, 0, n_lasers / 4, 0, min_follow_distance);

        // Check if there's an obstacle ahead
        bool obstacle_front = obstacleDetected(ranges, n_lasers / 4, 3 * n_lasers / 4, 0, desired_distance);

        // Check if there's an obstacle on the left
        bool obstacle_left = obstacleDetected(ranges, 3 * n_lasers / 4, n_lasers - 1, 0, min_follow_distance);

        // If there's an obstacle on the right or front, turn left
        if (obstacle_right || obstacle_front) {
            robot.rotateClockwiseBy(20,-90);
            robot.moveForwards(0.2,0.5); 
        }
        // If there's an obstacle on the left, turn right
        else if (obstacle_left) {
            robot.rotateClockwiseBy(20,90);
            robot.moveForwards(0.2,0.5);
        }
        // Otherwise, continue forward
        else {
            robot.moveForwards(0.2,0.5);
        }

        // Spin once and sleep
        robot.spinOnce();
        ros::Duration(0.1).sleep();
    }
}

int main ( int argc, char **argv ) {
    // ROS setup
    ros::init(argc, argv, "contest1");

    ROS_INFO("Starting up...\n");

    ros::NodeHandle nh;
    ros::Rate loop_rate(2);

    ROS_INFO("Creating Robot");

    // Robot object setup
    Team1::Robot robot( nh, 2);
    robot.spinOnce();
    ros::Duration(0.5).sleep(); // Sleep to ensure is initialized correctly

    // GLOBAL params setup
    program_start = std::chrono::system_clock::now();


    // ROS_INFO("Angle from 10 degrees: %.2f", robot.getAngleTo(10));
    // ROS_INFO("Angle from -90 degrees: %.2f", robot.getAngleTo(-90));
    // ROS_INFO("Angle to relative point 10, 10: %.2f", robot.getAngleToRelativePoint(10, 10));
    // ROS_INFO("Angle to abs point 10, 10: %.2f", robot.getAngleToPoint(10, 10));

    while ( ros::ok() && secondsElapsed() <= program_duration ) {
        std::cout << "Ranges:\n";
        printVectorFloats( robot.getRanges() );
        std::cout << "N Lasers: " << robot.getNLasers() << "\n";
        try {
            robot.checkBumpers();
        } catch ( BumperException& exc ) { }
        
        robot.spinOnce();
        ROS_INFO("Position: %.2f\nSpeed: %.2f\n", robot.getTheta(), robot.getVelTheta());
        // Start wall following algorithm
        wallFollow(robot);
        robot.sleepOnce();
    }

    // ROS_INFO("Time ran out!\n");
    robot.stopMotion();
    ROS_INFO("Stopping robot!\n");
}




/**
 * =================================
 * === FUNCTIONS IMPLEMENTATIONS ===
 * =================================
*/
uint16_t secondsElapsed( void ) {
    return std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - program_start).count();
}

void printVectorFloats( const std::vector<float>& the_vector ) {
    std::cout << the_vector.size();
    //for ( const float& val : the_vector )
    //    std::cout << val << "; ";
    std::cout << "\n";
}
