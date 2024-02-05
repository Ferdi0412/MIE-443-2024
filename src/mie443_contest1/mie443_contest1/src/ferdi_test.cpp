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
int main ( int argc, char **argv ) {
    // ROS setup
    ros::init(argc, argv, "contest1");

    ROS_INFO("fmod(-180, 360): %.2f\n", fmod(-180, 360));

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

    robot.rotateClockwiseTo(-10, -50);

    // robot.moveForwards(0.2, 0.5);

    while ( ros::ok() && secondsElapsed() <= program_duration ) {
        std::cout << "Ranges:\n"
        printVectorFloats( robot.getRanges() );
        std::cout << "Intensities:\n";
        printVectorFloats( robot.getIntensities() );
        std::cout << "N Lasers: " << robot.getNLasers() << "\n";
        robot.checkBumpers();
        robot.spinOnce();
        ROS_INFO("Position: %.2f\nSpeed: %.2f\n", robot.getTheta(), robot.getVelTheta());
        robot.sleepOnce();
    }

    ROS_INFO("Time ran out!\n");
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
    for ( const float& val : the_vector )
        std::cout << val << "; ";
    std::cout << "\n";
}
