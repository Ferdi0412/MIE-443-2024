// STD imports
#include <vector>
#include <cmath>
#include <chrono>

// ROS imports
#include <ros/console.h>
#include "ros/ros.h"

#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvents.h>
#include <sensor_msgs/LaserScan.h>

// Team1::Robot import
#include "robot.cpp"



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
 * =====================
 * === GLOBAL params ===
 * =====================
*/
static std::chrono::time_point<std::chrono::system_clock> program_start;

static const uin16_t program_duration = 480;



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

    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    // Robot object setup
    Team1::Robot robot( nh, loop_rate, &(ros::spinOnce) );

    // GLOBAL params setup
    program_start = std::chrono::system_clock::now();

    while ( ros::ok() && secondsElapsed() <= program_duration ) {
        robot.spinOnce();

        ROS_INFO("Position: %d\n", robot.getX());
    }
}



/**
 * =================================
 * === FUNCTIONS IMPLEMENTATIONS ===
 * =================================
*/
uint16_t secondsElapsed( void ) {
    return std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - program_start).count();
}
