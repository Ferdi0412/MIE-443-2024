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
 * moveAndScan
 *
 * An example function for how to move and scan the robot.
*/
void moveAndScan( Team1::Robot robot );

/**
 * cleanup
 *
 * Should run after everything else
*/
void cleanup( void );

/**
 * =====================
 * === GLOBAL params ===
 * =====================
*/
static std::chrono::time_point<std::chrono::system_clock> program_start;

static const unsigned long long program_duration = 10;


/**
 * ============
 * === MAIN ===
 * ============
 *
 * @param argc number of params used in starting program (ignore but keep)
 * @param argv string params used when starting program  (ignore but keep)
*/
int main ( int argc, char **argv ) {
    // === SETUP ===
    ROS_INFO("SETUP...\n");

    ros::init(argc, argv, "contest1");

    ros::NodeHandle nh;
    ros::Rate loop_rate(2);
    Team1::Robot robot( nh, 2);

    ros::Duration(0.5).sleep(); // Sleep to ensure is initialized correctly
    robot.spinOnce();

    program_start = std::chrono::system_clock::now();


    // === MAIN ===



    // === PROGRAM END ===
    robot.stopMotion();
    ROS_INFO("ENDED...\n");
}



/**
 * =================================
 * === FUNCTIONS IMPLEMENTATIONS ===
 * =================================
*/
uint16_t secondsElapsed( void ) {
    return std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - program_start).count();
}

void moveAndScan( Team1::Robot robot ) {
    double start_x = robot.getX(), start_y = robot.getY();
    robot.jogForwardsSafe( 0.2 ); // Start moving at 0.2 [m/s]

    // Check distance from robot to starting position, until 2 [m] has been travelled
    while ( robot.distanceToPoint( start_x, start_y ) < 2 ) {
        // Update values in robot from subscriptions
        robot.spinOnce();

        // If any bumper has been triggered, robot has collided with a wall...
        // Stop the motion of the robot, and return
        if ( robot.getBumperAny() ) {
            robot.stopMotion();
            return;
        }

        // Do something with the scan data
        std::vector<float> scan_distances = robot.getRanges();
        // ...
    }

    // Once enough distance travelled...
    robot.stopMotion();
}

void cleanup( void ) {

}

