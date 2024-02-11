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

// ========= AUXILLIARY =========
/**
 * secondsElapsed
 *
 * @returns number of seconds from program_start
*/
uint16_t secondsElapsed(void);

static std::chrono::time_point<std::chrono::system_clock> program_start;

static const unsigned long long program_duration = 480;


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
    Team1::Robot robot( nh, 2);

    // Sleep to ensure is initialized correctly
    ros::Duration(0.5).sleep();
    robot.spinOnce();

    // Start timer
    program_start = std::chrono::system_clock::now();

    // Add in to wait on laser_ranges
    robot.waitOnLaserRanges();

    // === MAIN ===
    // loop until program_duration [seconds] is reached
    while ( ros::ok() && secondsElapsed() <= program_duration ) {
        // Main stuff here...

        // Track odometry for previous positions

        /**
         * === STARTUP ===
         * 1. Check for wall   -> if true, wallFollow move in that direction until wall
         * 2. Check for corner -> if true, turn 45 degrees and next cycle
         * 3. scanForArea      -> move in that direction until wall
         *                     -> if scanForArea issue -> randomMotion -> move in that direction until wall
        */

        /**
         * === NORMAL CYCLE ===
         * SAME AS STARTUP
        */

        /**
         * === BUMPED TOO OFTEN ===
         * - Store vector/fixed sized queue of timestamps
         * -> if first element too recent, run this...
         *
         * 3 bumps -> within 10 seconds (something on ground, or stuck in corner)
         *
         * 1. turn 180
         * 2. small random rotation -> +/- 45 degrees???
        */

        /**
         * === BUMP INTO OBJECT BUT NO SCAN ===
         * - account for object on ground
        */

        robot.sleepOnce();
    }


    // === PROGRAM END ===
    robot.stopMotion();
    ROS_INFO("ENDED...\n");
}



/**
 * =================================
 * === FUNCTIONS IMPLEMENTATIONS ===
 * =================================
*/
// ========= EDIT THE FOLLOWING DEFINITION BELOW =========
uint16_t secondsElapsed( void ) {
    return std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - program_start).count();
}
