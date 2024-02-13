// STD imports
#include <vector>
#include <cmath>
#include <chrono>
#include <stdint.h>
#include <stdio.h>

#include <queue>
#include <boost/circular_buffer.hpp>

// ROS imports
#include <ros/console.h>
#include "ros/ros.h"

// Team1::Robot import
#include "robot.cpp"

// Higher level movements
#include "include/assumed_functions.hpp"

// Ferdi's functions
#include "include/ferd_functions.cpp"

// Parin's functions
#include "include/parin_functions.cpp"

// Emma's functions
#include "include/emma_functions.cpp"

// Ajeya's Functions
#include "include/ajeya_functions.cpp"

// Linear approximation dependency
#include "include/lin_approx.cpp"

// ========= AUXILLIARY =========
#define WALL_DISTANCE 0.2

typedef unsigned long long time_elapsed_t;

time_elapsed_t secondsElapsed(void);

time_elapsed_t getCurrentTime( void );

void storeBumperTimestamp( void ); // Stores the current timestamp in the bumper events queue

void initializeBumperQueue( void );

boost::circular_buffer<time_elapsed_t> bumper_queue(3); // 3 Bumper events...

time_elapsed_t timeSinceFirstBumper( void );

static std::chrono::time_point<std::chrono::system_clock> program_start;

static time_elapsed_t program_duration = 480;

int move_res = 0;

wallDirectionEnum direction = any;

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

    initializeBumperQueue();

    ros::init(argc, argv, "contest1");

    ros::NodeHandle nh;
    Team1::Robot robot( nh, 2);

    // Sleep to ensure is initialized correctly
    ros::Duration(0.5).sleep();
    robot.spinOnce();

    // Start timer
    program_start = std::chrono::system_clock::now();

    // Add in to wait on laser_ranges
    robot.waitOnLaserRanges(); // FERDI -> To implement

    // === MAIN ===
    // loop until program_duration [seconds] is reached
    while ( ros::ok() && secondsElapsed() <= program_duration ) {
        ROS_INFO("=== === Wall ANGLE: %.2f\n", getWallAngleFromLaserScan(robot));

        // robot.sleepOnce();
        robot.sleepFor(1);
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
time_elapsed_t secondsElapsed( void ) {
    return std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - program_start).count();
}


time_elapsed_t getCurrentTime( void ) {
    return secondsElapsed();
}


void storeBumperTimestamp( void ) {
    bumper_queue.push_back( getCurrentTime() );
}


void initializeBumperQueue( void ) {
    while ( bumper_queue.size() > 0 )
        bumper_queue.pop_front();
}

time_elapsed_t timeSinceFirstBumper( void ) {
    if ( bumper_queue.size() == bumper_queue.capacity() )
        return (getCurrentTime() - bumper_queue.front());
    return 1000;
}
