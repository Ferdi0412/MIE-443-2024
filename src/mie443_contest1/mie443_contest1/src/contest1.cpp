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

// Higher level movements
#include "include/assumed_functions.hpp"

// Ferdi's functions
#include "include/ferd_functions.cpp"

// ========= AUXILLIARY =========
#define WALL_DISTANCE 0.2

typedef unsigned long long time_elapsed_t;

time_elapsed_t secondsElapsed(void);

// FERDI -> To implement...
time_elapsed_t timeSinceFirstBumper( void ); // Return the first bumper in the queue...

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
        robot.spinOnce(); // Update values....
        /**
         * === MOVEMENTS FAILED ===
        */
        if ( move_res > 0 ) { // If move_res > 0 -> some movement failed/returned early
            ROS_INFO("Handling movement 'exception'...\n");
            // Handle cases such as bumper triggered here...
            // Rotate after bumping into a wall
            if ( move_res == WALL_BUMPED ) { // EMMA -> Go through and check logic
                if ( wallInFront( robot ) && distanceToWall( robot ) > 0.1 ) {
                    move_res = rotateAfterBumper( robot );
                }
                else move_res = rotateAfterBumper( robot );
            }
            // Do nothing if wall scan stopped motion -> allow the wallFollow to work if needed...
            else if ( move_res == WALL_IN_FRONT ) ;
        }

        /** ALL -> consider this
         * TBD: Use odometry to track previous movements/positions to prevent stuck in cycle/loop
        */

        /**
         * === BUMPED TOO OFTEN ===
        */
        if ( timeSinceFirstBumper() < 10 ) { // If too many bumps in the last 10 seconds...
            ROS_WARN("=== BUMPED TOO OFTEN ===\n");

            // NOTE: If this block just ran, and move_res indicates a wall bummp, you likely didn't do much...
            // Consider "forceful turn..." -> moves regardless of bumping

            // Implement logic here...
            move_res = turnRobotBy( robot, 180 );
            if ( move_res > 0 ) continue; // Go to the MOVEMENTS FAILED section...
            move_res = randomMotion( robot, -45, 45 );
            continue;
        }

        // STARTUP/NORMAL CYCLE
        if ( wallInFront( robot ) ) {
            move_res = wallFollow( robot, direction );
            continue;
        }
        else if ( emptyInFront( robot ) ) {
            moveForwardsBy( robot, 0.2, WALL_DISTANCE );
            continue;
        }
        else if ( checkIfFacingCorner( robot ) ) { // V-Shaped corner
            move_res = turnRobotBy( robot, 45 );
            continue;
        }
        else {
            move_res = scanForArea( robot );

            // If scanForArea failed
            if ( move_res > 0 )
                move_res = randomMotion( robot, -90, 90 );
            continue;
        }

        /**
         * === STARTUP ===
         * 1. Check for wall   -> if true, wallFollow move in that direction until wall
         * 1b. Check if open in front ->
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
time_elapsed_t secondsElapsed( void ) {
    return std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - program_start).count();
}
