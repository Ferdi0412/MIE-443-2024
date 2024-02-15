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
#define MINUTE 60

typedef unsigned long long time_elapsed_t;

time_elapsed_t secondsElapsed(void);

time_elapsed_t getCurrentTime( void );

void storeBumperTimestamp( void ); // Stores the current timestamp in the bumper events queue

void initializeBumperQueue( void );

boost::circular_buffer<time_elapsed_t> bumper_queue(3); // 3 Bumper events...

time_elapsed_t timeSinceFirstBumper( void );

static std::chrono::time_point<std::chrono::system_clock> program_start;

const static time_elapsed_t program_duration = 480;

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
    Team1::Robot robot( nh, 50);

    // Sleep to ensure is initialized correctly
    ros::Duration(0.5).sleep();
    robot.spinOnce();

    // Start timer
    program_start = std::chrono::system_clock::now();

    // Add in to wait on laser_ranges
    robot.waitOnLaserRanges(); // FERDI -> To implement

    ROS_WARN("=== INITIAL RANDOM PHASE ===\n");
    while ( ros::ok() && secondsElapsed() <= (MINUTE * 1.) ) {
        // Update timestamps on wall bumps
        if ( move_res == WALL_BUMPED ) {
            storeBumperTimestamp();
            move_res = rotateAfterBumper( robot );
            continue;
        }

        move_res = scanMotion( robot );;
        continue;

    }

    // === MAIN ===
    // loop until program_duration [seconds] is reached
    ROS_WARN("=== MAIN PHASE ===\n");
    while ( ros::ok() && secondsElapsed() <= (program_duration - MINUTE) ) {
        robot.spinOnce(); // Update values....
        /**
         * === MOVEMENTS FAILED ===
        */
        if ( move_res > 0 ) { // If move_res > 0 -> some movement failed/returned early
            ROS_INFO("Handling movement 'exception' ( %d )...\n", move_res);
            // Handle cases such as bumper triggered here...
            // Rotate after bumping into a wall
            if ( move_res == WALL_BUMPED ) { // EMMA -> Go through and check logic
                ROS_INFO("=== WALL BUMPED ===\n");
                storeBumperTimestamp();
                // if ( wallInFront( robot ) && distanceToWall( robot ) > 0.1 ) {
                //     move_res = rotateAfterBumper( robot );
                // }
                // else
                move_res = rotateAfterBumper( robot );
            }
            // Do nothing if wall scan stopped motion -> allow the wallFollow to work if needed...
            else if ( move_res == WALL_IN_FRONT ) ROS_INFO("=== WALL IN FRONT (do nothing) ===\n");
            else {
                ROS_ERROR("UNCAUGHT 'ERROR': %d...\n", move_res);
            }
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
            // continue; // IGNORE THESE ERRORS FOR NOW...
        }

        // STARTUP/NORMAL CYCLE
        if ( wallInFront( robot ) ) {
            ROS_INFO("=== wallInFront ===\n");
            move_res = wallFollow( robot, direction );
            continue;
        }
        else if ( emptyInFront( robot ) ) {
            ROS_INFO("=== emptyInFront ===\n");
            moveForwardsBy( robot, 0.2, WALL_DISTANCE );
            continue;
        }
        else if ( checkIfFacingCorner( robot,  WALL_DISTANCE) ) { // V-Shaped corner
            ROS_INFO("=== checkIfFacingCorner ===\n");
            move_res = turnRobotBy( robot, 45 );
            continue;
        }
        else {
            ROS_INFO("=== scanForArea ===\n");
            move_res = scanMotion( robot );

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

    /**
     * START OF SECOND PHASE!
    */

    move_res = REACHED_TARGET;

    ROS_WARN("=== SECOND PHASE ===\n");
    while ( ros::ok() && secondsElapsed() <= program_duration ) {
        if ( timeSinceFirstBumper() ) {
            randomMotion( robot, -180, 180 );
        }

        // Update timestamps on wall bumps
        if ( move_res == WALL_BUMPED ) {
            storeBumperTimestamp();
            move_res = rotateAfterBumper( robot );
            continue;
        }

        move_res = moveForwardsBy( robot, 1, 0 );
        continue;

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
