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

// ========= EDIT THE FOLLOWING DEFINITION BELOW =========
/**
 * moveAndScan_example
 *
 * An example function for how to move and scan the robot.
 *
 * @param robot the robot object
 * @param angle the distance to travel [meters]
*/
void moveAndScan_example( Team1::Robot robot, double distance );


// ========= AUXILLIARY =========
/**
 * secondsElapsed
 *
 * @returns number of seconds from program_start
*/
uint16_t secondsElapsed(void);

/**
 * exitIfTimeRunOut
 *
 * Will terminate the program (exit/return) if the program has completed
*/
void exitIfTimeRunOut( void );

/**
 * exitIfTimeRunOut [OVERLOAD]
 *
 * Will terminate the program (exit/return) with exit_code if the program has completed
 *
 * @param exit_code 0 means no error, any other value means program ended with an error (C-standard)
*/
void exitIfTimeRunOut( unsigned int exit_code );



/**
 * =====================
 * === GLOBAL params ===
 * =====================
*/
#define LINEAR_SPEED 0.2
#define ROTATIONAL_SPEED 10

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

    while ( ros::ok() && robot.getRanges().size() == 0 ) {
        robot.spinOnce();
        ros::Duration(0.5).sleep();
    }

    program_start = std::chrono::system_clock::now();

    // === MAIN ===
    // loop until program_duration [seconds] is reached
    while ( ros::ok() && secondsElapsed() <= program_duration ) {
        // Main stuff here...  
        ROS_INFO("Position: (%f,%f) degrees Range: (%f,%f) Lasers:(%d)\n",robot.getX(),robot.getY(),robot.getRangeMin(),robot.getRangeMax(),robot.getNLasers());
        std::vector<float> ranges = robot.getRanges();
        ROS_INFO("Ranges size: %d -> %d\n", ranges.size(), robot.getNLasers()/2);
        float center_point = ranges [robot.getNLasers()/2];
        ROS_INFO("Following center_point\n");
        robot.calibrateLinearMotion(1.5);
        try { 
        robot.jogClockwise(0.2);
        robot.moveForwards(0.05,0.1);
        }
        catch (BumperException)
        {
            std::cout << "Wall!!";
        }

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
void moveAndScan_example( Team1::Robot robot, double distance ) {
    double start_x = robot.getX(), start_y = robot.getY();
    robot.jogForwardsSafe( 0.2 ); // Start moving at 0.2 [m/s]

    // Check distance from robot to starting position, until distance (in units of [m]) has been travelled
    while ( (secondsElapsed() << program_duration) && (robot.distanceToPoint( start_x, start_y ) < distance) ) {
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

        robot.sleepOnce();
    }

    // Once enough distance travelled...
    robot.stopMotion();
}


// ========= AUXILLIARY =========
uint16_t secondsElapsed( void ) {
    return std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - program_start).count();
}

void exitIfTimeRunOut( void ) {
    if ( secondsElapsed() > program_duration ) exit(0);
}

void exitIfTimeRunOut( unsigned int exit_code ) {
    if ( secondsElapsed() > program_duration ) exit( exit_code );
}