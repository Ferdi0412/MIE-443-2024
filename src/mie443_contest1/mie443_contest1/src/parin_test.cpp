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

// Enum for direction
enum Direction { LEFT, RIGHT, ANY };

/**
 * printVectorFloats prints each float in a vector
*/
void printVectorFloats( const std::vector<float>& the_vector );

Direction wallFollow(Team1::Robot& robot, Direction dir);
int wallParallel(Team1::Robot& robot);
Direction avoidObstacles(Team1::Robot& robot, Direction dir);
void rotateAfterBumper(Team1::Robot& robot);

/**
 * =====================
 * === GLOBAL params ===
 * =====================
*/
static std::chrono::time_point<std::chrono::system_clock> program_start;
static const unsigned long long program_duration = 500;


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

// ****** Wall Follow Function ***** //
Direction wallFollow(Team1::Robot& robot, Direction dir) {
    
    // Parallelize with wall
    int wall_turn = wallParallel(robot);
    robot.spinOnce();

    // Obstacle Avoidance Algorithm
    Direction direction = avoidObstacles(robot, dir);
    
    return direction;
}

// ***** Parallelize Robot wrt Walls ***** //
int wallParallel(Team1::Robot& robot) {

    const float MAX_DISTANCE = 0.6;
    // Laser scan data
    const std::vector<float> laser_ranges = robot.getRanges();
    const float right_value = laser_ranges[0];
    const float left_value = laser_ranges[laser_ranges.size() - 1];

    // Wall angle data
    // const float wall_angle = getWallAngleFromLaserScan();
    const float wall_angle = 0;

    if (wall_angle != 0){
        if (right_value <= left_value){
            ROS_INFO("Turn CW");
            // turnRobotBy(robot, wall_angle);
            return 1; //Turn CW
        } else if (right_value > left_value){
            ROS_INFO("Turn CCW");
            // turnRobotBy(robot, -wall_angle);
            return 2; //Turn CCW
        }
    } else if (wall_angle == 0) {
        ROS_INFO("Already PARALLEL");
        return 0; // Already parallel
    }

}

// Function to make the robot avoid obstacles
Direction avoidObstacles(Team1::Robot& robot, Direction dir) {

    const float MIN_DISTANCE = 0.3; // Minimum distance to consider an obstacle
    const float MAX_DISTANCE = 0.6; // Maximum distance to consider an obstacle

    // Laser scan data
    const std::vector<float> laser_ranges = robot.getRanges();
    // printVectorFloats(laser_ranges);
    
    if (!laser_ranges.empty()) {

        const float middle_value = laser_ranges[laser_ranges.size() / 2];
        const float right_value = laser_ranges[0];
        const float left_value = laser_ranges[laser_ranges.size() - 1];
        
        std::cout << "Middle Value: " << middle_value << std::endl;
        std::cout << "Left Value:" << left_value << std::endl;
        std::cout << "Right Value" << right_value << std::endl;

        if (dir == ANY){
            if (middle_value > MAX_DISTANCE){
                ROS_INFO("Move FORWARD");
                // moveForwardsBy(robot,0.2,MAX_DISTANCE);
                return ANY;
            } else if (right_value >= left_value){
                ROS_INFO("TURN RIGHT");
                // turnRobotBy(robot, 90);
                return RIGHT;
            } else if (right_value < left_value){
                ROS_INFO("TURN LEFT");
                // turnRobotBy(robot, -90);
                return LEFT;
            }

        } else if (dir == LEFT){
            if (middle_value <= MAX_DISTANCE){
                ROS_INFO("TURN LEFT");
                // turnRobotBy(robot, -90);
                return LEFT;
            }
        } else if (dir == RIGHT){
            if (middle_value <= MAX_DISTANCE){
                ROS_INFO("TURN RIGHT");
                // turnRobotBy(robot, 90);
                return RIGHT;
            }
        }
    }
    return ANY; // Default case
}

void rotateAfterBumper(Team1::Robot& robot){
            if (robot.getBumperRight() == true){
                robot.moveForwards(-0.25,0.2);
                robot.rotateClockwiseBy(60, -45);
            }
            else if (robot.getBumperLeft() == true){
                robot.moveForwards(-0.25,0.2);
                robot.rotateClockwiseBy(60, 45);
            }
            else {
            robot.moveForwards(-0.25,0.2);
}
    robot.spinOnce();
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
    Direction dir = ANY;

    while ( ros::ok() && secondsElapsed() <= program_duration && (dir == ANY || dir == LEFT || dir == RIGHT)) {
        std::cout << "Ranges:\n";
        // printVectorFloats( robot.getRanges() );
        std::cout << "N Lasers: " << robot.getNLasers() << "\n";
        try {
              robot.checkBumpers();
        } catch ( BumperException) {
            try{
                rotateAfterBumper(robot);
            } catch(BumperException){
            }
         }
        
        robot.spinOnce();
        ROS_INFO("Position: %.2f\nSpeed: %.2f\n", robot.getTheta(), robot.getVelTheta());
        // Check for obstacles and avoid them
        try {
              robot.checkBumpers();
              dir = wallFollow(robot, dir);
        } catch ( BumperException) {
            try{
                rotateAfterBumper(robot);
            } catch(BumperException){
            }
        }
        
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
    // std::cout << the_vector.size();
    for ( unsigned int i = 0; i < the_vector.size(); i++ )
        std::cout << the_vector[i] << "; ";
    //for ( const float& val : the_vector )
    //    std::cout << val << "; ";
    std::cout << "\n";
}
