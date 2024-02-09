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
bool avoidObstacles(Team1::Robot& robot);
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

// Function to make the robot avoid obstacles
bool avoidObstacles(Team1::Robot& robot) {
    const float MIN_DISTANCE = 0.3; // Minimum distance to consider an obstacle
    const float MAX_DISTANCE = 0.6; // Maximum distance to consider an obstacle

    // Check if an obstacle is detected within the specified range
    const std::vector<float> the_vector = robot.getRanges();
    if (!the_vector.empty()) {
        const float middle_value = the_vector[the_vector.size() / 2];
        std::cout << "Middle Value: " << middle_value << std::endl;
        if (middle_value <= MAX_DISTANCE){
            ROS_INFO("STOP and TURN");
            //Obstacle detected STOP
            robot.stopMotion();
            //Turn away from the obstacle
            robot.rotateClockwiseBy(50,-45);
            return true;
        } else{
            //No obstacle detected, MoVE FORWARD
            robot.moveForwards(0.2,0.3);
            ROS_INFO("Move forward");
            return true;
        }
    } else {
        std::cout << "Vector is empty!" << std::endl;
        return false;
    }
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

bool movement = true;

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

    while ( ros::ok() && secondsElapsed() <= program_duration && movement == true ) {
        std::cout << "Ranges:\n";
        printVectorFloats( robot.getRanges() );
        std::cout << "N Lasers: " << robot.getNLasers() << "\n";
        try {
              robot.checkBumpers();
        } catch ( BumperException) {
            rotateAfterBumper(robot);
         }
        
        robot.spinOnce();
        ROS_INFO("Position: %.2f\nSpeed: %.2f\n", robot.getTheta(), robot.getVelTheta());
        // Check for obstacles and avoid them
        try {
              robot.checkBumpers();
              movement = avoidObstacles(robot);
        } catch ( BumperException) {
            rotateAfterBumper(robot);
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
