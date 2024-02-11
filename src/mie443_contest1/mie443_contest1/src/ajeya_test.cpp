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

// trying to get to use this as an interrupt in the main obs avoid function
class CollisionException : std::exception {};
// class BumperException : std::exception {};


/**
 * secondsElapsed
 *
 * @returns number of seconds from program_start
*/
uint16_t secondsElapsed(void);

/**
 * printVectorFloats prints each float in a vector
*/

int detectObstacles(Team1::Robot& robot);
void dodgeObstacles(Team1::Robot& robot, int obstacle);
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



#define SPEED_HIGH 0.1
// #define ROT_HIGH 0.2


/**
 * ============
 * === MAIN ===
 * ============
 *
 * @param argc number of params used in starting program (ignore but keep)
 * @param argv string params used when starting program  (ignore but keep)
*/





// ####################
//  MAIN LOOP         #
// ####################

int main ( int argc, char **argv ) {
    // ROS setup
    ros::init(argc, argv, "contest1");
    ROS_INFO("Starting up...\n");
    ros::NodeHandle nh;
    ros::Rate loop_rate(2);
    ROS_INFO("Creating Robot");

    // Robot object setup
    Team1::Robot robot( nh, 2);
    // establishing contact with callback functions and refreshing data input readings
    robot.spinOnce();
    // Sleep to ensure is initialized correctly
    ros::Duration(0.5).sleep(); 

    // GLOBAL params setup
    program_start = std::chrono::system_clock::now();

    int final_obstacle_type = 0;
    int movement_stopped = 0;



    // ####################
    //  CONTROL LOOP      #
    // ####################

    while ( ros::ok() && secondsElapsed() <= program_duration && final_obstacle_type != 69) {
        // std::cout << "Ranges:\n";
        // printVectorFloats( robot.getRanges() );
        // std::cout << "N Lasers: " << robot.getNLasers() << "\n";
        // try {
        //       robot.checkBumpers();
        // } catch ( BumperException) {
        //     rotateAfterBumper(robot);
        //  }
        
        // robot.spinOnce();
        // ROS_INFO("Position: %.2f\nSpeed: %.2f\n", robot.getTheta(), robot.getVelTheta());
        // // Check for obstacles and avoid them
        // try {
        //       robot.checkBumpers();
        //       movement = avoidObstacles(robot);
        // } catch ( BumperException) {
        //     rotateAfterBumper(robot);
        // }

        // ajeya's obstacle avoidance code

        robot.spinOnce();

        // add try that handles obstacle avoidance using laser scanners

        try {
            robot.spinOnce();
            // robot.rotateClockwiseBy(60, 90);
            // robot.moveForwards(0.2,0.3);
            final_obstacle_type = detectObstacles(robot);
            std::cout << "Obstacle Type: " << final_obstacle_type << std::endl;
            
            if (final_obstacle_type == 1){
                robot.moveForwards(0.2,0.3);
                ROS_INFO("Sailing forward...");
            }
            else if (final_obstacle_type == 69){
                // robot.rotateClockwiseBy(60, 90);
                ROS_INFO("Error 69...");
            }
            

        } catch (CollisionException){
            ROS_INFO("Obstacle Detected...");

            try {

                dodgeObstacles(robot, final_obstacle_type);
                ROS_INFO("Dodged...");

            } catch (BumperException) {
                // checking if obs avoid works without bumpers
                movement_stopped = 1;
            }

        }



        
        robot.sleepOnce();
    }

    robot.stopMotion();
    ROS_INFO("Stopping robot!\n");
}




/**
 * =================================
 * === FUNCTIONS IMPLEMENTATIONS ===
 * =================================
*/

// Function to make the robot detect obstacles hopefully without hitting them
int detectObstacles(Team1::Robot& robot) {
    robot.stopMotion();

    const float MIN_DISTANCE = 0.5; // Minimum distance to consider an obstacle
    const float MAX_DISTANCE = 0.75; // Maximum distance to consider an obstacle

    // the_vector stores all the laserScan readings
    const std::vector<float> the_vector = robot.getRanges();
    std::cout << "No. of laser data points: " << the_vector.size() << std::endl;


    // // port side scanned value (the_vector[0])
    // float left_scan_value = the_vector.front();
    // std::cout << "<<<--- Port Side: " << left_scan_value;
    // // head on distance is the middle value of the peripheral field of view
    // int vector_size = the_vector.size()/2;
    // float middle_scan_value = the_vector[vector_size];
    // std::cout << "      ^^^ Fore Side: " << middle_scan_value;
    // // starboard side scanned value (the_vector[-1])
    // float right_scan_value = the_vector.back();
    // std::cout << " ^^^      Starboard Side: " << right_scan_value;
    // std::cout << " --->>>" << std::endl;

    float left_scan_value = std::isnan(the_vector.front()) ? 2.0f : the_vector.front();
    std::cout << "<<<--- Port Side: " << left_scan_value;

    // Head-on distance is the middle value of the peripheral field of view
    int vector_size = the_vector.size();
    float middle_scan_value = (vector_size > 0 && !std::isnan(the_vector[vector_size / 2])) ? the_vector[vector_size / 2] : 2.0f;
    std::cout << "      ^^^ Fore Side: " << middle_scan_value;

    // Starboard side scanned value
    float right_scan_value = std::isnan(the_vector.back()) ? 2.0f : the_vector.back();
    std::cout << " ^^^      Starboard Side: " << right_scan_value;

    std::cout << " --->>>" << std::endl;
    
    

    // check if laser is sending legitimate values if not, close function
    if (!the_vector.empty()) {
        int obstacle_type = 0;
        std::cout << "Obstacle type before loop: " << obstacle_type << std::endl;


        if (left_scan_value <= MIN_DISTANCE && right_scan_value >= left_scan_value && middle_scan_value >= left_scan_value){
            // Robot faces a wall on the left
            obstacle_type = 111;
            std::cout << "Obstacle type after loop: " << obstacle_type << std::endl;

            throw CollisionException();
            return obstacle_type;
        }
        else if (right_scan_value <= MIN_DISTANCE && right_scan_value <= middle_scan_value && middle_scan_value <= left_scan_value){
            // Robot faces a wall on the right
            obstacle_type = 333;
            std::cout << "Obstacle type after loop: " << obstacle_type << std::endl;

            throw CollisionException();
            return obstacle_type;

        }
        else if (middle_scan_value <= MIN_DISTANCE && right_scan_value >= middle_scan_value && middle_scan_value <= left_scan_value){
            // Robot faces a wall head on
            obstacle_type = 222;
            std::cout << "Obstacle type after loop: " << obstacle_type << std::endl;

            throw CollisionException();
            return obstacle_type;

        }

        else if (left_scan_value <= MIN_DISTANCE){
            obstacle_type = 11;
            std::cout << "Obstacle type after loop: " << obstacle_type << std::endl;

            throw CollisionException();
            return obstacle_type;

        }
        else if (right_scan_value <= MIN_DISTANCE){
            obstacle_type = 33;
            std::cout << "Obstacle type after loop: " << obstacle_type << std::endl;

            throw CollisionException();
            return obstacle_type;

        }
        else if (middle_scan_value <= MIN_DISTANCE){
            obstacle_type = 22;
            std::cout << "Obstacle type after loop: " << obstacle_type << std::endl;

            throw CollisionException();
            return obstacle_type;

        }
        else{
            // no obstacle
            obstacle_type = 1;
            return obstacle_type;
            std::cout << "Obstacle type after loop: " << obstacle_type << std::endl;
            

        }
    } else {
        std::cout << "Vector is empty!" << std::endl;
        return 69;
    }
}

// Function to make the robot detect obstacles hopefully without hitting them
void dodgeObstacles(Team1::Robot& robot, int obstacle) {
    const float MIN_DISTANCE = 0.2; // Minimum distance to consider an obstacle
    const float MAX_DISTANCE = 0.3; // Maximum distance to consider an obstacle

    // the_vector stores all the laserScan readings
    const std::vector<float> the_vector = robot.getRanges();
    std::cout << "No. of laser data points: " << the_vector.size() << std::endl;


    // // port side scanned value (the_vector[0])
    // float left_scan_value = the_vector.front();
    // std::cout << "<<<--- Port Side: " << left_scan_value;
    // // head on distance is the middle value of the peripheral field of view
    // float middle_scan_value = the_vector[the_vector.size() / 2];
    // std::cout << "      ^^^ Fore Side: " << middle_scan_value;
    // // starboard side scanned value (the_vector[-1])
    // float right_scan_value = the_vector.back();
    // std::cout << " ^^^      Starboard Side: " << right_scan_value;
    // std::cout << " --->>>" << std::endl;

    float left_scan_value = std::isnan(the_vector.front()) ? 2.0f : the_vector.front();
    std::cout << "<<<--- Port Side: " << left_scan_value;

    // Head-on distance is the middle value of the peripheral field of view
    int vector_size = the_vector.size();
    float middle_scan_value = (vector_size > 0 && !std::isnan(the_vector[vector_size / 2])) ? the_vector[vector_size / 2] : 2.0f;
    std::cout << "      ^^^ Fore Side: " << middle_scan_value;

    // Starboard side scanned value
    float right_scan_value = std::isnan(the_vector.back()) ? 2.0f : the_vector.back();
    std::cout << " ^^^      Starboard Side: " << right_scan_value;

    std::cout << " --->>>" << std::endl;

    bool wall_on_left,wall_on_right, wall_head_on;
    bool obstacle_on_left, obstacle_on_right, obstacle_head_on;

    // check if laser is sending legitimate values if not, close function
    if (!the_vector.empty()) {

        // Part 1: Wall detection

        if (obstacle == 111){
            // Robot faces a wall on the left
            // change orientation to head on to the wall and then rotate 90
            // dont do this if walls on all three sides

            bool wall_on_left = true;
            ROS_INFO("Port Side Wall Detected...");

        }
        if (obstacle == 333){
            // Robot faces a wall on the right

            bool wall_on_right = true;
            ROS_INFO("Starboard Side Wall Detected...");

        }
        if (obstacle == 222){
            // Robot faces a wall head on

            bool wall_head_on = true;
            ROS_INFO("Head on Wall Detected...");

        }

        // Part 2: Object Detection

        if (obstacle == 11){
            bool obstacle_on_left = true;
        }
        if (obstacle == 33){
            bool obstacle_on_right = true;
        }
        if (obstacle == 22){
            bool obstacle_head_on = true;
        }

        // Part 3: Correction 
        // ik part 1 and part 2 can be removed but i will do that later

            // obstacles
        if (obstacle_on_left){
            robot.stopMotion();
            robot.rotateClockwiseBy(50,45);
            ROS_INFO("Rotating CW...");
            // return 0;
        }else if (obstacle_on_right){
            robot.stopMotion();
            robot.rotateClockwiseBy(50,-45);
            ROS_INFO("Rotating CCW...");
            // return 0;
        }else if (obstacle_head_on){
            robot.stopMotion();
            robot.rotateClockwiseBy(50,180);
            ROS_INFO("Rotating 180 CW...");
            // return 0;
        }


            // walls (NOT DONE YET)
        if (wall_on_left){
            robot.stopMotion();
            robot.rotateClockwiseBy(50,45);
            ROS_INFO("Rotating CW...");
            // return 0;
        }else if (wall_on_right){
            robot.stopMotion();
            robot.rotateClockwiseBy(50,-45);
            ROS_INFO("Rotating CCW...");
            // return 0;
        }else if (wall_head_on){
            robot.stopMotion();
            robot.rotateClockwiseBy(50,180);
            ROS_INFO("Rotating 180 CW...");
            // return 0;
        }

        // add inverse trig parallelize code

        // if (middle_value <= MAX_DISTANCE){
        //     ROS_INFO("STOP and TURN");
        //     //Obstacle detected STOP
            // robot.stopMotion();
        //     //Turn away from the obstacle
        //     robot.rotateClockwiseBy(50,-45);
        //     return 0;
        // } else{
        //     //No obstacle detected, MoVE FORWARD
        //     robot.moveForwards(0.2,0.3);
        //     ROS_INFO("Move forward");
        //     return 0;
        // }
    } else {
        std::cout << "Vector is empty!" << std::endl;
        throw BumperException();
        // return 1;
    }
}

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