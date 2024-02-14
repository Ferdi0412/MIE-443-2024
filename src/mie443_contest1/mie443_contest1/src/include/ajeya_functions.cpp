#ifndef AJEYA_FUNCTIONS_CPP
#define AJEYA_FUNCTIONS_CPP

#include "assumed_functions.hpp"
#include "lin_approx.hpp"

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
#include "../robot.cpp"

// Temp imports
#include <cmath>
#include <iostream>
#include <vector>

int moveForwardsByRev2( Team1::Robot& robot, double target_distance, float wall_distance ) {
    double start_x, start_y;
    std::vector<float> laser_scan;

    // Update positions
    robot.spinOnce();

    // Store starting positions
    start_x = robot.getX();
    start_y = robot.getY();

    std::cout << "Moving forwards by REV2 --- Running..." << std::endl;


    // Try to start forwards motion...
    try {
        if ( target_distance > 0 ){
            robot.jogForwardsSafe( LINEAR_VELOCITY );
        }
        else if ( target_distance < 0 ){
            robot.jogForwardsSafe( -LINEAR_VELOCITY );
        }
        else{
            return REACHED_TARGET;
            std::cout << "Moving forwards by REV2 --- REACHED TARGET..." << std::endl;
        }
    }
    catch (BumperException) {
        std::cout << "Moving forwards by REV2 --- BUMPED into something..." << std::endl;
        
        return WALL_BUMPED;

    }

    // During motion... Until target_distance is reached...
    while ( robot.distanceToPoint( start_x, start_y ) < target_distance ) {
        robot.spinOnce();

        // Check bumpers
        try {
            robot.checkBumpers();
        } catch (BumperException) {
            std::cout << "Moving forwards by REV2 --- BUMPED into something..." << std::endl;

            return WALL_BUMPED;
        }

        // Check distance in laser_scan
        laser_scan = robot.getRanges();

        // Calculated distance to wall in front
        if ( distanceToWallHeadOn( robot ) <= wall_distance ) {
            std::cout << "Moving forwards by REV2 --- ..." << std::endl;

            robot.stopMotion();
            
            return WALL_IN_FRONT;
        }
    }

    // If no early stop... stop motion and return that target location was reached...
    robot.stopMotion();
    return REACHED_TARGET;
}

bool checkIfFacingCorner( Team1::Robot& robot , int MIN_DISTANCE){

    // very niche case - facing a corner head on

    // the_vector stores all the laserScan readings
    const std::vector<float> the_vector = robot.getRanges();
    // std::cout << "No. of laser data points: " << the_vector.size() << std::endl;

    float left_scan_value = std::isnan(the_vector.front()) ? std::numeric_limits<float>::infinity() : the_vector.front();
    std::cout << "<<<--- Port Side: " << left_scan_value;

    // Head-on distance is the middle value of the peripheral field of view
    int vector_size = the_vector.size();
    float middle_scan_value = (vector_size > 0 && !std::isnan(the_vector[vector_size / 2])) ? the_vector[vector_size / 2] : std::numeric_limits<float>::infinity();
    std::cout << "      ^^^ Fore Side: " << middle_scan_value;

    // Starboard side scanned value
    float right_scan_value = std::isnan(the_vector.back()) ? std::numeric_limits<float>::infinity() : the_vector.back();
    std::cout << " ^^^      Starboard Side: " << right_scan_value;

    std::cout << " --->>>" << std::endl;

    int obstacle_type=0;

    if (!the_vector.empty()){


        std::cout << "Obstacle type before scan: " << obstacle_type << std::endl;


        if (left_scan_value <= MIN_DISTANCE && right_scan_value >= left_scan_value && middle_scan_value >= left_scan_value){
            // Robot faces a wall on the left
            obstacle_type = 111;
            std::cout << "Obstacle type after scan: " << obstacle_type << std::endl;
            return false;
        }
        else if (right_scan_value <= MIN_DISTANCE && right_scan_value <= middle_scan_value && middle_scan_value <= left_scan_value){
            // Robot faces a wall on the right
            obstacle_type = 333;
            std::cout << "Obstacle type after scan: " << obstacle_type << std::endl;
            return false;

        }
        else if (middle_scan_value >= MIN_DISTANCE && right_scan_value <= middle_scan_value && left_scan_value <= middle_scan_value){
            // Robot faces a corner head on
            obstacle_type = 222;
            std::cout << "Obstacle type after scan: " << obstacle_type << std::endl;
            return true;
        }
        else{
            // no corner
            obstacle_type = 1;
            std::cout << "Obstacle type after scan: " << obstacle_type << std::endl;
            return false;
        }

    }
    else{
        std::cout << "Vector is empty!" << std::endl;
        return false;
    }


}

double distanceToWallHeadOn( Team1::Robot& robot ){

    // the_vector stores all the laserScan readings
    const std::vector<float> the_vector = robot.getRanges();
    // std::cout << "No. of laser data points: " << the_vector.size() << std::endl;

    float left_scan_value = std::isnan(the_vector.front()) ? std::numeric_limits<float>::infinity() : the_vector.front();
    std::cout << "<<<--- Port Side: " << left_scan_value;

    // Head-on distance is the middle value of the peripheral field of view
    int vector_size = the_vector.size();
    float middle_scan_value = (vector_size > 0 && !std::isnan(the_vector[vector_size / 2])) ? the_vector[vector_size / 2] : std::numeric_limits<float>::infinity();
    std::cout << "      ^^^ Fore Side: " << middle_scan_value;

    // Starboard side scanned value
    float right_scan_value = std::isnan(the_vector.back()) ? std::numeric_limits<float>::infinity() : the_vector.back();
    std::cout << " ^^^      Starboard Side: " << right_scan_value;

    std::cout << " --->>>" << std::endl;

    int start_index = std::max(0, vector_size / 2 - 5); // Ensure start_index doesn't go negative
    int end_index = std::min(vector_size, start_index + 10);

    // Compute the average of the middle ten values
    double middle_scan_avg = 0.0;
    int num_valid_values = 0;
    for (int i = start_index; i < end_index; ++i) {
        if (!std::isnan(the_vector[i])) {
            middle_scan_avg += the_vector[i];
            num_valid_values++;
        }
    }
    if (num_valid_values > 0) {
        middle_scan_avg /= static_cast<double>(num_valid_values);
    } else {
        middle_scan_avg = std::numeric_limits<double>::infinity();
    }

    return middle_scan_avg;

}

bool wallHeadOn( Team1::Robot& robot , int MIN_DISTANCE){

    /*

    Returns true if there is a wall in front of the robot (head on), else false

    uses linear approximation to return true ONLY if the estimated head on wall is flat

    */

    // the_vector stores all the laserScan readings
    const std::vector<float> the_vector = robot.getRanges();
    // std::cout << "No. of laser data points: " << the_vector.size() << std::endl;

    float left_scan_value = std::isnan(the_vector.front()) ? std::numeric_limits<float>::infinity() : the_vector.front();
    std::cout << "<<<--- Port Side: " << left_scan_value;

    // Head-on distance is the middle value of the peripheral field of view
    int vector_size = the_vector.size();
    float middle_scan_value = (vector_size > 0 && !std::isnan(the_vector[vector_size / 2])) ? the_vector[vector_size / 2] : std::numeric_limits<float>::infinity();
    std::cout << "      ^^^ Fore Side: " << middle_scan_value;

    // Starboard side scanned value
    float right_scan_value = std::isnan(the_vector.back()) ? std::numeric_limits<float>::infinity() : the_vector.back();
    std::cout << " ^^^      Starboard Side: " << right_scan_value;

    std::cout << " --->>>" << std::endl;


    int obstacle_type = 0;

    if (!the_vector.empty()){


        std::cout << "Obstacle type before scan: " << obstacle_type << std::endl;


        if (left_scan_value <= MIN_DISTANCE && right_scan_value >= left_scan_value && middle_scan_value >= left_scan_value){
            // Robot faces a wall on the left
            obstacle_type = 111;
            std::cout << "Obstacle type after scan: " << obstacle_type << std::endl;
            return false;
        }
        else if (right_scan_value <= MIN_DISTANCE && right_scan_value <= middle_scan_value && middle_scan_value <= left_scan_value){
            // Robot faces a wall on the right
            obstacle_type = 333;
            std::cout << "Obstacle type after scan: " << obstacle_type << std::endl;
            return false;

        }
        else if (middle_scan_value <= MIN_DISTANCE && right_scan_value >= middle_scan_value && middle_scan_value <= left_scan_value){
            // Robot faces a wall head on
            obstacle_type = 222;
            std::cout << "Obstacle type after scan: " << obstacle_type << std::endl;

            lin_approx_t linear_approximation;
            linear_approximation = linearApproximation(the_vector, the_vector.size() * 2 / 5, the_vector.size() * 3 / 5, 0.);

            if (isStraightLine(linear_approximation, 0.1)){
                return true;
            }
        }
        else{
            // no wall
            obstacle_type = 1;
            std::cout << "Obstacle type after scan: " << obstacle_type << std::endl;
            return false;
        }

    }
    else{
        std::cout << "Vector is empty!" << std::endl;
        return false;
    }


}

bool wallNearby( Team1::Robot& robot , int MIN_DISTANCE){

    /*

    Returns true if there is a wall nearby, else false

    */

    // the_vector stores all the laserScan readings
    const std::vector<float> the_vector = robot.getRanges();
    // std::cout << "No. of laser data points: " << the_vector.size() << std::endl;

    float left_scan_value = std::isnan(the_vector.front()) ? std::numeric_limits<float>::infinity() : the_vector.front();
    std::cout << "<<<--- Port Side: " << left_scan_value;

    // Head-on distance is the middle value of the peripheral field of view
    int vector_size = the_vector.size();
    float middle_scan_value = (vector_size > 0 && !std::isnan(the_vector[vector_size / 2])) ? the_vector[vector_size / 2] : std::numeric_limits<float>::infinity();
    std::cout << "      ^^^ Fore Side: " << middle_scan_value;

    // Starboard side scanned value
    float right_scan_value = std::isnan(the_vector.back()) ? std::numeric_limits<float>::infinity() : the_vector.back();
    std::cout << " ^^^      Starboard Side: " << right_scan_value;

    std::cout << " --->>>" << std::endl;


    int obstacle_type = 0;

    if (!the_vector.empty()){


        std::cout << "Obstacle type before scan: " << obstacle_type << std::endl;


        if (left_scan_value <= MIN_DISTANCE && right_scan_value >= left_scan_value && middle_scan_value >= left_scan_value){
            // Robot faces a wall on the left
            obstacle_type = 111;
            std::cout << "Obstacle type after scan: " << obstacle_type << std::endl;
            return true;
        }
        else if (right_scan_value <= MIN_DISTANCE && right_scan_value <= middle_scan_value && middle_scan_value <= left_scan_value){
            // Robot faces a wall on the right
            obstacle_type = 333;
            std::cout << "Obstacle type after scan: " << obstacle_type << std::endl;
            return true;

        }
        else if (middle_scan_value <= MIN_DISTANCE && right_scan_value >= middle_scan_value && middle_scan_value <= left_scan_value){
            // Robot faces a wall head on
            obstacle_type = 222;
            std::cout << "Obstacle type after scan: " << obstacle_type << std::endl;
            return true;
        }
        else{
            // no wall
            obstacle_type = 1;
            std::cout << "Obstacle type after scan: " << obstacle_type << std::endl;
            return false;
        }

    }
    else{
        std::cout << "Vector is empty!" << std::endl;
        return false;
    }


}


#endif
