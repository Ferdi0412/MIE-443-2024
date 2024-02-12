#ifndef PARIN_FUNCTIONS_CPP
#define PARIN_FUNCTIONS_CPP


#include "assumed_functions.hpp"
#include "ferd_functions.cpp"
#include <ros/console.h>
#include "ros/ros.h"



int wallParallel(Team1::Robot& robot);
int avoidObstacles(Team1::Robot& robot, wallDirectionEnum dir);


/**
 * === IMPLEMENTATIONS ===
*/

int wallFollow( Team1::Robot& robot, wallDirectionEnum wall_direction ) {
    // Parallelize with wall
    int wall_turn = wallParallel(robot);
    // if ( (wall_turn == WALL_NOT_FOUND) || (wall_turn == NO_MOVE) ) return WALL_NOT_FOUND;
    robot.spinOnce();

    // Obstacle Avoidance Algorithm
    int direction = avoidObstacles(robot, wall_direction);

    return direction;

}

// ***** Parallelize Robot wrt Walls ***** //
int wallParallel(Team1::Robot& robot) {

    const float MAX_DISTANCE = 0.7;
    // Laser scan data
    const std::vector<float> laser_ranges = robot.getRanges();
    const float right_value = laser_ranges[0];
    const float left_value = laser_ranges[laser_ranges.size() - 1];
    const float middle_value = laser_ranges[laser_ranges.size()/2];
    float wall_angle = 90 - getWallAngleFromLaserScan(robot);
    // Wall angle data
    // float wall_angle = getWallAngleFromLaserScan(robot);
    // double distance_to_wall = distanceToWall(robot);

    // if ( std::isinf(wall_angle) ) {
    //     ROS_ERROR("Wall angle is inf!!!\n");
    //     return NO_MOVE;
    // }

    // if ( middle_value >= MAX_DISTANCE) {
    //     moveForwardsBy(robot, middle_value - MAX_DISTANCE, MAX_DISTANCE);
    //     robot.spinOnce();
    //     wall_angle = getWallAngleFromLaserScan(robot);
    // }

    std::cout << "Distance to wall: " << middle_value << std::endl;
    std::cout << "Wall Angle: " << wall_angle << std::endl;
    
    if ( std::isinf(wall_angle) ) {
        ROS_WARN("Wall angle is inf!!!\n");
        return WALL_NOT_FOUND;
    }

    if ( middle_value <= MAX_DISTANCE) {
        if (wall_angle < 0){
            ROS_INFO("Turn CW");
            turnRobotBy(robot, -wall_angle);
            return -2; //Turn CW
        } else if (wall_angle > 0){
            ROS_INFO("Turn CCW");
            turnRobotBy(robot, -wall_angle);
            return -1; //Turn CCW
        } else if (wall_angle == 0){
            ROS_INFO("Already PARALLEL");
            return 0; // Already parallel
        } 
    } 

    return 0;
    

}

// Function to make the robot avoid obstacles
int avoidObstacles(Team1::Robot& robot, wallDirectionEnum dir) {

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

        if (dir == any){
            if (middle_value > MAX_DISTANCE){
                ROS_INFO("Move FORWARD");
                moveForwardsBy(robot, middle_value - 0.4,MAX_DISTANCE);
                return 0;
            } else if (right_value >= left_value){
                ROS_INFO("TURN RIGHT");
                turnRobotBy(robot, 90);
                return -2;
            } else if (right_value < left_value){
                ROS_INFO("TURN LEFT");
                turnRobotBy(robot, -90);
                return -1;
            }

        } else if (dir == left){
            if (middle_value <= MAX_DISTANCE){
                ROS_INFO("TURN LEFT");
                turnRobotBy(robot, -90);
                return -1;
            }
        } else if (dir == right){
            if (middle_value <= MAX_DISTANCE){
                ROS_INFO("TURN RIGHT");
                // turnRobotBy(robot, 90);
                return -2;
            }
        }
    }
    return 0; // Default case
}


#endif
