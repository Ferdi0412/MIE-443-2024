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
    if ( wall_turn > 0 )
        return wall_turn;
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
    float wall_angle = getWallAngleFromLaserScanNonStraight(robot);
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
        return NO_MOVE;
    }

    if (wall_angle < 0){
        ROS_INFO("Turn CW");
        return turnRobotBy(robot, -wall_angle) || REACHED_TARGET_RIGHT;
    } else if (wall_angle > 0){
        ROS_INFO("Turn CCW");
        return turnRobotBy(robot, -wall_angle) || REACHED_TARGET_LEFT;
    } else if (wall_angle == 0){
        ROS_INFO("Already PARALLEL");
        return REACHED_TARGET_CENTER; // Already parallel
    }

    return WALL_NOT_FOUND;
}

// Function to make the robot avoid obstacles
int avoidObstacles(Team1::Robot& robot, wallDirectionEnum dir) {

    const float MIN_DISTANCE = 0.4; // Minimum distance to consider an obstacle
    const float MAX_DISTANCE = 0.6; // Maximum distance to consider an obstacle

    int move_res;

    // Laser scan data
    std::vector<float> laser_ranges = robot.getRanges();
    // printVectorFloats(laser_ranges);

    if (!laser_ranges.empty()) {

        float middle_value = laser_ranges[laser_ranges.size() / 2];
        const double right_value = laser_ranges[0];
        const double left_value = laser_ranges[laser_ranges.size() - 1];

        std::cout << "Middle Value: " << middle_value << std::endl;
        std::cout << "Left Value:" << left_value << std::endl;
        std::cout << "Right Value" << right_value << std::endl;

        if (dir == any){
            if (middle_value > MAX_DISTANCE){
                ROS_INFO("Move FORWARD");
                return moveForwardsBy(robot, middle_value - MAX_DISTANCE, MAX_DISTANCE) || REACHED_TARGET;
            } else if (right_value >= left_value){
                ROS_INFO("TURN RIGHT");
                move_res = turnRobotBy(robot, 90);
                if ( move_res > 0 ) return move_res;
                robot.spinOnce();
                laser_ranges = robot.getRanges();
                middle_value = laser_ranges[laser_ranges.size() / 2];
                return moveForwardsBy(robot, middle_value - MAX_DISTANCE, MAX_DISTANCE) || REACHED_TARGET_RIGHT;
            } else if (right_value < left_value){
                ROS_INFO("TURN LEFT");
                move_res = turnRobotBy(robot, -90);
                if ( move_res > 0 ) return move_res;
                robot.spinOnce();
                laser_ranges = robot.getRanges();
                middle_value = laser_ranges[laser_ranges.size() / 2];
                return moveForwardsBy(robot, middle_value - MAX_DISTANCE, MAX_DISTANCE) || REACHED_TARGET_LEFT;
            }

        } else if (dir == left){
            if (middle_value <= MAX_DISTANCE){
                ROS_INFO("TURN LEFT");
                return turnRobotBy(robot, -90) || REACHED_TARGET_LEFT;
            }
        } else if (dir == right){
            if (middle_value <= MAX_DISTANCE){
                ROS_INFO("TURN RIGHT");
                return turnRobotBy(robot, 90) || REACHED_TARGET_RIGHT;
            }
        }
    }
    ROS_WARN("=== avoidObstacles ---> DEFAULT\n");
    return REACHED_TARGET; // Default case
}


#endif
