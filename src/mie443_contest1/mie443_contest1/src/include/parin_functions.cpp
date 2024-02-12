#ifndef FERDI_FUNCTIONS_CPP
#define FERDI_FUNCTIONS_CPP


#include "assumed_functions.hpp"
#include "lin_approx.hpp"

#include <vector>
#include <limits>
#include <cmath>

#define LINEAR_LIMIT 0.5
#define ROTATIONAL_VELOCITY 50
#define LINEAR_VELOCITY 0.2

#ifndef RAD2DEG
#define RAD2DEG(rad) ((rad) * 180. / M_PI)
#endif

#ifndef DEG2RAD
#define DEG2RAD(deg) ((deg) * M_PI / 180.)
#endif


#ifndef NUM_SCAN_SEGM
#define NUM_SCAN_SEGM 5
#endif

int wallParallel(Team1::Robot& robot);
int avoidObstacles(Team1::Robot& robot, wallDirectionEnum dir);


bool wallInFront( Team1::Robot& robot ) {
    lin_approx_t linear_approximation;
    linear_approximation = linearApproximation( robot.getRanges(), robot.getRanges().size() * 2 / 5, robot.getRanges().size() * 3 / 5);
    if ( checkApproximationError( linear_approximation) ) return false;
    return isStraightLine(linear_approximation, 0.1);
}



float getWallAngleFromLaserScan( Team1::Robot& robot ) {
    lin_approx_t linear_approximation;
    std::vector<float> laser_ranges;
    float slope, angle;

    laser_ranges = robot.getRanges();

    linear_approximation = linearApproximation( laser_ranges, laser_ranges.size() * 2 / 5, laser_ranges.size() * 3 / 5 );

    if ( checkApproximationError(linear_approximation) ) return std::numeric_limits<float>::infinity();

    if ( !isStraightLine(linear_approximation, LINEAR_LIMIT) ) return std::numeric_limits<float>::infinity();

    slope = getSlope( linear_approximation );

    // Get normal angle from slope...
    angle = RAD2DEG(acos( slope ));

    return angle;
}



int turnRobotBy( Team1::Robot& robot, double angle ) {
    try {
        robot.rotateClockwiseBy( ROTATIONAL_VELOCITY, angle );
    }
    catch (BumperException) {
        return WALL_BUMPED;
    }
    return REACHED_TARGET;
}



int moveForwardsBy( Team1::Robot& robot, double target_distance, double wall_distance ) {
    double start_x, start_y;
    std::vector<float> laser_scan;

    // Update positions
    robot.spinOnce();

    // Store starting positions
    start_x = robot.getX();
    start_y = robot.getY();

    // Try to start forwards motion...
    try {
        if ( target_distance > 0 )
            robot.jogForwardsSafe( LINEAR_VELOCITY );
        else if ( target_distance < 0 )
            robot.jogForwardsSafe( -LINEAR_VELOCITY );
        else
            return REACHED_TARGET;
    }
    catch (BumperException) {
        return WALL_BUMPED;
    }

    // During motion... Until target_distance is reached...
    while ( robot.distanceToPoint( start_x, start_y ) < target_distance ) {
        robot.spinOnce();

        // Check bumpers
        if ( robot.getBumperAny() ) {
            robot.stopMotion();
            return WALL_BUMPED;
        }

        // Check distance in laser_scan
        laser_scan = robot.getRanges();

        // Calculated distance to wall in front
        if ( distanceToWall( robot ) <= wall_distance ) {
            robot.stopMotion();
            return WALL_IN_FRONT;
        }
    }

    // If no early stop... stop motion and return that target location was reached...
    robot.stopMotion();
    return REACHED_TARGET;
}



double getRotateAfterDistance( void ) {
    return 0.2;
}



double getRotateAfterAngle( void ) {
    return 45;
}


int wallFollow( Team1::Robot& robot, wallDirectionEnum wall_direction ) {
    // Parallelize with wall
    int wall_turn = wallParallel(robot);
    robot.spinOnce();

    // Obstacle Avoidance Algorithm
    int direction = avoidObstacles(robot, wall_direction);
    
    return direction;

}

// ***** Parallelize Robot wrt Walls ***** //
int wallParallel(Team1::Robot& robot) {

    const float MAX_DISTANCE = 0.6;
    // Laser scan data
    // const std::vector<float> laser_ranges = robot.getRanges();
    // const float right_value = laser_ranges[0];
    // const float left_value = laser_ranges[laser_ranges.size() - 1];

    // Wall angle data
    const float wall_angle = getWallAngleFromLaserScan(robot);

    if (!std::isinf(wall_angle)){
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
    } else if (std::isinf(wall_angle)) {
        ROS_INFO("Incorrect Wall Angle");
    }

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
                moveForwardsBy(robot,0.2,MAX_DISTANCE);
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




int rotateAfterBumper( Team1::Robot& robot ) {
    try {
        // If center bumper pressed... TBD
        if ( robot.getBumperCenter() ) {
            robot.moveForwards(-LINEAR_VELOCITY, getRotateAfterDistance());
        }
        // If left bumper is pressed, rotate clockwise
        else if ( robot.getBumperLeft() ) {
            robot.moveForwards(-LINEAR_VELOCITY, getRotateAfterDistance());
            robot.rotateClockwiseBy(ROTATIONAL_VELOCITY, getRotateAfterAngle());
        }
        // If right bumper is pressed, rotate counter-clockwise
        else if ( robot.getBumperRight() ) {
            robot.moveForwards(-LINEAR_VELOCITY, getRotateAfterDistance());
            robot.rotateClockwiseBy(ROTATIONAL_VELOCITY, -getRotateAfterAngle());
        }
    }
    // If any given movement fails, return WALL_BUMPED
    catch (BumperException) {
        return WALL_BUMPED;
    }

    // Otherwise return REACHED_TARGET when all movements complete
    return REACHED_TARGET;
}



bool emptyInFront( Team1::Robot& robot ) {
    // Return true if no valid values in scan, or all values more than a meter away
    std::vector<float> ranges = robot.getRanges();
    float front_mean = getMean( ranges, ranges.size() * (NUM_SCAN_SEGM / 2), ranges.size() * (NUM_SCAN_SEGM / 2) + 1 );
    if ( front_mean == std::numeric_limits<float>::infinity() )
        return false;
    return front_mean < 1.;
}



#ifndef distanceToWall
double distanceToWall( Team1::Robot& robot ) {
    std::vector<float> ranges = robot.getRanges();
    float front_mean = getMean( ranges, ranges.size() * (NUM_SCAN_SEGM / 2), ranges.size() * (NUM_SCAN_SEGM / 2) + 1 );
    return (double) front_mean;
}
#endif


#endif
