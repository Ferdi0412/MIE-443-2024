#ifndef FERDI_FUNCTIONS_CPP
#define FERDI_FUNCTIONS_CPP


#include "assumed_functions.hpp"
#include "lin_approx.hpp"
#include "lin_approx.cpp"

#include <ros/console.h>
#include "ros/ros.h"

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
#define NUM_SCAN_SEGM 9
#endif


bool wallInFront( Team1::Robot& robot ) {
    lin_approx_t linear_approximation;
    linear_approximation = linearApproximation( robot.getRanges(), robot.getRanges().size() * (NUM_SCAN_SEGM / 2) / NUM_SCAN_SEGM, robot.getRanges().size() * (NUM_SCAN_SEGM / 2 + 1) / NUM_SCAN_SEGM);
    if ( checkApproximationError( linear_approximation) ) return false;
    ROS_INFO("MSE on scan: %.2f\n", getMeanSquaredError(linear_approximation));
    return isStraightLine(linear_approximation, 0.00005);
}



float getWallAngleFromLaserScan( Team1::Robot& robot ) {
    lin_approx_t linear_approximation;
    std::vector<float> laser_ranges;
    float slope, angle;

    laser_ranges = robot.getRanges();

    linear_approximation = linearApproximation( robot.getRanges(), robot.getRanges().size() * (NUM_SCAN_SEGM / 2) / NUM_SCAN_SEGM, robot.getRanges().size() * (NUM_SCAN_SEGM / 2 + 1) / NUM_SCAN_SEGM);

    if ( checkApproximationError(linear_approximation) ) return std::numeric_limits<float>::infinity();

    if ( !isStraightLine(linear_approximation, LINEAR_LIMIT) ) return std::numeric_limits<float>::infinity();

    slope = getSlope( linear_approximation );

    // Get normal angle from slope...
    angle = RAD2DEG(atan( slope ));
    ROS_INFO("Calculated angle: %.2f\n", angle);

    return angle;
}



float getWallAngleFromLaserScanNonStraight( Team1::Robot& robot ) {
    lin_approx_t linear_approximation;
    std::vector<float> laser_ranges;
    float slope, angle;

    laser_ranges = robot.getRanges();

    linear_approximation = linearApproximation( laser_ranges, laser_ranges.size() * 2 / 5, laser_ranges.size() * 3 / 5 );

    if ( checkApproximationError(linear_approximation) ) return std::numeric_limits<float>::infinity();

    // if ( !isStraightLine(linear_approximation, LINEAR_LIMIT) ) return std::numeric_limits<float>::infinity();

    slope = getSlope( linear_approximation );

    // Get normal angle from slope...
    angle = RAD2DEG(atan( slope ));
    ROS_INFO("Calculated angle: %.2f\n", angle);

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



int moveForwardsBy( Team1::Robot& robot, double target_distance, float wall_distance ) {
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



// int rotateAfterBumper( Team1::Robot& robot ) {
//     try {
//         // If center bumper pressed... TBD
//         if ( robot.getBumperCenter() ) {
//             robot.moveForwards(-LINEAR_VELOCITY, getRotateAfterDistance());
//         }
//         // If left bumper is pressed, rotate clockwise
//         else if ( robot.getBumperLeft() ) {
//             robot.moveForwards(-LINEAR_VELOCITY, getRotateAfterDistance());
//             robot.rotateClockwiseBy(ROTATIONAL_VELOCITY, getRotateAfterAngle());
//         }
//         // If right bumper is pressed, rotate counter-clockwise
//         else if ( robot.getBumperRight() ) {
//             robot.moveForwards(-LINEAR_VELOCITY, getRotateAfterDistance());
//             robot.rotateClockwiseBy(ROTATIONAL_VELOCITY, -getRotateAfterAngle());
//         }
//     }
//     // If any given movement fails, return WALL_BUMPED
//     catch (BumperException) {
//         return WALL_BUMPED;
//     }

//     // Otherwise return REACHED_TARGET when all movements complete
//     return REACHED_TARGET;
// }



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
