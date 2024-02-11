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
    try
        robot.rotateClockwiseBy( ROTATIONAL_VELOCITY, angle );
    catch (BumperException)
        return WALL_BUMPED;
    return REACHED_TARGET;
}



int moveForwards( Team1::Robot& robot, double target_distance, float wall_distance ) {
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
    catch (BumperException)
        return WALL_BUMPED;

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
        if ( getDistanceToWall( laser_scan ) <= wall_distance ) {
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
    catch (BumperException)
        return WALL_BUMPED;

    // Otherwise return REACHED_TARGET when all movements complete
    return REACHED_TARGET;
}



#endif
