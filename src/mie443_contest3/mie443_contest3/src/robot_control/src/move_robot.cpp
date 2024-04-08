#ifndef MOVE_ROBOT_CPP
#define MOVE_ROBOT_CPP

#include "../move_robot.h"
#include "../robot.cpp"

/**
 * ============================
 * === SUPPORT DECLARATIONS ===
 * ============================
*/
Team1::Robot* robot = nullptr;

void initialize_move_robot( ros::NodeHandle node_handler ) {
    if ( robot == nullptr )
        robot = new Team1::Robot( node_handler, 10 );
}


Team1::Robot& get_robot( ) {
    if ( robot == nullptr ) {
        std::cout << "=== ROBOT NOT INITIALIZED ===\n";
        exit(-1);
    }

    return *robot;
}


/**
 * ======================
 * === IMPLEMENTATION ===
 * ======================
*/

bool move_forwards( double distance, double speed ) {
    try {
        speed = distance > 0 ? fabs(speed) : -fabs(speed);
        get_robot().moveForwards( speed, fabs(distance) );
    } catch ( BumperException& exc ) {
        return false;
    }
    return true;
}

bool rotate_clockwise( double angle_deg, double speed ) {
    try {
        get_robot().rotateClockwiseBy( speed, angle_deg );
    } catch ( BumperException& exc ) {
        return false;
    }
    return true;
}



#endif // MOVE_ROBOT_CPP
