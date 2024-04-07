#ifndef MOVE_ROBOT_H
#define MOVE_ROBOT_H

/**
 * move_forwards - moves robot forwards by a set amount at a given speed
*/
bool move_forwards( double distance, double speed = 0.2 );

/**
 * rotate_clockwise - rotates robot clockwise on the spot
*/
bool rotate_clockwise( double angle_deg, double speed = 30);

/**
 * rotate_follow - rotates robot clockwise whilst following the follower node output
 * Eg. for use in "shaking" behaviour
*/


/**
 * ======================
 * === IMPLEMENTATION ===
 * ======================
*/

#include "src/move_robot.cpp"



#endif // ~ MOVE_ROBOT_H
