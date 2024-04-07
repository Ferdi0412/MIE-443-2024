/**
 * This file uses re-creations of the functions from contest 1 using contest 3 functions...
*/

#ifndef ROBOT_MOVE_H
#define ROBOT_MOVE_H

/**
 * @param angle_deg [degree]        - for counter-clock, use LESS THAN 0
 * @param speed     [degree/second] - must be GREATER THAN 0
*/
bool rotate_clockwise_2( double angle_deg, double speed = 30 );


/**
 * ======================
 * === IMPLEMENTATION ===
*/
#include "src/robot_move.cpp"

#endif // ~ ROBOT_MOVE_H
