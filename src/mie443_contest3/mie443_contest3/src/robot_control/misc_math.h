#ifndef MISC_MATH_H
#define MISC_MATH_H

#include <array>

#include <geometry_msgs/Quaternion.h>

/**
 * ==================
 * === ROTATIONAL ===
 * ==================
*/

/**
 * rad_2_degree - converts angle in radians to degrees
*/
double rad_2_degree( double radian_angle );



/**
 * deg_2_radian - converts angle in degrees to radians
*/
double deg_2_radian( double degree_angle );



/**
 * get_pitch_yaw_roll - returns {pitch, yaw, roll}
*/
std::array<double, 3> get_pitch_yaw_roll( const geometry_msgs::Quaternion& input_quaternion );



/**
 * ==============
 * === LINEAR ===
 * ==============
*/

/**
 * get_hypotenuse - returns length of hypotenuse
*/
double get_hypotenuse( double x, double y );



/**
 * distance_from_point - returns distance between two points
*/
double distance_from_point( double x_from, double y_from, double x_to, double y_to );



/**
 * ======================
 * === IMPLEMENTATION ===
 * ======================
*/

#include "src/misc_math.cpp"



#endif // ~ MISC_MATH_H
