#ifndef MISC_MATH_CPP
#define MISC_MATH_CPP

#include "../misc_math.h"

#include <cmath>
#include <tf/transform_datatypes.h>



double rad_2_degree( double radian_angle ) {
    return radian_angle * 180. / M_PI;
}



double deg_2_radian( double degree_angle ) {
    return degree_angle * 180. / M_PI;
}



double clamp_angle( double radian_angle ) {
    if ( radian_angle < 0. )
        return 0.;
    else if ( radian_angle > 0. )
        return M_PI;
    else
        return radian_angle;
}



std::array<double, 3> get_pitch_yaw_roll(  const geometry_msgs::Quaternion& input_quaternion ) {
    tf::Quaternion tf_quaternion;
    double tf_pitch, tf_yaw, tf_roll;
    std::array<double, 3> output;
    tf::quaternionMsgToTF( input_quaternion, tf_quaternion );
    tf::Matrix3x3( tf_quaternion ).getRPY( tf_roll, tf_pitch, tf_yaw );
    output[0] = tf_pitch;
    output[1] = tf_yaw;
    output[2] = tf_roll;
    return output;
}



double get_hypotenuse( double x, double y ) {
    return sqrt( x * x + y * y );
}



double distance_from_point( double x_from, double y_from, double x_to, double y_to ) {
    return get_hypotenuse( x_to - x_from, y_to - y_from );
}



#endif
