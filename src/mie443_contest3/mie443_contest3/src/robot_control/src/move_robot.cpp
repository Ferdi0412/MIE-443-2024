#ifndef MOVE_ROBOT_CPP
#define MOVE_ROBOT_CPP

#include "../move_robot.h"
#include "../misc_math.h"
#include "../program_timer.h"

#include <cmath>
#include <geometry_msgs/Pose.h>

#define LIN_VEL_DEVIATION 0.1



/**
 * ============================
 * === SUPPORT DECLARATIONS ===
 * ============================
*/
bool linear_move( bool is_fwd, double distance, double speed );
bool angular_move( bool is_cc, double angle_deg, double speed );


/**
 * ======================
 * === IMPLEMENTATION ===
 * ======================
*/

bool move_forwards( double distance, double speed ) {
    if ( distance == 0. || speed == 0. )
        return true;

    if ( distance > 0.  )
        return linear_move( true, fabs(distance), fabs(speed) );
    else
        return linear_move( false, fabs(distance), fabs(speed) );
}



bool rotate_clockwise( double angle_deg, double speed ) {
    // if ( angle_deg == 0. )
    //     return true;

    // return angular_move( (angle_deg < 0.), deg_2_radian(fabs(angle_deg)), deg_2_radian(fabs(speed)) );
}



bool linear_move( bool is_fwd, double distance, double speed ) {
    double               x_start, y_start;
    geometry_msgs::Twist target_velocity;
    double               prev_velocity;
    ros::Rate            loop_rate(10);
    bool                 movement_complete = false;
    int start_time = seconds_elapsed();
    int expected_duration = 3 * fabs( distance / speed ) + 2.;

    // Update robot values
    ros::spinOnce();

    // Store starting state of robot
    x_start = get_odom_x();
    y_start = get_odom_y();
    prev_velocity = get_odom_lin_velocity(); // Add some way to check whether it is in the right direction or not
    target_velocity.linear.x = is_fwd ? fabs(speed) : -fabs(speed);

    // Iterate until one of exit conditions is met
    while ( ros::ok() && ((seconds_elapsed()) - start_time) <= expected_duration ) {
        double x_curr, y_curr;
        double curr_velocity;

        ros::spinOnce();

        // Get up-to-date robot values
        x_curr = get_odom_x();
        y_curr = get_odom_y();
        curr_velocity = get_odom_lin_velocity();

        // Check if target distance reached - No check for direction...
        if ( distance_from_point( x_start, y_start, x_curr, y_curr ) >= distance ) {
            movement_complete = true;
            break;
        }

        // If robot bumps into something in front of it...
        if ( is_fwd && check_bumpers() )
            break;

        publish_velocity( target_velocity );

        loop_rate.sleep();
    }

    publish_velocity( empty_twist() );
    ros::spinOnce();
    return movement_complete;
}



bool angular_move( bool is_cc, double angle_deg, double speed ) {
    // double phi_curr;
    // double curr_velocity;

    // ros::spin
}



#endif // ~ MOVE_ROBOT_CPP
