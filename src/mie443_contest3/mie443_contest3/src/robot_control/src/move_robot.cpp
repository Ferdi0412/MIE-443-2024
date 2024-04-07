#ifndef MOVE_ROBOT_CPP
#define MOVE_ROBOT_CPP

#include "../move_robot.h"
#include "../misc_math.h"
#include "../program_timer.h"

#include <cmath>
#include <geometry_msgs/Pose.h>

#define ROT_DEVIATION 2



/**
 * ============================
 * === SUPPORT DECLARATIONS ===
 * ============================
*/
bool linear_move(       bool is_fwd, double distance,              double speed );
// bool angular_move(      bool is_cc,  double final_orientation_rad, double speed_deg );
// bool angular_move_to_0( bool is_cc,  double speed_deg );
// bool check_passes_0(    bool is_cc,  double final_orientation_rad );


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



// bool rotate_clockwise( double angle_deg, double speed ) {
//     if ( angle_deg == 0. || speed == 0. )
//         return true;

//     bool is_cc = angle_deg < 0.;

//     ros::spinOnce();

//     double target_angle = radian_rolloff( get_odom_phi() + deg_2_radian( angle_deg ) );
//     bool   passes_0     = check_passes_0( is_cc, target_angle );

//     if ( passes_0 )
//         return angular_move_to_0( is_cc, speed ) && angular_move( is_cc, target_angle, speed );
//     else
//         return angular_move( is_cc, target_angle, speed );
// }



bool linear_move( bool is_fwd, double distance, double speed ) {
    double               x_start, y_start;
    geometry_msgs::Twist target_velocity;
    ros::Rate            loop_rate(10);
    bool                 movement_complete = false;

    // Keep an "expected_duration" - in case it gets stuck somewhere so it doesn't run forever....
    int start_time = seconds_elapsed();
    int expected_duration = 3 * fabs( distance / speed ) + 2.;

    // Update robot position
    ros::spinOnce();

    // Store starting state of robot
    x_start = get_odom_x();
    y_start = get_odom_y();
    target_velocity.linear.x = is_fwd ? fabs(speed) : -fabs(speed);

    // Iterate until one of exit conditions is met
    while ( ros::ok() && ((seconds_elapsed() - start_time) <= expected_duration) ) {
        double x_curr, y_curr;

        ros::spinOnce();

        // Get up-to-date robot values
        x_curr = get_odom_x();
        y_curr = get_odom_y();

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



/**
 * Rotate such that robot is aligned with angle_deg, but NEVER passes 0 orientation...
*/
// bool angular_move( bool is_cc, double final_orientation_rad, double speed_deg ) {
//     double phi_start;
//     geometry_msgs::Twist target_velocity;
//     ros::Rate            loop_rate(10);
//     bool                 movement_complete = false;

//     // Handling values...
//     double speed = radian_rolloff(fabs(deg_2_radian(speed)));
//     speed *= is_cc ? 1 : -1;
//     final_orientation_rad = radian_rolloff(final_orientation_rad)

//     // Angle buffer
//     double buffer = fabs(speed) / 5;

//     // Update robot orientation
//     ros::spinOnce();

//     // Store orientation
//     phi_start = get_odom_phi();
//     target_velocity.angular.z = speed;

//     // Keep an expected duration - in case it gets stuck somewhere, so it doesn't run forever...
//     int start_time = seconds_elapsed();
//     int expected_duration = 3 * fabs(final_orientation_rad - phi_start) + 2.;

//     while ( ros::ok() && ((seconds_elapsed() - start_time) <= expected_duration)) {
//         double phi_curr;

//         ros::spinOnce();

//         phi_curr = get_odom_phi();

//         if ( is_cc && ( phi_curr + buffer > final_orientation_rad ) ) {
//             movement_complete = true;
//             break;
//         }
//         else if ( !is_cc && ( phi_curr - buffer < final_orientation_rad ) ) {
//             movement_complete = true;
//             break;
//         }

//         publish_velocity( target_velocity );
//         loop_rate.sleep();
//     }

//     publish_velocity( empty_twist() );
//     ros::spinOnce();
//     return movement_complete();
// }



// /**
//  * Rotate such that robot is aligned with 0 orientation...
// */
// bool angular_move_to_0( bool is_cc, double speed_deg ) {

// }


#endif // ~ MOVE_ROBOT_CPP
