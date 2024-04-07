#ifndef ROBOT_MOVE_CPP
#define ROBOT_MOVE_CPP

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "../math.h"
#include "../program_timer.h"
#include "../basic_publishers.h"
#include "../basic_subscriptions.h"

#define ROT_ANGLE_BUFFER 1

double get_curr_angle( ) {
    return clamp_angle( get_odom_phi );
}

/**
 * @param velocity     [rad/s] - Counter-clockwise positive
 * @param target_angle [rad]   - Counter-clockwise positive
*/
bool rotate_counter_clockwise_to( double velocity, double target_angle ) {
    bool          reached_target = false;
    double        initial_diff;
    long          starting_ns = nanoseconds_elapsed(),
                  ending_ns;
    ros::Duration loop_rate(10);
    geometry_msgs::Twist set_velocity;
    set_velocity.angular.z = velocity;

    if ( velocity == 0. )
        return true;

    ros::spinOnce();

    // Wait for robot to stop moving... Max of 1 second wait...
    while ( (fabs(get_odom_rot_velocity()) > 0.1) && (nanoseconds_elapsed() - starting_ns > 1000) ) {
        loop_rate.sleep();
        ros::spinOnce();
    }

    starting_ns = nanoseconds_elapsed();
    ending_ns   = starting_ns + fabs(target_angle - get_curr_angle() / velocity) + 2.;

    target_angle = clamp_angle( target_angle );
    initial_diff = target_angle - get_curr_angle();

    if ( velocity > 0 ) {
        if ( target_angle <= get_curr_angle() )
            target_angle += 360.;

        while ( ros::ok() && (nanoseconds_elapsed() < ending_ns) ) {
            if (target_angle >= get_curr_angle()) {
                reached_target = true;
                break;
            }

            if ( check_bumpers() )
                break;

            if ( (target_angle - get_curr_angle()) > (initial_diff + ROT_ANGLE_BUFFER) )
                target_angle -= 360.;

            publish_velocity( set_velocity );

            ros::spinOnce();
        }
    }

    else {
        if ( target_angle >= get_curr_angle() )
            target_angle += 360.;

        while ( ros::ok() && (nanoseconds_elapsed() < ending_ns) ) {
            if (target_angle <= get_curr_angle()) {
                reached_target = true;
                break;
            }

            if ( check_bumpers() )
                break;

            if ( (target_angle - get_curr_angle()) < (initial_diff + ROT_ANGLE_BUFFER) )
                target_angle += 360.;

            publish_velocity( set_velocity );

            ros::spinOnce();
        }
    }

    publish_velocity( empty_twist() );

    ros::spinOnce();

    return reached_target;
}




bool rotate_clockwise( double angle_deg, double speed ) {
    if ( angle_deg == 0. || speed == 0. )
        return true;

    ros::spinOnce();
    if ( angle_deg > 0 )
        speed = deg_2_radian(fabs(speed));
    else
        speed = -deg_2_radian(fabs(speed));
    return rotate_counter_clockwise_to( speed, clamp_angle(get_curr_angle() + angle_deg) );
}




#endif // ~ ROBOT_MOVE_CPP
