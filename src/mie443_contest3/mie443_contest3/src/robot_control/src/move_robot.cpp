#ifndef MOVE_ROBOT_CPP
#define MOVE_ROBOT_CPP

#include "../move_robot.h"
<<<<<<< HEAD
#include "../misc_math.h"

#include <cmath>
#include <geometry_msgs/Pose.h>

#define LIN_VEL_DEVIATION 0.1
#define ROT_VEL_DEVIATION 5

=======
#include "../robot.cpp"
>>>>>>> 8107ab2434f6a90cf9ef2275867c3a54dd4da6f2

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
<<<<<<< HEAD
    if ( angle_deg == 0. )
        return true;

    if (angle_deg > 0.)
        return angular_move( true, deg_2_radian(fabs(angle_deg)), deg_2_radian(fabs(speed)) );
    else    
        return angular_move( false, deg_2_radian(fabs(angle_deg)), deg_2_radian(fabs(speed)) );
}



bool linear_move( bool is_fwd, double distance, double speed ) {
    double               x_start, y_start;
    geometry_msgs::Twist target_velocity;
    double               prev_velocity;
    ros::Rate            loop_rate(10);
    bool                 movement_complete = false;

    // Update robot values
    ros::spinOnce();

    // Store starting state of robot
    x_start = get_odom_x();
    y_start = get_odom_y();
    prev_velocity = get_odom_lin_velocity(); // Add some way to check whether it is in the right direction or not
    target_velocity.linear.x = is_fwd ? fabs(speed) : -fabs(speed);

    // Iterate until one of exit conditions is met
    while ( ros::ok() ) {
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

        // If robot suddenly stops moving in the right direction OR accelerating in the right direction
        if ( is_fwd && curr_velocity < (prev_velocity - LIN_VEL_DEVIATION) )
            break;
        else if ( !is_fwd && curr_velocity > (prev_velocity - LIN_VEL_DEVIATION) )
            break;
        else
            prev_velocity = curr_velocity;

        // If robot bumps into something in front of it...
        if ( is_fwd && check_bumpers() )
            break;

        publish_velocity( target_velocity );

        loop_rate.sleep();
=======
    try {
        get_robot().rotateClockwiseBy( speed, angle_deg );
    } catch ( BumperException& exc ) {
        return false;
>>>>>>> 8107ab2434f6a90cf9ef2275867c3a54dd4da6f2
    }
    return true;
}



<<<<<<< HEAD
bool angular_move( bool is_cc, double angle_deg, double speed ) {
    double               phi_start;
    geometry_msgs::Twist target_velocity; 
    double               prev_velocity;
    ros::Rate            loop_rate(10);
    bool                 movement_complete = false;

    // Update robot values
    ros::spinOnce();

    // Store starting state of robot
    phi_start = get_odom_phi();
    prev_velocity = get_odom_rot_velocity(); // Add some way to check whether it is in the right direction or not
    target_velocity.angular.z = is_cc ? fabs(angle_deg) : -fabs(angle_deg);

    // Iterate until one of exit conditions is met
    while ( ros::ok() ) {
        double phi_curr;
        double curr_velocity;

        ros::spinOnce();

        // Get up-to-date robot values
        phi_curr = get_odom_phi();
        curr_velocity = get_odom_rot_velocity();

        // Check if target angle reached
        if (fabs(phi_curr - phi_start) >= fabs(angle_deg)) {
            movement_complete = true;
            break;
        }

        // If robot suddenly stops moving in the right direction OR accelerating in the right direction
        if ( is_cc && curr_velocity < (prev_velocity - ROT_VEL_DEVIATION) )
            break;
        else if ( !is_cc && curr_velocity > (prev_velocity - ROT_VEL_DEVIATION) )
            break;
        else
            prev_velocity = curr_velocity;

        // If robot bumps into something in front of it...
        if ( is_cc && check_bumpers() )
            break;

        publish_velocity( target_velocity );

        loop_rate.sleep();
    }

    publish_velocity( empty_twist() );
    ros::spinOnce();
    return movement_complete;

}



#endif // ~ MOVE_ROBOT_CPP
=======
#endif // MOVE_ROBOT_CPP
>>>>>>> 8107ab2434f6a90cf9ef2275867c3a54dd4da6f2
