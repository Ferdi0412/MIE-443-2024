#ifndef BASIC_FOLLOWER_PUBLISHERS_CPP
#define BASIC_FOLLOWER_PUBLISHERS_CPP

#include "../basic_subscriptions.h"

#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>

#include <tf/transform_datatypes.h>

/**
 * ==========================
 * === WORKING WITH TWIST ===
*/
geometry_msgs::Twist increase_velocity( const geometry_msgs::Twist& reference, double d_vel_x, double d_vel_phi ) {
    geometry_msgs::Twist new_twist = reference;
    new_twist.linear.x  += d_vel_x;
    new_twist.angular.z += d_vel_phi;
    return new_twist;
}


geometry_msgs::Twist empty_twist( ) {
    return geometry_msgs::Twist();
}

/**
 * ===========================
 * === PUBLISHING COMMANDS ===
*/
static ros::Publisher velocity_publisher;
static bool           velocity_pub_ready = false;

void setup_velocity_publisher( ros::NodeHandle node_handler ) {
    if ( !velocity_pub_ready ) {
        velocity_publisher = node_handler.advertise<geometry_msgs::Twist>( "cmd_vel_mux/input/teleop", 1 );
        velocity_pub_ready = true;
    }
}



void publish_velocity( geometry_msgs::Twist msg ) {
    velocity_publisher.publish(msg);
}



void publish_lin_velocity( double vel_x ) {
    geometry_msgs::Twist msg;
    msg.linear.x = vel_x;
    publish_velocity(msg);
}



void publish_velocity( double vel_x, double vel_phi ) {
    geometry_msgs::Twist msg;
    msg.linear.x  = vel_x;
    msg.angular.z = vel_phi;
    publish_velocity(msg);
}



/**
 * ====================
 * === INITIALIZERS ===
*/
void initialize_basic_movers( ros::NodeHandle& node_handler ) {
    setup_velocity_publisher( node_handler );
}



#endif // ~ BASIC_FOLLOWER_PUBLISHERS_CPP
