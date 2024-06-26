#ifndef BASIC_FOLLOWER_PUBLISHERS_H
#define BASIC_FOLLOWER_PUBLISHERS_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

/**
 * =============================================
 * === FUNCTIONS FOR WORKING WITH TWIST TYPE ===
 */

/**
 * increase_velocity - Copies all values from reference, and adds to the x component and phi (orientation-related) component
 * @param reference  - reference object/velocities
 * @param d_vel_x    - additional x-component [m/s]
 * @param d_vel_phi  - additional phi-component [rad/s]
 */
geometry_msgs::Twist increase_velocity(const geometry_msgs::Twist& reference, double d_vel_x, double d_vel_phi);

/**
 * empty_twist - returns a twist representing no motion
 */
geometry_msgs::Twist empty_twist();



/**
 * =============
 * === SETUP ===
 * Before any following functions, the initialize_basic_movers(...) must be run
 */

void initialize_basic_movers(ros::NodeHandle& node_handler);



/**
 * ==============
 * === MOVING ===
 */

void publish_velocity(geometry_msgs::Twist msg);
void publish_lin_velocity(double vel_x);
void publish_velocity(double vel_x, double vel_phi);



/**
 * ======================
 * === IMPLEMENTATION ===
 * ======================
*/

#include "src/basic_publishers.cpp"



#endif // ~ BASIC_FOLLOWER_PUBLISHERS_H
