#ifndef BASIC_FOLLOWER_SUBSCRIPTIONS_H // Import guard
#define BASIC_FOLLOWER_SUBSCRIPTIONS_H

#include <ros/ros.h>

/**
 * ====================
 * === INITIALIZERS ===
*/

/**
 * initialize_robot_subscriptions
 * This MUST be run to use anything from the robot
 * This includes all functions except the target_far and target_found related functions
*/
void initialize_robot_subscriptions( ros::NodeHandle& node_handler );

/**
 * initialize_follower_subscriptions
 * This MUST be run to use any target_far or target_found related functions
*/
void initialize_follower_subscriptions( ros::NodeHandle& node_handler );

/**
 * ===============
 * === BUMPERS ===
*/

/**
 * get_bumper_left AND get_bumper_right AND get_bumper_center
 * Each of the above check their respective bumpers
 *
 * true ==> Bumper is triggered
 * false ==> Bumper is NOT triggered
*/
bool get_bumper_left();
bool get_bumper_right();
bool get_bumper_center();

/**
 * check_bumpers - Will return `true` if any bumper is triggered
*/
bool check_bumpers();

/**
 * wait_for_bumper_msg - Will return `true` if the topic was received within the given timeout
 * @param node_handler
 * @param timeout - Maximum number of seconds to wait
*/
bool wait_for_bumper_msg( ros::NodeHandle node_handler, double timeout );



/**
 * =================
 * === WHEELDROP ===
*/

/**
 * check_raised - Will return `true` if either wheel is off the ground
*/
bool check_raised();

/**
 * wait_for_wheeldrop_msg - Will return `true` if the topic was received within the given timeout
 * @param node_handler
 * @param timeout - Maximum number of seconds to wait
*/
bool wait_for_wheeldrop_msg( ros::NodeHandle node_handler, double timeout );



/**
 * ================
 * === ODOMETRY ===
*/

/**
 * There are fetch methods for each of the following:
 * @param x
 * @param y
 * @param z     - the height
 * @param phi   - the rotation on the floor (imagine tilting it in the air)
 * @param pitch - Pitch of the robot
 * @param yaw   - Same as `phi`
 * @param roll  - Roll of the robot
*/
double get_odom_x();
double get_odom_y();
double get_odom_z();
double get_odom_phi();
double get_odom_pitch();
double get_odom_yaw();
double get_odom_roll();

/**
 * wait_for_odom_msg - Will return `true` if the topic was received within the given timeout
 * @param node_handler
 * @param timeout - Maximum number of seconds to wait
*/
bool wait_for_odom_msg( ros::NodeHandle node_handler, double timeout );



/**
 * ================
 * === FOLLOWER ===
*/

/**
 * get_target_far - Returns `true` if the target is too far away to follow
*/
bool get_target_far();

/**
 * get_target_found - Returns `true` if the target is found - `false` if the target was lost
*/
bool get_target_found();

/**
 * get_target_to_follow - Returns `true if the target is found and within following distance
*/
bool get_target_to_follow();






#include "./basic_subscriptions.cpp"

#endif // ~ BASIC_FOLLOWER_SUBSCRIPTIONS_H
