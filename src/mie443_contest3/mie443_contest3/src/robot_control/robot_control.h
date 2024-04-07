// Includes all the robot_control files...
#include <ros/ros.h>

#include "program_timer.h"
#include "move_robot.h"
#include "basic_subscriptions.h"
#include "basic_publishers.h"

void initialize_all( ros::NodeHandle node_handler ) {
    initialize_move_robot(             node_handler );
    initialize_robot_subscriptions(    node_handler );
    initialize_follower_subscriptions( node_handler );
    initialize_basic_movers(           node_handler );
}