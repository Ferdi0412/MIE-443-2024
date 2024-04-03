#ifndef BASIC_FOLLOWER_SUBSCRIPTIONS_CPP // Import guard
#define BASIC_FOLLOWER_SUBSCRIPTIONS_CPP

#include "../basic_subscriptions.h"
#include "../misc_math.h"

#include <array>

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Quaternion.h>
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/WheelDropEvent.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>

#include <tf/transform_datatypes.h>


// #include <geometry_msgs/Twist.h>

/**
 * ===============
 * === CAMERAS ===
*/
imageTransporter subscribe_to_webcam( ) {
    return imageTransporter( "camera/image/", sensor_msgs::image_encodings::BGR8 );
}

imageTransporter subscribe_to_kinect( ) {
    return imageTransporter( "camera/rgb/image_raw", sensor_msgs::image_encodings::BGR8 );
}

imageTransporter subscribe_to_depth_sensor( ) {
    return imageTransporter( "camera/depth_registered/image_raw", sensor_msgs::image_encodings::TYPE_32FC1 );
}

/**
 * ======================
 * === BUMPER SENSORS ===
*/
static ros::Subscriber bumper_subscription;
static bool            bumper_subscribed = false;

static std::array<bool, 3> bumper_states = {false, false, false};

void bumper_callback( const kobuki_msgs::BumperEvent::ConstPtr& msg ) {
    switch ( msg->bumper ) {
        case kobuki_msgs::BumperEvent::LEFT:
            bumper_states[0] = msg->state;
            break;

        case kobuki_msgs::BumperEvent::CENTER:
            bumper_states[1] = msg->state;
            break;

        case kobuki_msgs::BumperEvent::RIGHT:
            bumper_states[2] = msg->state;
            break;
    }
}

void subscribe_to_bumper( ros::NodeHandle& node_handler ) {
    if ( !bumper_subscribed ) {
        bumper_subscription = node_handler.subscribe( "mobile_base/events/bumper", 10, &bumper_callback );
        bumper_subscribed   = true;
    }
}

bool wait_for_bumper_msg( ros::NodeHandle& node_handler, double timeout ) {
    kobuki_msgs::BumperEvent::ConstPtr msg = ros::topic::waitForMessage<kobuki_msgs::BumperEvent>( "mobile_base/events/bumper", node_handler, ros::Duration(timeout));

    if ( msg != nullptr )
        bumper_callback( msg );

    return msg != nullptr;
}

bool get_bumper_left( ) {
    return bumper_states[0];
}

bool get_bumper_center( ) {
    return bumper_states[1];
}

bool get_bumper_right( ) {
    return bumper_states[2];
}

bool check_bumpers( ) {
    for ( size_t i = 0; i < bumper_states.size(); i++ ) {
        if ( bumper_states[i] )
            return true;
    }
    return false;
}

/**
 * =====================
 * === CLIFF SENSORS ===
*/
static ros::Subscriber wheeldrop_subscription;
static bool            wheeldrop_subscribed = false;

static std::array<bool, 2> wheeldrop_states = {false, false};

void wheeldrop_callback( const kobuki_msgs::WheelDropEvent::ConstPtr& msg ) {
    switch ( msg->wheel ) {
        case kobuki_msgs::WheelDropEvent::LEFT:
            wheeldrop_states[0] = (kobuki_msgs::WheelDropEvent::RAISED);
            break;

        case kobuki_msgs::WheelDropEvent::RIGHT:
            wheeldrop_states[1] = (kobuki_msgs::WheelDropEvent::RAISED);
            break;
    }
}

void subscribe_to_wheeldrop( ros::NodeHandle& node_handler ) {
    if ( !wheeldrop_subscribed ) {
        wheeldrop_subscription = node_handler.subscribe( "/mobile_base/events/wheel_drop", 10, &wheeldrop_callback );
        wheeldrop_subscribed   = true;
    }
}

bool wait_for_wheeldrop_msg( ros::NodeHandle& node_handler, double timeout ) {
    kobuki_msgs::WheelDropEvent::ConstPtr msg = ros::topic::waitForMessage<kobuki_msgs::WheelDropEvent>( "/mobile_base/events/wheel_drop", node_handler, ros::Duration(timeout));

    if ( msg != nullptr )
        wheeldrop_callback( msg );

    return msg != nullptr;
}

bool check_raised( ) {
    for ( size_t i = 0; i < bumper_states.size(); i++ ) {
        if ( bumper_states[i] )
            return true;
    }
    return false;
}

/**
 * =========================
 * === ODOM SUBSCRIPTION ===
*/
static ros::Subscriber odom_subscribtion;
static bool            odom_subscribed = false;

static double x = 0., y = 0., z = 0., phi = 0.;
static std::array<double, 3> pitch_yaw_roll = {0., 0., 0.};
geometry_msgs::Twist odom_velocity;

void odom_callback( const nav_msgs::Odometry::ConstPtr& msg ) {

    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    z = msg->pose.pose.position.z;

    pitch_yaw_roll = get_pitch_yaw_roll(msg->pose.pose.orientation);
    phi            = tf::getYaw(msg->pose.pose.orientation);

    odom_velocity  = msg->twist.twist;
}

void subscribe_to_odom( ros::NodeHandle& node_handler ) {
    if ( !odom_subscribed ) {
        odom_subscribtion = node_handler.subscribe( "odom", 1, &odom_callback );
        odom_subscribed   = true;
    }
}

bool wait_for_odom_msg( ros::NodeHandle& node_handler, double timeout ) {
    nav_msgs::Odometry::ConstPtr msg = ros::topic::waitForMessage<nav_msgs::Odometry>( "odom", node_handler, ros::Duration(timeout));

    if ( msg != nullptr )
        odom_callback( msg );

    return msg != nullptr;
}

double get_odom_x( ) {
    return x;
}

double get_odom_y( ) {
    return y;
}

double get_odom_z( ) {
    return z;
}

double get_odom_phi( ) {
    return phi;
}

double get_odom_pitch( ) {
    return pitch_yaw_roll[0];
}

double get_odom_yaw( ) {
    return pitch_yaw_roll[1];
}

double get_odom_roll( ) {
    return pitch_yaw_roll[2];
}

geometry_msgs::Twist& get_odom_velocity( ) {
    return odom_velocity;
}

double get_odom_lin_velocity( ) {
    return odom_velocity.linear.x;
}

double get_odom_rot_velocity( ) {
    return odom_velocity.angular.z;
}


/**
 * ================
 * === FOLLOWER ===
*/
static ros::Subscriber follower_subscription;
static bool            follower_subscribed;

static geometry_msgs::Twist follower_cmd;

void follower_callback( const geometry_msgs::Twist::ConstPtr& msg ) {
    follower_cmd = *msg;
}

void subscribe_to_follower( ros::NodeHandle& node_handler ) {
    if ( !follower_subscribed ) {
        follower_subscription = node_handler.subscribe( "follower_velocity_smoother/smooth_cmd_vel", 1, &follower_callback );
        follower_subscribed = true;
    }
}

bool wait_for_follower_msg( ros::NodeHandle& node_handler, double timeout ) {
    geometry_msgs::Twist::ConstPtr msg = ros::topic::waitForMessage<geometry_msgs::Twist>( "follower_velocity_smoother/smooth_cmd_vel", node_handler, ros::Duration(timeout) );

    if ( msg != nullptr )
        follower_callback( msg );

    return msg != nullptr;
}

geometry_msgs::Twist& get_follower_cmd( ) {
    return follower_cmd;
}

/**
 * =======================
 * === FOLLOWER TARGET ===
 * Ferdi - I have added some topics to the follower.cpp to publish when no person is detected or when person is too far away
*/
static ros::Subscriber target_found_subscription;
static bool            target_found_subscribed;
static ros::Subscriber target_far_subscription;
static bool            target_far_subscribed;

static bool target_far = false;
static bool target_found = false;

void target_found_callback( const std_msgs::Bool::ConstPtr& msg ) {
    target_found = msg->data;
}

bool wait_for_target_found_msg( ros::NodeHandle& node_handler, double timeout ) {
    std_msgs::Bool::ConstPtr msg = ros::topic::waitForMessage<std_msgs::Bool>( "follower/target_found", node_handler, ros::Duration(timeout));

    if ( msg != nullptr )
        target_found_callback( msg );

    return msg != nullptr;
}

bool get_target_found( ) {
    return target_found;
}


void target_far_callback( const std_msgs::Bool::ConstPtr& msg ) {
    target_far = msg->data;
}

bool wait_for_target_far_msg( ros::NodeHandle& node_handler, double timeout ) {
    std_msgs::Bool::ConstPtr msg = ros::topic::waitForMessage<std_msgs::Bool>( "follower/target_far", node_handler, ros::Duration(timeout));

    if ( msg != nullptr )
        target_far_callback( msg );

    return msg != nullptr;
}

bool get_target_far( ) {
    return target_far;
}



bool get_target_available( ) {
    return target_found && (!target_far);
}



void initialize_follower_target_subscriptions( ros::NodeHandle& node_handler ) {
    // Subscribe to the target_found and target_far topics
    if ( !target_found_subscribed ) {
        target_found_subscription = node_handler.subscribe( "follower/target_found", 1, &target_found_callback );
        target_found_subscribed   = true;
    }

    if ( !target_far_subscribed ) {
        target_far_subscription = node_handler.subscribe( "follower/target_far", 1, &target_far_callback );
        target_far_subscribed   = true;
    }
}

/**
 * =============
 * === SETUP ===
*/
void initialize_robot_subscriptions( ros::NodeHandle& node_handler ) {
    // Subscribe to each topic
    subscribe_to_bumper(    node_handler );
    subscribe_to_odom(      node_handler );
    subscribe_to_wheeldrop( node_handler );
    // subscribe_to_clock(     node_handler );

    // // Setup clock variables...
    // if (wait_for_clock_msg( node_handler, 3. ))
    //     set_start_time( get_clock_time() );
    // else
    //     set_start_time( get_curr_time() );
}

void initialize_follower_subscriptions( ros::NodeHandle& node_handler ) {
    subscribe_to_follower( node_handler );
    initialize_follower_target_subscriptions( node_handler );
}


#endif // ~ BASIC_FOLLOWER_SUBSCRIPTIONS_CPP
