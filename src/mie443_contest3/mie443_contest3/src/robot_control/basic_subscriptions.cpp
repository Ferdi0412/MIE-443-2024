#include <array>

#include <cmath>

#include <ros/ros.h>

#include <rosgraph_msgs/Clock.h>

#include <geometry_msgs/Quaternion.h>
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/WheelDropEvent.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>

#include <tf/transform_datatypes.h>

// #include <geometry_msgs/Twist.h>

/**
 * ============
 * === MISC ===
*/
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

double rad_2_degree( double radian_angle ) {
    return radian_angle * 180. / M_PI;
}

double deg_2_radian( double degree_angle ) {
    return degree_angle * M_PI / 180.;
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

void odom_callback( const nav_msgs::Odometry::ConstPtr& msg ) {

    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    z = msg->pose.pose.position.z;

    pitch_yaw_roll = get_pitch_yaw_roll(msg->pose.pose.orientation);
    phi            = tf::getYaw(msg->pose.pose.orientation);
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

/**
 * =============
 * === CLOCK ===
 *
 * I am using this stuff to try get the "live" time - so if Gazebo runs slower, this also runs slower...
*/
ros::Subscriber clock_subscription;
bool            clock_subscribed;

ros::Time start_time;
ros::Time clock_time;

ros::Time get_curr_time( ){
    return ros::Time::now();
}

void set_start_time( ros::Time start_time_set ) {
    start_time = start_time_set;
}


void clock_callback( const rosgraph_msgs::Clock::ConstPtr& msg ) {
    clock_time = msg->clock;
}

void subscribe_to_clock( ros::NodeHandle& node_handler ) {
    if ( !clock_subscribed ) {
        clock_subscription = node_handler.subscribe( "clock", 1, &clock_callback );
        clock_subscribed   = true;
    }
}

bool wait_for_clock_msg( ros::NodeHandle& node_handler, double timeout ) {
    rosgraph_msgs::Clock::ConstPtr msg = ros::topic::waitForMessage<rosgraph_msgs::Clock>( "clock", node_handler, ros::Duration(timeout));

    if ( msg != nullptr )
        clock_callback( msg );

    return msg != nullptr;
}

ros::Time get_clock_time( ) {
    return clock_time;
}

ros::Duration get_clock_duration_from_start( ) {
    return clock_time - start_time;
}

double get_clock_seconds_from_start( ) {
    return get_clock_duration_from_start().toSec();
}


/**
 * ================
 * === FOLLOWER ===
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



void target_far_callback( const std_msgs::Bool::ConstPtr& msg ) {
    target_far = msg->data;
}

bool wait_for_target_far_msg( ros::NodeHandle& node_handler, double timeout ) {
    std_msgs::Bool::ConstPtr msg = ros::topic::waitForMessage<std_msgs::Bool>( "follower/target_far", node_handler, ros::Duration(timeout));

    if ( msg != nullptr )
        target_far_callback( msg );

    return msg != nullptr;
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
    subscribe_to_clock(     node_handler );

    // Setup clock variables...
    if (wait_for_clock_msg( node_handler, 3. ))
        set_start_time( get_clock_time() );
    else
        set_start_time( get_curr_time() );
}

void initialize_follower_subscriptions( ros::NodeHandle& node_handler ) {
    initialize_follower_target_subscriptions( node_handler );
}
