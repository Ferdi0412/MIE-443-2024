#include <auxilliary.h>

#include <cmath>

#include <tf/transform_datatypes.h>

/**
 * =============
 * === TIMER ===
*/
static aux_timer_t main_clock;

void mainTimerStart( void ) {
    main_clock = getTimer();
}

uint64_t mainTimerSecondsElapsed( void ) {
    return getSecondsElapsed( main_clock );
}

aux_timer_t getTimer( void ) {
    return std::chrono::system_clock::now();
}

uint64_t getSecondsElapsed( aux_timer_t input_timer ) {
    return std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - input_timer).count();
}

uint64_t getMillisecondsElapsed( aux_timer_t input_timer ) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - input_timer).count();
}



/**
 * ================
 * === POSITION ===
*/
SimplePose::SimplePose() : x(0), y(0), phi(0) {}

SimplePose::SimplePose( float x, float y, float phi ) {
    this->x = x;
    this->y = y;
    this->phi = phi;
}

SimplePose::SimplePose( float x, float y, geometry_msgs::Quaternion orientation ) {
    this->x = x;
    this->y = y;
    this->phi = tf::getYaw(orientation);
}

SimplePose::SimplePose( geometry_msgs::Pose pose ) {
    this->x = pose.position.x;
    this->y = pose.position.y;
    this->phi = tf::getYaw(pose.orientation);
}

SimplePose::SimplePose( RobotPose robot_pose ) {
    this->x = robot_pose.x;
    this->y = robot_pose.y;
    this->phi = robot_pose.phi;
}

SimplePose distance_from_pose( SimplePose target, float distance, float delta_phi ) {
    float new_x, new_y, new_phi = target.phi + delta_phi;
    new_x = target.x + distance * cos(new_phi);
    new_y = target.y + distance * sin(new_phi);
    return SimplePose( new_x, new_y, new_phi );
}

float flip_orientation( float phi_radians ) {
    return phi_radians + M_PI;
}

SimplePose flip_orientation( SimplePose pose_to_flip ) {
    pose_to_flip.phi = flip_orientation(pose_to_flip.phi);
    return pose_to_flip;
}