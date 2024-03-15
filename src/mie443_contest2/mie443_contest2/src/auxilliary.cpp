#include <auxilliary.h>

#include <cmath>
#include <iostream>

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



/**
 * ========================
 * === BOXES NAVIGATION ===
*/
RobotPlan* robot_planner = NULL;
static std::vector<std::vector<float>> boxes_positions;
static std::vector<bool>               boxes_found;

void initialize_boxes_navigation( ros::NodeHandle& nh, const Boxes& boxes, RobotPose& robot_pose ) {
    if ( robot_planner != NULL )
        delete robot_planner; // Prevent memory leak...
    robot_planner   = new RobotPlan( nh, robot_pose );

    boxes_positions = boxes.coords;
    boxes_found     = std::vector<bool>(false, boxes_positions.size());
}

void mark_as_found( size_t box_index, bool found ) {
    if ( box_index > boxes_found.size() ) {
        std::cout << "boxes_positions is out-of-bounds!!!\n";
        return;
    }

    boxes_found[box_index] = found;
}

bool has_been_found( size_t box_index, bool found ) {
    if ( box_index > boxes_found.size() ) {
        std::cout << "boxes_positions is out-of-bounds!!!\n";
        return false;
    }

    return boxes_found[box_index];
}

SimplePose location_facing_box( size_t box_index, float distance_from, float delta_phi ) {
    if ( box_index > boxes_positions.size() ) {
        std::cout << "boxes_positions is out-of-bounds!!!\n";
        return SimplePose();
    }

    // Get location and orientation of target
    float x = boxes_positions[box_index][0],
          y = boxes_positions[box_index][1],
          phi = boxes_positions[box_index][2];
    
    // Get location facing the same direction as the target...
    SimplePose facing_target = distance_from_pose(SimplePose(x, y, phi), fabs(distance_from), delta_phi);

    // Flip direction to be facing the actual target
    return flip_orientation(facing_target);
}

bool check_for_plan( SimplePose some_position ) {
    if ( robot_planner == NULL ) {
        std::cout << "RobotPlanner not initialized!!!\nYou MUST run initialize_boxes_navigation for this function to work!\n";
        exit(-1);
    }

    // Check if plan can actually be made
    return robot_planner->get_plan(some_position.x, some_position.y, some_position.phi);
}