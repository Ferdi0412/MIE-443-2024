#include <auxilliary.h>

#include <navigation.h>

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
static std::vector<int>                boxes_template_ids;



void initialize_boxes_navigation( ros::NodeHandle& nh, const Boxes& boxes, RobotPose& robot_pose ) {
    if ( robot_planner != NULL )
        delete robot_planner; // Prevent memory leak...

    robot_planner   = new RobotPlan( nh, robot_pose );

    boxes_positions    = boxes.coords;
    boxes_found        = std::vector<bool>(boxes_positions.size(), false);
    boxes_template_ids = std::vector<int>(boxes_positions.size(), -1);
}



void mark_as_found( size_t box_index, int template_id ) {
    if ( box_index >= boxes_found.size() ) {
        std::cout << "boxes_positions is out-of-bounds!!!\n";
        return;
    }

    if ( box_index >= boxes_template_ids.size() ) {
        std::cout << "box_index is " << box_index << " AND should be less than " << boxes_template_ids.size() << std::endl;
    }

    bool found                    = (template_id > -1);
    boxes_found[box_index]        = found;
    boxes_template_ids[box_index] = template_id;
}



bool has_been_found( size_t box_index ) {
    if ( box_index >= boxes_found.size() ) {
        std::cout << "boxes_positions is out-of-bounds!!!\n";
        return false;
    }

    return boxes_found[box_index];
}



bool all_found( ) {
    for ( size_t i = 0; i < boxes_found.size(); i++ ) {
        if ( !boxes_found[i] )
            return false;
    }
    return true;
}



SimplePose location_facing_box( size_t box_index, float distance_from, float delta_phi ) {
    if ( box_index >= boxes_positions.size() ) {
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



std::vector<int>& get_box_ids() {
    return boxes_template_ids;
}



int get_box_id( size_t box_index ) {
    if ( box_index >= boxes_template_ids.size() ) {
        std::cout << "get_box_id is out-of-bounds!!!\n";
        return -1;
    }

    return boxes_template_ids[box_index];
}


bool is_duplicate_image( size_t box_index ) {
    if ( box_index >= boxes_template_ids.size() ) {
        std::cout << "get_box_id is out-of-bounds!!!\n";
        return -1;
    }

    int template_id = get_box_id( box_index );

    if ( template_id < 0 )
        return false;

    // Check if this is a duplicate
    for ( size_t i = 0; i < box_index; i++ ) {
        if ( boxes_template_ids[i] == template_id )
            return true;
    }

    return false;
}


float degree_2_radian( float degrees ) {
    return degrees * M_PI / 180.;
}



bool refresh_costmap( ) {
    if ( robot_planner == NULL ) {
        std::cout << "RobotPlanner not initialized!!!\nYou MUST run initialize_boxes_navigation for this function to work!\n";
        exit(-1);
    }

    // Clear the AMCL costmap
    return robot_planner->clear_costmaps( );
}



bool move_pose_estimate( float fwd_dist, float left_dist ) {
    if ( robot_planner == NULL ) {
        std::cout << "RobotPlanner not initialized!!!\nYou MUST run initialize_boxes_navigation for this function to work!\n";
        exit(-1);
    }

    SimplePose curr_pose(robot_planner->get_robot_pose());

    if ( (fwd_dist == 0.) && (left_dist == 0.) )
        return robot_planner->set_pose_estimate( curr_pose.x, curr_pose.y, curr_pose.phi );

    float abs_dist  = std::sqrt( fwd_dist * fwd_dist + left_dist * left_dist );
    float delta_phi;

    // Calculate angle needed to turn to get to the position at fwd_dist, left_dist from curr position and orientation
    if ( fwd_dist != 0. )
        delta_phi = std::atan( left_dist / -fwd_dist );
    // Prevent divide by 0 by hard-coding delta_phi for positive and negative left_distance
    else if ( left_dist > 0. )
        delta_phi = M_PI / 2.;
    else
        delta_phi = -M_PI / 2.;


    SimplePose new_pose = distance_from_pose( curr_pose, abs_dist, delta_phi );

    // Move the AMCL costmap by the desired amount, keeping current orientation
    return robot_planner->set_pose_estimate( new_pose.x, new_pose.y, curr_pose.phi );
}





bool move_robot_by( float fwd_dist, float left_dist ) {
    if ( robot_planner == NULL ) {
        std::cout << "RobotPlanner not initialized!!!\nYou MUST run initialize_boxes_navigation for this function to work!\n";
        exit(-1);
    }

    SimplePose curr_pose(robot_planner->get_robot_pose());

    float abs_dist  = std::sqrt( fwd_dist * fwd_dist + left_dist * left_dist );
    float delta_phi;

    // Calculate angle needed to turn to get to the position at fwd_dist, left_dist from curr position and orientation
    if ( fwd_dist > 0. )
        delta_phi = std::atan( left_dist / -fwd_dist );
    else if ( fwd_dist < 0. )
        delta_phi = std::atan( left_dist / -fwd_dist ) + M_PI;
    // Prevent divide by 0 by hard-coding delta_phi for positive and negative left_distance
    else if ( left_dist > 0. )
        delta_phi = M_PI / 2.;
    else
        delta_phi = -M_PI / 2.;

    SimplePose new_pose = distance_from_pose( curr_pose, abs_dist, delta_phi );;

    // Check if plan can actually be made
    if ( !robot_planner->get_plan(new_pose.x, new_pose.y, curr_pose.phi) )
        return false;

    return Navigation::moveToGoal(new_pose.x, new_pose.y, curr_pose.phi);
}


RobotPlan& get_robot_planner( ) {
    if ( robot_planner == NULL ) {
        std::cout << "RobotPlanner not initialized!!!\nYou MUST run initialize_boxes_navigation for this function to work!\n";
        exit(-1);
    }

    return *robot_planner;
}
