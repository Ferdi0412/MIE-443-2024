/**
 * Here are some auxilliary functions intended to make building the main program easier.
 *
 * The important stuff
 * 1. TIMER - This section contains some functions for use in tracking program time.
 * 2. BOXES NAVIGATION - This section contains some functions for use in getting to each box.
 *
 * Example code (assuming main, boxes, robotPose and some n -> node handler - are all setup):
 * ...
 * initialize_boxes_navigation( n, boxes, robotPose );
 * mainTimerStart();
 *
 * // Iterate for up to 300 seconds (5 minutes) or until all boxes found
 *  while ( (mainTimerSecondsElapsed() < 300) && (!all_found()) ) {
 *      size_t     box_index;
 *      SimplePose box_position;
 *      bool       path_to_box = false;
 *
 *      for ( size_t i = 0; i < boxes.size(); i++ ) {
 *          // Only go through boxes that haven't been found yet...
 *          if ( has_been_found(size_t) )
 *              continue;
 *
 *          // Get the position of the box
 *          box_position = location_facing_box( i ); // Can adjust 2 optional params - distance_from, delta_phi (angle_offset)
 *
 *          // Ensure there exists a valid path from your location to the box
 *          if ( check_for_plan(box_position) ) {
 *              box_index   = i;
 *              path_to_box = true;
 *              break;
 *          }
 *      }
 *
 *      if ( !path_to_box )
 *          std::cout << "No valid boxes with any valid paths were found!!!\n";
 *
 *      /* === Feature Detection... === * /
 *      ... using box_index - the physical box you are travelling to ...
 *
 *      // If box found...
 *      mark_as_found(box_index);
 *  }
 *
*/
#ifndef AUXILLIARY_H
#define AUXILLIARY_H

#include <chrono>
#include <vector>

#include <robot_pose.h>
#include <boxes.h>
#include <robot_plan.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>



/**
 * =============
 * === TIMER ===
 *
 * The following functions should make controlling the time the program runs for...
*/
typedef std::chrono::time_point<std::chrono::system_clock> aux_timer_t;

/**
 * setMainStartTime must run once at the start of the program, for the other TIME functions to work
*/
void mainTimerStart( void );

/**
 * getMainSecondsElapsed will return the number of seconds since setMainStartTime was run
 *
 * @returns <uint64_t> the number of seconds
*/
uint64_t mainTimerSecondsElapsed( void );

/**
 * getTimer will return a timer
 *
 * @returns <aux_timer_t> a timer
*/
aux_timer_t getTimer( void );

/**
 * getSecondsElapsed will return the number of seconds since getTimer was run
 *
 * @returns <uint64_t> the number of seconds
*/
uint64_t getSecondsElapsed( aux_timer_t input_timer );

/**
 * getMillisecondsElapsed will return the number of milliseconds since getTimer was run
 *
 * @returns <uint64_t> the number of milliseconds
*/
uint64_t getMillisecondsElapsed( aux_timer_t input_timer );



/**
 * ================
 * === POSITION ===
*/
/**
 * SimplePose - Just a wrapper around x, y and phi (angle), to make passing these in and out of functions a little simpler...
*/
class SimplePose {
    public:
        float x;
        float y;
        float phi;

    public:
        // If no variables are passed, x y and phi are set to 0
        SimplePose();

        // If x y and phi are given, the values given are used
        SimplePose( float x, float y, float phi );

        // If a Quaternion orientation is passed, it will be translated into phi and set here using the tf::getYaw(...) method as done in tutorials
        SimplePose( float x, float y, geometry_msgs::Quaternion orientation );

        // If a pose is given, the x y and phi are extracted from it for easier access
        SimplePose( geometry_msgs::Pose pose );

        // To enable the x y and phi to be retrieved from this class (IDK why, I used it a bit in testing)...
        SimplePose( RobotPose robot_pose );
};

/**
 * distance_from_pose - @returns the pose that aligns with the target, with an orientation
 *                       defined by the target plus delta_phi, at a given distance behind that pose
*/
SimplePose distance_from_pose( SimplePose target, float distance, float delta_phi = 0. );

/**
 * flip_orientation - rotates the orientation by 180 degrees/pi radians
*/
float flip_orientation( float phi_radians );
SimplePose flip_orientation( SimplePose pose_to_flip );



/**
 * ========================
 * === BOXES NAVIGATION ===
*/
/**
 * initialize_boxes_navigation - MUST BE RUN FIRST... Sets up the other functions used below....
*/
void initialize_boxes_navigation( ros::NodeHandle& nh, const Boxes& boxes, RobotPose& robot_pose );

/**
 * mark_as_found - indicate that a given box has been found...
 * TODO: Add some form of "store-position" function to go along with this...
*/
void mark_as_found( size_t box_index, int template_id );

/**
 * has_been_found - returns true only if mark_as_found has been run on this box before...
*/
bool has_been_found( size_t box_index );

/**
 * all_found - returns true if ALL boxes have been found
*/
bool all_found( );

/**
 * location_facing_box - returns a SimplePose representing to travel to in order to face a targeted box
*/
SimplePose location_facing_box( size_t box_index, float distance_from = 0.3, float delta_phi = 0 );

/**
 * check_for_plan - returns true if a valid path can be found to some_position...
*/
bool check_for_plan( SimplePose some_position );

/**
 * get_box_ids returns the vector of template_ids as saved using mark_as_found(index, id)
*/
std::vector<int>& get_box_ids();

/**
 * get_box_id returns the template_id of the given box_index
*/
int get_box_id( size_t box_index );

#endif // ~ AUXILLIARY_H
