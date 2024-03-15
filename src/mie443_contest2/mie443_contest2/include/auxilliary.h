/**
 * Here are some auxilliary functions intended to make building the main program easier.
*/
#ifndef AUXILLIARY_H
#define AUXILLIARY_H

#include <chrono>

#include <robot_pose.h>

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
class SimplePose {
    public:
        float x;
        float y;
        float phi;
    
    public:
        SimplePose();
        SimplePose( float x, float y, float phi );
        SimplePose( float x, float y, geometry_msgs::Quaternion orientation );
        SimplePose( geometry_msgs::Pose pose );
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

#endif
