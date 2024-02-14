/**
 * 1. Team1::Robot
 * |- This is a class to control the robot, with functions to process subscription messages, and publish movements.
 * |  Included is also some functions for calibrating motion, by adjusting for any biases in motion.
 * |  There is currently nothing to account for odometry drift, or random errors, only for any
 * |  deviation from expected paths observer of the robot during linear/rotational motion.
 *
 *
 * Implements methods to control the kobuki robot base... See documentation at:
 * http://wiki.ros.org/kobuki/Tutorials/Kobuki%27s%20Control%20System#How_it_works
 *
 *
 * NOTE: BumperException is thrown even on 'Timeouts' in the BLOCKING movements.
*/

/**
 * Import guard...
 * The `ifndef ROBOT_V2_CPP` prevents this file from being included more than once,
 * as this would lead to issues with compilation...
*/
#ifndef ROBOT_V2_CPP
#define ROBOT_V2_CPP

/**
 * ===============
 * === IMPORTS ===
 * ===============
 *
 * The first set are the require ROS classes/functions.
 * NOTE: I am unsure if everything here is available should a header file and CMake linkage be used in the future...
 *
 * The second set are the standard C/C++ classes and functions.
*/

// ROS classes/functions
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/console.h>
#include "ros/ros.h"

// STD functions/classes
#include <exception>
#include <stdexcept>
#include <cmath>
#include <math.h>
#include <vector>
#include <chrono>
// #include <stdio.h>
#include <iostream>


/**
 * =================
 * === CONSTANTS ===
 * =================
*/
#ifndef BUMPER_TOPIC
#define BUMPER_TOPIC "mobile_base/events/bumper"
#endif

#ifndef SCAN_TOPIC
#define SCAN_TOPIC "scan"
#endif

#ifndef ODOM_TOPIC
#define ODOM_TOPIC "odom"
#endif

#ifndef VEL_COMMAND_TOPIC
#define VEL_COMMAND_TOPIC "cmd_vel_mux/input/teleop"
#endif

#ifndef RAD2DEG
#define RAD2DEG(rad) ((rad) * 180. / M_PI)
#endif

#ifndef DEG2RAD
#define DEG2RAD(deg) ((deg) * M_PI / 180.)
#endif

// T1_ROBOT_ANGLE_BUFFER is used in checking for rootaattion "overflow"
#define T1_ROBOT_ANGLE_BUFFER 1
#define MOVE_TIME_BUFFER      7 /* number of seconds on top of predicted time to complete motion to block before throwing a BumperException */
#define ROBOT_ANGLE_VEL_BUFFER 0.1

// std::numeric_limits<float>::infinity()

/**
 * BumperException is thrown when bumper is pressed when travelling.
*/
class BumperException : std::exception {};

// For storing timestamps...
typedef std::chrono::time_point<std::chrono::system_clock> _sys_clock_t;


namespace Team1 {
    class Robot {
        private:
            /* === PRIVATE VARIABLES === */
            ros::Subscriber bumper_sub, laser_sub, odom_sub, vel_sub;
            ros::Publisher  vel_pub;
            ros::NodeHandle subscription_node_handler;

            double pos_x = 0, pos_y = 0, pos_theta = 0;
            double vel_x = 0, vel_y = 0, vel_theta = 0;

            double vel_fwd_act = 0, vel_clock_act = 0; // Internal state from robot
            geometry_msgs::Twist motion_set;   // Set/command to send to robot

            bool   bumper_right = 0, bumper_left = 0, bumper_center = 0;

            // void (*spin_once_ros)(void); // Try use ros::spinOnce(); instead of a callback
            ros::Rate spin_rate;

            int32_t n_lasers;
            float angle_min = 0, angle_max = 0, angle_increment = 0, range_min = 0, range_max = 0;
            std::vector<float> ranges; //, intensities;

            /**
             * === Robot calibration variables ===
             * ~param linear_calibration will be applied during linear motion, to adjust for any difference in power
             * from left to right wheel
            */
            double linear_calibration = 0;


            /* === SUBSCRIPTION CALLBACKS === */
            /**
             * laserCallback runs per laser update on the /scan topic
             *
             * @param msg the incoming LaserScan message
            */
            void laserCallback( const sensor_msgs::LaserScan::ConstPtr& msg ) {
                angle_increment = msg->angle_increment;
                angle_min = msg->angle_min;
                angle_max = msg->angle_max;
                range_min = msg->range_min;
                range_max = msg->range_max;

                // ranges apparently implemented as a vector...
                // std::cout << "INCOMING ranges: " << msg->ranges.size() << "\n";
                ranges.assign(msg->ranges.begin(), msg->ranges.end());
                // ranges.assign(msg->ranges, msg->ranges + n_lasers);
                // Ignore this field - constantly empty...
                // intensities.assign(msg->intensities.begin(), msg->intensities.end()); // intensities.assign(msg->intensities, msg->intensities + n_lasers);

                // Calculate number of laser points in scan
                // n_lasers = (angle_min - angle_max) / angle_increment;
                // Get n_lasers from the ranges vector
                n_lasers = ranges.size();
            }

            /**
             * odomCallback runs per cycle on the /odom topic
             *
             * This topic displays the robot's position and velocity relative to it's "global" coordinates.
             *
             * @param msg the incoming Odometry message
             *
             * NOTE: Flip pos_theta and vel_theta to make them clockwise-positive.
            */
            void odomCallback( const nav_msgs::Odometry::ConstPtr& msg ) {
                pos_x     = msg->pose.pose.position.x;
                pos_y     = msg->pose.pose.position.y;
                pos_theta = - RAD2DEG( tf::getYaw(msg->pose.pose.orientation) );

                vel_x     = msg->twist.twist.linear.x;
                vel_y     = msg->twist.twist.linear.y;
                vel_theta = - RAD2DEG( msg->twist.twist.angular.z );
            }

            /**
             * velocityCallback to be set on the /mobile_base/commands/velocity
             *
             * This topic displays what velocity the robot (should) be going with respect to it's base (ie. fwd/clockwise).
             *
             * @param msg Twist object from the topic
             *
             * NOTE: Flip vel_clock_act to make it clockwise-positive.
            */
            void velocityCallback( const geometry_msgs::Twist::ConstPtr& msg ) {
                vel_fwd_act   = msg->linear.x;
                vel_clock_act = - (msg->angular.z);
            }

            /**
             * bumperCallback runs on the /mobile_base/events/bumper topic, updating to reflect state of bumpers
             *
             * @param msg the incoming bumper message
            */
            void bumperCallback( const kobuki_msgs::BumperEvent::ConstPtr& msg ) {
                switch (msg->bumper) {
                    case kobuki_msgs::BumperEvent::LEFT:
                        bumper_left = msg->state; break;

                    case kobuki_msgs::BumperEvent::CENTER:
                        bumper_center = msg->state; break;

                    case kobuki_msgs::BumperEvent::RIGHT:
                        bumper_right = msg->state; break;

                    default:
                        break;
                }
            }

            /* === AUXILLIARY FUNCTIONS === */
            /**
             * publishVelocity will publish the velocity control topic
            */
            void publishVelocity( void ) {
                vel_pub.publish( motion_set );
            }

            /**
             * getEuclideanDistance computes the euclidean distance between two points (NOTE: no overflow checking)
             *
             * @param x1 x-coordinate of position 1
             * @param y1 y-coordinate of position 1
             * @param x2 x-coordinate of position 2
             * @param y2 y-coordinate of position 2
             * @returns  euclidean distance
            */
            double getEuclideanDistance( double x1, double y1, double x2, double y2 ) {
                double dist = pow( x2 - x1, 2 ) + pow( y2 - y1, 2 );
                return sqrt(dist);
            }

            /**
             * clampAngle applies the range restrictions [-180, 180] on the input angle
             *
             * @param angle the angle to clamp
            */
            double clampAngle( double angle ) {
                angle = fmod(angle, 360);
                if ( angle > 180. )
                    return angle - 360;
                return angle;
            }

            /**
             * unclampAngle shifts the range restrictions [-180, 180] to an easier to work with range [0, 360]
             *
             * @param angle the angle to unclamp
            */
            double unclampAngle( double angle ) {
                angle = fmod(angle, 360);
                if ( angle < 0. )
                    return angle + 360;
                return angle;
            }

            /**
             * getAngleBetween computes the (shortest) clockwise-positive angle between two orientations
             *
             * @param theta1 initial orientation [deg]
             * @param theta2 target orientation  [deg]
             * @returns angle between            [deg]
            */
            double getAngleBetween( double theta1, double theta2 ) {
                double diff_theta = clampAngle(theta2) - clampAngle(theta1);
                if ( diff_theta > 180 )
                    return -360 + diff_theta;
                else if ( diff_theta < -180 )
                    return 360 + diff_theta;
                return diff_theta;
            }

            /**
             * getAngleToFromNormal returns the angle of a point, from normal
             * NOTE: Normal is currently defined with the x-axis defined as orientation 0
             *
             * @param delta_x the x-offset
             * @param delta_y the y-offset
            */
            double getAngleToFromNormal( double delta_x, double delta_y ) {
                return atan2( delta_y, delta_x );
            }

            /**
             * rotateClockwiseToPrivate BLOCKING
             *
             * @param velocity     [deg/s]
             * @param target_angle [deg]   must be greater than pos_theta, add 360 to it if necessary
            */
            void rotateClockwiseToPrivate( double velocity, double target_angle ) {
                double initial_difference, expected_duration;
                _sys_clock_t start_time;
                stopMotion();
                spinOnce();
                if ( getVelTheta() < ROBOT_ANGLE_VEL_BUFFER )
                    ROS_INFO("[Robot.rotatingClockwise] -> waiting to stop motion...\n");
                while ( getVelTheta() < ROBOT_ANGLE_VEL_BUFFER ) {
                    spinOnce();
                    sleepOnce();
                }
                ROS_INFO("[Robot.rotatingClockwise]\n");
                target_angle = clampAngle(target_angle);
                if ( pos_theta >= target_angle ) target_angle += 360.;
                if ( velocity == 0 )             throw BumperException(); // Throw something that is caught for Contest 1 TODO: Improve
                initial_difference = target_angle - pos_theta;
                expected_duration  = fabs((target_angle - pos_theta) / velocity) + MOVE_TIME_BUFFER;
                start_time = getTimeNow();
                jogClockwiseSafe( fabs(velocity) );
                while ( target_angle > pos_theta ) {
                    if ( secondsSince( start_time ) > expected_duration ) {
                        ROS_ERROR("[Robot.rotateClockwiseXxx] -> timeout!\n");
                        stopMotion();
                        throw BumperException();
                    }
                    checkBumpers();
                    spinOnce();
                    // If difference between current and target has increased, assume angle rotated past 180. degrees
                    if ( (target_angle - pos_theta) > (initial_difference + T1_ROBOT_ANGLE_BUFFER) )
                        target_angle -= 360.;
                }
                stopMotion();
            }

            /**
             * rotateCounterClockwiseToPrivate BLOCKING
             *
             * @param velocity     [deg/s]
             * @param target_angle [deg]   must be less than pos_theta, subtract 360 from it if necessary
            */
            void rotateCounterClockwiseToPrivate( double velocity, double target_angle ) {

                double initial_difference, expected_duration;
                _sys_clock_t start_time;
                stopMotion();
                spinOnce();
                if ( getVelTheta() > ROBOT_ANGLE_VEL_BUFFER )
                    ROS_INFO("[Robot.rotatingCounterClockwise] -> waiting to stop motion...\n");
                while ( getVelTheta() > ROBOT_ANGLE_VEL_BUFFER ) {
                    spinOnce();
                    sleepOnce();
                }
                ROS_INFO("[Robot.rotatingCounterClockwise]\n");
                target_angle = clampAngle(target_angle);
                if ( pos_theta <= target_angle ) target_angle -= 360.;
                if ( velocity == 0 )             throw BumperException(); // Throw something that is caught for Contest 1 TODO: Improve
                initial_difference = target_angle - pos_theta;
                expected_duration  = fabs((target_angle - pos_theta) / velocity) + MOVE_TIME_BUFFER;
                start_time = getTimeNow();
                jogClockwiseSafe( -fabs(velocity) );
                while ( target_angle < pos_theta ) {
                    if ( secondsSince( start_time ) > expected_duration ) {
                        ROS_ERROR("[Robot.rotateClockwiseXxx] -> timeout!\n");
                        stopMotion();
                        throw BumperException();
                    }
                    checkBumpers();
                    spinOnce();
                    // If difference between current and target has decreased, assume angle rotated past -180. degrees
                    if ( (target_angle - pos_theta) < (initial_difference - T1_ROBOT_ANGLE_BUFFER) )
                        target_angle += 360.;
                }
                stopMotion();
            }

            /** === MOTION CONTROL === */
            /**
             * setMotion will set and publish the velocity components relative to the turtlebot base
             *
             * @param fwd_speed   is the linear speed (in m/s) to travel forwards
             * @param clock_speed is the angular speed (in rad/s) to travel clockwise
            */
            void setMotion( double fwd_speed, double clock_speed ) {
                motion_set.linear.x  = fwd_speed;
                motion_set.angular.z = -clock_speed; // Adjust for wrong direction of rotation...
                publishVelocity();
            }


            /**
             * === TIME CONTROL ===
            */

           /**
            * getTimeNow will return the 'system' formatted timestamp
            *
            * @returns _sys_clock_t variable representing the current clock/timestamp
           */
            _sys_clock_t getTimeNow( void ) {
                return std::chrono::system_clock::now();
            }

            /**
             * secondsSince will return the number of seconds since the given time
             *
             * @param timestamp the time to calculate seconds elapsed from
            */
            unsigned long long secondsSince( _sys_clock_t timestamp ) {
                return std::chrono::duration_cast<std::chrono::seconds>(getTimeNow() - timestamp).count();
            }

            /**
             * nanosecondsSince will return the number of nanoseconds since the given time
             *
             * @param timestamp the time to calculate elapsed nanoseconds from
            */
            unsigned long long nanosecondsSince( _sys_clock_t timestamp ) {
                return std::chrono::duration_cast<std::chrono::nanoseconds>(getTimeNow() - timestamp).count();
            }




        public:
            /* === PUBLIC GETTERS === */
            double getX()     { return pos_x; }
            double getY()     { return pos_y; }
            double getTheta() { return pos_theta; }

            double getVelX()     { return vel_x; }
            double getVelY()     { return vel_y; }
            double getVelTheta() { return vel_theta; }

            double getVelFwdAct()   { return vel_fwd_act; }
            double getVelClockAct() { return vel_clock_act; }

            double getVelFwdSet()   { return motion_set.linear.x; }
            double getVelClockSet() { return motion_set.angular.z; }

            bool getBumperLeft()   { return bumper_left; }
            bool getBumperCenter() { return bumper_center; }
            bool getBumperRight()  { return bumper_right; }
            bool getBumperAny()    { return bumper_left || bumper_center || bumper_right; }

            float getAngleMin()       { return RAD2DEG(angle_min); }
            float getAngleMax()       { return RAD2DEG(angle_max); }
            float getAngleIncrement() { return RAD2DEG(angle_increment); }

            float getRangeMin()       { return RAD2DEG(range_min); }
            float getRangeMax()       { return RAD2DEG(range_max); }

            uint32_t getNLasers() { return n_lasers; }

            const std::vector<float>& getRanges()      { return ranges; }

            /* === PUBLIC SETTERS === */
            /**
             * calibrateLinearMotion
             *
             * Add a clockwise rotation speed to adjust for any mismatch in power between left and right wheels.
             * Will be applied as a factor to the linear speed to set the rotational speed for motion.
             *
             * @param clockwise_offset  [(deg/s) / (m/s)]
            */
            void calibrateLinearMotion( double clockwise_offset ) { linear_calibration = DEG2RAD(clockwise_offset); }

            /* === LASER METHODS === */
            /**
             * getDesiredLaserCount
             *
             * @param desired_angle for which to calculate laser points required
             * @returns number of lasers needed to get said angle
            */
            uint32_t getDesiredNLasers( float desired_angle ) {
                return desired_angle / getAngleIncrement();
            }

            /* === ROS METHODS === */
            /**
             * spinOnceROS will update class values with any topic changes/messages (and publish topics)
            */
            void spinOnce( void ) {
                publishVelocity();
                ros::spinOnce();
            }

            /**
             * sleepOnce will sleep for the rest of a "cycle" -> defined by the spin_rate
            */
            void sleepOnce( void ) {
                spin_rate.sleep();
            }

            /**
             * sleepFor will sleep for a given duration
             *
             * @param duration [s]
            */
            void sleepFor( double duration ) {
                ros::Duration(duration).sleep();
            }

            /* === MOTION CONTROL === */
            /**
             * stopMotion will stop all motion of the robot
            */
            void stopMotion( void ) {
                setMotion(0, 0);
            }

            /**
             * jogForwards will start a forwards motion
             *
             * @param velocity linear velocity [m/s]
            */
            void jogForwards( double velocity ) {
                setMotion( velocity, linear_calibration * velocity );
            }

            /**
             * jogForwardsSafe will start a forwards motion, and check if bumpers are triggered in the forwards direction
             *
             * @param velocity linear velocity [m/s]
             * @throws BumperException
            */
            void jogForwardsSafe( double velocity ) {
                if ( (velocity > 0) && getBumperAny() )
                    throw BumperException();
                setMotion( velocity, linear_calibration * velocity );
            }

            /**
             * moveForwards will move the robot forwards, and check if bumpers are triggered during the motion
             * BLOCKING -> will not return until movement has completed
             *
             * @param velocity linear velocity [m/s]         -> Negative to move backwards
             * @param distance linear distance to travel [m] -> MUST ALWAYS BE POSITIVE
             * @throws BumperException       -> If a bumper is triggered
             * @throws std::invalid_argument -> If the distance is less than 0
            */
            void moveForwards( double velocity, double distance ) {
                double start_x, start_y, expected_duration;
                _sys_clock_t start_time;
                if ( (velocity == 0) || (distance == 0) ) return;
                spinOnce(); // Update the current x and y coordinates
                start_x = pos_x;
                start_y = pos_y;
                if ( distance < 0 ) throw std::invalid_argument("[Robot::moveForwards] distance must be greater than zero!\n");
                expected_duration = 3 * fabs(distance / velocity) + MOVE_TIME_BUFFER; // More likely to actually fail on rotation
                start_time        = getTimeNow();
                jogForwardsSafe( velocity );
                while ( getEuclideanDistance(start_x, start_y, pos_x, pos_y) < distance ) {
                    if ( secondsSince(start_time) > expected_duration ) {
                        ROS_ERROR("[Robot.moveForwards] -> timeout!\n");
                        stopMotion();
                        throw BumperException();
                    }
                    checkBumpers(); // Stop and throw BumperException if bumpers triggered
                    spinOnce();     // Update pos_x and pos_y
                }
                stopMotion();
            }

            /**
             * distanceToPoint will return the euclidean distance from the current position to a point (NOTE: No overflow handling)
             *
             * @param target_x is the target x-coordinate
             * @param target_y is the target y-coordinate
            */
            double distanceToPoint( double target_x, double target_y ) {
                return getEuclideanDistance( pos_x, pos_y, target_x, target_y );
            }

            /**
             * jogClockwise will start a clockwise rotation, and check if bumpers in that direction are triggered
             *
             * @param velocity rotational velocity [deg/s]
            */
            void jogClockwise( double velocity ) {
                setMotion( 0, DEG2RAD(velocity) );
            }

            /**
             * jogClockwiseSafe will start a clockwise rotation, and check if bumpers in that direction are triggered
             *DEG2RAD
             * @param velocity rotational velocity [deg/s]
             * @throws BumperException
            */
            void jogClockwiseSafe( double velocity ) {
                if ( getBumperCenter() )                  throw BumperException();
                if ( (velocity > 0) && getBumperRight() ) throw BumperException();
                if ( (velocity < 0) && getBumperLeft() )  throw BumperException();
                // If no problems with bumpers...
                setMotion( 0, DEG2RAD(velocity) );
            }

            /**
             * rotateClockwiseTo will rotate the robot clockwise (or counter-clockwise) until it reaches a given angle
             * BLOCKING -> will not return until rotation has completed
             *
             * @param velocity     angular velocity (velocity <0 for counter-clockwise) [deg/s]
             * @param target_angle target angle                                         [deg]
             * @throws BumperException
             * @throws std::invalid_argument -> something has gone wrong internally - let Ferdi know
            */
            void rotateClockwiseTo( double velocity, double target_angle ) {
                target_angle = clampAngle(target_angle);
                if ( (velocity == 0) || (pos_theta == target_angle) ) return;
                else if ( (velocity > 0) && (pos_theta > target_angle) ) rotateClockwiseToPrivate( velocity, target_angle + 360 );
                else if ( (velocity > 0) && (pos_theta < target_angle) ) rotateClockwiseToPrivate( velocity, target_angle );
                else if ( (velocity < 0) && (pos_theta > target_angle) ) rotateCounterClockwiseToPrivate( velocity, target_angle );
                else if ( (velocity < 0) && (pos_theta < target_angle) ) rotateCounterClockwiseToPrivate( velocity, target_angle - 360 );
            }

            /**
             * rotateClockwiseBy will rotate the robot clockwise (or counter-clockwise) by a given angle
             * BLOCKING -> it will not return until rotation has been completed
             *
             * @param velocity angular velocity (always > 0)                     [deg/s]
             * @param angle    angle to rotate  (negative for counter-clockwise) [deg]
             * @throws BumperException
            */
            void rotateClockwiseBy( double velocity, double angle ) {
                spinOnce(); // Update pos_theta
                if ( angle == 0 ) return;
                if ( angle > 0 )
                    rotateClockwiseTo( fabs(velocity), pos_theta + angle );
                else
                    rotateClockwiseTo( -fabs(velocity), pos_theta + angle );
            }

            /**
             * getAngleTo will calculate smallest clockwise-positive angle to rotate to align the robot with a given orientation
             *
             * @param target_orientation the target angle
            */
            double getAngleTo( double target_orientation ) {
                return getAngleBetween( pos_theta, target_orientation );
            }

            /**
             * getAngleToPoint will calculate smallest clockwise-positive angle to rotate to face a point
             * NOTE: assumes that pos_theta := 0 when the robot is aligned with the x-axis
             * NOTE: No overflow handling is done...
             *
             * @param target_x target x-coordinate
             * @param target_y target y-coordinate
            */
            double getAngleToPoint( double target_x, double target_y ) {
                return getAngleToRelativePoint( (target_x - pos_x), (target_y - pos_y) );
            }

            /**
             * getAngleToRelativePoint will calculate the smallest clockwise-positive to rotate to face a point at dist_x and dist_y from the current position
             * NOTE: No overflow handling is done...
             *
             * @param dist_x distance along x-axis to target
             * @param dist_y distance along y-axis to target
            */
            double getAngleToRelativePoint( double dist_x, double dist_y ) {
                return getAngleBetween(pos_theta, getAngleToFromNormal( dist_x, dist_y ));
            }

            /**
             * makeClockwise will translate an angle such that it is greater than 0
             *
             * @param angle   in range [-180, 180]
             * @returns angle in range [0, 360]
            */
            double makeClockwise( double angle ) {
                if ( angle < 0 )
                    return 360 + angle;
                return angle;
            }

            /**
             * makeCounterClockwise will translate an angle such that it is less than 0
             *
             * @param angle   in range [-180, 180]
             * @returns angle in range [-360, 0]
            */
            double makeCounterClockwise( double angle ) {
                if ( angle > 0 )
                    return -360 + angle;
                return angle;
            }

            /**
             * getAngleToLaserPoint returns the angle from the center of the laser scan to a laser point in the scane
             * NOTE: No overflow handling implemented...
             *
             * @param  laserpoint_index  index in vector of the laserpoint to face towards
             * @throws std::out_of_range when laserpoint_index is not in the vector
            */
            double getAngleToLaserPoint( unsigned int laserpoint_index ) {
                if ( laserpoint_index >= ranges.size() ) throw std::out_of_range("[Robot::getAngleToLaserPoint] -> The laserpoint_index cannot be larger than the number of laser scan points.");
                return getAngleMin() + (laserpoint_index * getAngleIncrement());
            }

            /**
             * checkBumpers will read the bumper states, and stop motion if any are triggered
             *
             * @throws BumperException
            */
            void checkBumpers( void ) {
                if ( ((getVelFwdSet() > 0) && getBumperAny()) || ((getVelClockSet() > 0) && getBumperRight()) || ((getVelClockSet() < 0) && getBumperLeft()) ) {
                    setMotion(0, 0);
                    throw BumperException();
                }
            }

            /**
             * waitOnLaserRanges will BLOCK until a valid laser scan is retrieved
            */
            void waitOnLaserRanges( void ) {
                if ( ranges.size() > 0 ) return;
                // Wait for a message, and run laserCallback on it...
                laserCallback(ros::topic::waitForMessage<sensor_msgs::LaserScan>(SCAN_TOPIC, subscription_node_handler));
            }

            /* === CONSTRUCTORS/DESTRUCTORS === */
            /**
             * Robot class constructor
             *
             * @param node_handler   object used to configure subscriptions/publish topics for ROS
             * @param spin_frequency the rate for use in BLOCKING calls (to be added...) [Hz]
            */
            Robot( ros::NodeHandle node_handler, double spin_frequency ) : spin_rate(spin_frequency) {
                // Configure subscribers
                bumper_sub = node_handler.subscribe(BUMPER_TOPIC, 10, &Team1::Robot::bumperCallback, this);
                laser_sub  = node_handler.subscribe(SCAN_TOPIC, 1, &Team1::Robot::laserCallback, this);
                odom_sub   = node_handler.subscribe(ODOM_TOPIC, 1, &Team1::Robot::odomCallback, this);

                // Configure publisher
                vel_pub    = node_handler.advertise<geometry_msgs::Twist>(VEL_COMMAND_TOPIC, 1);

                // Store node handler to be able to wait on a topic
                subscription_node_handler = node_handler;
            }
    };
}

#endif
