/**
 * Provides Team1::Robot class
 *
 * Implements methods to control the kobuki robot base... See documentation at:
 * http://wiki.ros.org/kobuki/Tutorials/Kobuki%27s%20Control%20System#How_it_works
*/
#ifndef ROBOT_V2_CPP
#define ROBOT_V2_CPP

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <kobuki_msgs/BumperEvent.h>
#include <ros/console.h>
#include "ros/ros.h"

#include <exception>
#include <stdexcept>
#include <cmath>
#include <math.h>
// #include <stdio.h>

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

/**
 * BumperException is thrown when bumper is pressed when travelling.
*/
class BumperException : std::exception {};

namespace Team1 {
    class Robot {
        private:
            /* === PRIVATE VARIABLES === */
            ros::Subscriber bumper_sub, laser_sub, odom_sub, vel_sub;
            ros::Publisher  vel_pub;

            double pos_x, pos_y, pos_theta;
            double vel_x, vel_y, vel_theta;

            double vel_fwd_act, vel_clock_act; // Internal state from robot
            geometry_msgs::Twist motion_set;   // Set/command to send to robot

            bool   bumper_right, bumper_left, bumper_center;

            // void (*spin_once_ros)(void); // Try use ros::spinOnce(); instead of a callback
            ros::Rate spin_rate;

            /* === SUBSCRIPTION CALLBACKS === */
            /**
             * odomCallback runs per cycle on the /odom topic
             *
             * This topic displays the robot's position and velocity relative to it's "global" coordinates.
             *
             * @param msg the incoming Odometry message
            */
            void odomCallback( const nav_msgs::Odometry::ConstPtr& msg ) {
                pos_x     = msg->pose.pose.position.x;
                pos_y     = msg->pose.pose.position.y;
                pos_theta = tf::getYaw(msg->pose.pose.orientation);

                vel_x     = msg->twist.twist.linear.x;
                vel_y     = msg->twist.twist.linear.y;
                vel_theta = msg->twist.twist.angular.z;
            }

            /**
             * velocityCallback to be set on the /mobile_base/commands/velocity
             *
             * This topic displays what velocity the robot (should) be going with respect to it's base (ie. fwd/clockwise).
             *
             * @param msg Twist object from the topic
            */
            void velocityCallback( const geometry_msgs::Twist::ConstPtr& msg ) {
                vel_fwd_act   = msg->linear.x;
                vel_clock_act = msg->angular.z;
            }

            /**
             * bumperCallback runs on the /mobile_base/events/bumper topic, updating to reflect state of bumpers
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
             * getAngleBetween computes the angle between two orientations
             *
             * @param theta1 initial orientation [rad]
             * @param theta2 target orientation  [rad]
             * @returns angle between            [rad]
            */
            double getAngleBetween( double theta1, double theta2 ) {
                return fmod(theta2 - theta1, 2 * M_PI);
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
             * setMotion will set and publish the velocity components relative to the turtlebot base
             *
             * @param fwd_speed   is the linear speed (in m/s) to travel forwards
             * @param clock_speed is the angular speed (in rad/s) to travel clockwise
            */
            void setMotion( double fwd_speed, double clock_speed ) {
                motion_set.linear.x  = fwd_speed;
                motion_set.angular.z = clock_speed;
                publishVelocity();
            }

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
                setMotion( velocity, 0 );
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
                setMotion( velocity, 0 );
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
                double start_x, start_y;
                spinOnce(); // Update the current x and y coordinates
                start_x = pos_x;
                start_y = pos_y;
                if ( distance < 0 ) throw std::invalid_argument("[Robot::moveForwards] distance must be greater than distance!\n");
                jogForwardsSafe( velocity );
                while ( getEuclideanDistance(start_x, start_y, pos_x, pos_y) < distance ) {
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
             * @param velocity rotational velocity [rad/s]
            */
            void jogClockwise( double velocity ) {
                setMotion( 0, velocity );
            }

            /**
             * jogClockwiseSafe will start a clockwise rotation, and check if bumpers in that direction are triggered
             *
             * @param velocity rotational velocity [rad/s]
             * @throws BumperException
            */
            void jogClockwiseSafe( double velocity ) {
                if ( getBumperCenter() )                  throw BumperException();
                if ( (velocity > 0) && getBumperRight() ) throw BumperException();
                if ( (velocity < 0) && getBumperLeft() )  throw BumperException();
                // If no problems with bumpers...
                setMotion( 0, velocity );
            }

            /**
             * rotateClockwiseTo will rotate the robot clockwise (or counter-clockwise) until it reaches a given angle
             * BLOCKING -> will not return until rotation has completed
             *
             * @param velocity angular velocity [rad/s]
             * @param angle    target angle     [rad]
             * @throws BumperException
            */
            void rotateClockwiseTo( double velocity, double angle ) {
                // To be implemented...
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
                // laser_sub  = node_handler.subscribe(LASER_TOPIC, 1, &laserCallback);
                odom_sub   = node_handler.subscribe(ODOM_TOPIC, 1, &Team1::Robot::odomCallback, this);
                // Configure publisher
                vel_pub    = node_handler.advertise<geometry_msgs::Twist>(VEL_COMMAND_TOPIC, 1);
            }
    };
}

#endif
