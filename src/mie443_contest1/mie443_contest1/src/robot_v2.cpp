/**
 * Provides Team1::Robot class
 *
 * Implements methods to control the kobuki robot base... See documentation at:
 * http://wiki.ros.org/kobuki/Tutorials/Kobuki%27s%20Control%20System#How_it_works
*/
#ifndef ROBOT_V2_CPP
#define ROBOT_V2_CPP

#include <nav_msgs/Odometry.h>
#include <kobuki_msgs/BumperEvent.h>
#include <ros/console.h>
#include "ros/ros.h"

#include <exception>
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

/**
 * BumperException is thrown when bumper is pressed when travelling.
*/
class BumperException : std::exception {};

namespace Team1 {
    class Robot {
        private:
            /* === PRIVATE VARIABLES === */
            ros::Subscriber bumper_sub, laser_sub, odom_sub, vel_sub;
            ros::Publisher  vel_sub;

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
                pos_theta = msg->pose.pose.orientation.z;

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
             * publishVelocity will publish the velocity control topic
            */
            void publishVelocity( void ) {
                vel_pub.publish( motion_set );
            }

            /**
             * spinOnceROS will update class values with any topic changes/messages (and publish topics)
            */
            void spinOnce( void ) {
                publishVelocity();
                ros::spinOnce();
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
            */
            void jogForwardsSafe( double velocity ) {
                if ( (velocity > 0) && getBumperAny() )
                    throw BumperException();
                setMotion( velocity, 0 );
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
            */
            void jogClockwiseSafe( double velocity ) {
                if ( getBumperCenter() )                  throw BumperException();
                if ( (velocity > 0) && getBumperRight() ) throw BumperException();
                if ( (velocity < 0) && getBumperLeft() )  throw BumperException();
                // If no problems with bumpers...
                setMotion( 0, velocity );
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
    }
}

#endif
