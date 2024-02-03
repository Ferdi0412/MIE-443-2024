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

namespace MotionExceptions {

    /**
     * BumperException is thrown in certain motion methods, when a bumper is pressed in the direction of travel
    */
    class BumperException : public std::exception {};
}

namespace Team1 {
    class Robot {
        private:
            ros::Subscriber bumper_sub, laser_sub, odom_sub; // vel_sub?
            ros::Publisher  vel_pub; // To move robot

            double _pos_x, _pos_y, _pos_theta;
            double _vel_x, _vel_y, _vel_theta;
            bool   _bumper_right, _bumper_left, _bumper_center;

            void (*_spin_once)(void);
            ros::Rate _read_rate;

            /**
             * odomCallback runs on the /odom topic subscription, updating the values in the robot object to the current state
             *
             * @param msg the incoming message
            */
            void odomCallback( const nav_msgs::Odometry::ConstPtr& msg ) {
                _pos_x =     msg->pose.pose.position.x;
                _pos_y =     msg->pose.pose.position.y;
                _pos_theta = msg->pose.pose.orientation.z;

                _vel_x =     msg->twist.twist.linear.x;
                _vel_y =     msg->twist.twist.linear.y;
                _vel_theta = msg->twist.twist.angular.z;
            }

            /**
             * bumperCallback runs on the /mobile_base/events/bumper topic, updating to reflect state of bumpers
            */
            void bumperCallback( const kobuki_msgs::BumperEvent::ConstPtr& msg ) {
                switch (msg->bumper) {
                    case kobuki_msgs::BumperEvent::LEFT:
                        _bumper_left = msg->state; break;

                    case kobuki_msgs::BumperEvent::CENTER:
                        _bumper_center = msg->state; break;

                    case kobuki_msgs::BumperEvent::RIGHT:
                        _bumper_right = msg->state; break;

                    default:
                        // OS_WARNING("[Team1Robot->bumperCallback] {msg->bumper} not recognized: %d := %d\n", msg->bumper, msg->state);
                        break;
                }
            }

        public:
            /**
             * "getter" functions
            */
            double getX() { return _pos_x; }
            double getY() { return _pos_y; }
            double getTheta() { return _pos_theta; }

            double getVelX() { return _vel_x; }
            double getVelY() { return _vel_y; }
            double getVelTheta() { return _vel_theta; }

            bool getBumperLeft() { return _bumper_left; }
            bool getBumperCenter() { return _bumper_center; }
            bool getBumperRight() { return _bumper_right; }
            bool getBumperAny() { return _bumper_left || _bumper_center || _bumper_right; }

            /**
             * spinOnceROS will update subscriptions and values in object...
            */
            void spinOnceROS( void ) { _spin_once(); }

            /**
             * sleepROS will sleep for the rest of the read_interval
            */
            void sleepROS( void ) { _read_rate.sleep(); }

            /**
             * stopMotion will stop the last motion command published
            */
            void stopMotion( void ) {
                geometry_msgs::Twist stop_motion;
                stop_motion.angular.z = 0;
                stop_motion.linear.x  = 0;
                vel_pub.publish(stop_motion);
                spinOnceROS();
            }

            /**
             * jogForwards will start moving the robot forwards by a given speed for an indefinite distance
             *
             * @param velocity linear velocity [try find units...]
            */
            void jogForwards( double velocity ) {
                geometry_msgs::Twist fwd_motion;
                fwd_motion.angular.z = 0;
                fwd_motion.linear.x  = velocity;
                vel_pub.publish(fwd_motion);
                spinOnceROS();
            }

            /**
             * jogForwardsSafe will first check if any of the bumpers are pressed
             *
             * @param velocity linear velocity [try find units..]
             *
             * @throws MotionExceptions::BumperException when going to move forwards, but a bumper is triggered
            */
            void jogForwardsSafe( double velocity ) {
                if ( (velocity > 0) && getBumperAny() )
                    throw MotionExceptions::BumperException();
                jogForwards( velocity );
            }

            /**
             * jogClockwise will start rotating the robot clockwise by a given speed for an indefinite angle of rotation
             *
             * @param velocity angular velocity [try find units...]
            */
            void jogClockwise( double velocity ) {
                geometry_msgs::Twist clock_motion;
                clock_motion.angular.z = velocity;
                clock_motion.linear.x  = 0;
                vel_pub.publish(clock_motion);
                spinOnceROS();
            }

            /**
             * jogClockwiseSafe will first check if the bumper in the direction of rotation is pressed
             *
             * @param velocity angular velocity [try find units...]
             *
             * @throws MotionExceptions::BumperException when bumper on side of rotation is pressed
            */
            void jogClockwiseSafe( double velocity ) {
                if ( (velocity > 0) && getBumperRight() )
                    throw MotionExceptions::BumperException();
                else if ( (velocity < 0) && getBumperLeft() )
                    throw MotionExceptions::BumperException();
                jogClockwise( velocity );
            }

            /**
             * Robot constructor
             *
             * @param node_handler object used to configure subscriptions/publish topics for ROS
             * @param read_interval a Rate object used to define read intervals in BLOCKING calls
             * @param spin_once_function function that takes no parameters, used to update subscriptions in BLOCKING calls
            */
            Robot( ros::NodeHandle node_handler, ros::Rate& read_rate_object, void (*spin_once_function)(void) ) : _read_rate(read_rate_object) {
                bumper_sub = node_handler.subscribe(BUMPER_TOPIC, 10, &Team1::Robot::bumperCallback, this);
                // laser_sub  = node_handler.subscribe(LASER_TOPIC, 1, &laserCallback);
                odom_sub   = node_handler.subscribe(ODOM_TOPIC, 1, &Team1::Robot::odomCallback, this);
                vel_pub    = node_handler.advertise<geometry_msgs::Twist>(VEL_COMMAND_TOPIC, 1);
                _spin_once = spin_once_function;
            };
    };
}
