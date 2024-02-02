/* Team1Robot class defined here... */
#ifndef TEAM_1_ROBOT_CPP
#define TEAM_1_ROBOT_CPP

#include "../robot.hpp"

// STD Libraries
#include <vector>
#include <cmath>

// ROS Libraries
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovatiance.h>
// #include <geometry_msgs/Pose.h>
// #include <geometry_msgs/Point.h>
// #include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TwistWithCovariance.h>
// #include <geometry_msgs/Twist.h>
// #include <geometry_msgs/Vector3.h>

#ifndef N_BUMPER
#define N_BUMPER (3)
#endif

#ifndef RAD2DEG
#define RAD2DEG(rad) ((rad) * 180. / M_PI)
#endif

#ifndef DEG2RAD
#define DEG2RAD(deg) ((deg) * M_PI / 180.)
#endif

class Team1Robot {
    private:
        ros::Subscriber bumper_sub;
        ros::Subscriber laser_sub;
        ros::Subscriber odom_sub;
        ros::Subscriber vel_sub;

        ros::Publisher  vel_pub;

        /* == ODOMETRY == */
        void odomCallback ( const nav_msgs::Odometry::ConstPtr& msg ) {
            // Update position and velocities
            _x      = msg->pose.pose.position.x;
            _y      = msg->pose.pose.position.y;
            _theta  = msg->pose.pose.orientation.z;
            _dx     = msg->twist.twist.linear.x; // Check if correct...
            _dy     = msg->twist.twist.linear.y;
            _dtheta = msg->twist.twist.angluar.z;
        }

        /* == ROBOT-VELOCITY == */
        void velCallback ( const void * msg ) {

        };

        /* == WORLD POSITION ==*/
        double _x, _y, _theta;

        /* == ROBOT VELOCITY == */
        double _dx, _dy, _dtheta, _dforward; // TODO: implement _dforward

        /* == BUMPERS == */
        bool _bumper_right, _bumper_left, _bumper_center;
        void bumperCallback( const kobuki_msgs::BumperEvent::ConstPtr& msg ) {
            switch (msg->bumper) {
                case kobuki_msgs::BumperEvent::LEFT:
                    _bumper_left = msg->state; break;

                case kobuki_msgs::BumperEvent::CENTER:
                    _bumper_center = msg->state; break;

                case kobuki_msgs::BumperEvent::RIGHT:
                    _bumper_right = msg->state; break;

                case default:
                    ROS_WARNING("[Team1Robot->bumperCallback] {msg->bumper} not recognized: %d := %d\n", msg->bumper, msg->state); break;
            }
        }

        /* == LASER-SCAN == */
        float _min_scan_dist;
        int32_t _n_lasers;
        void scanCallback( const sensor_msgs::LaserScan::ConstPtr& msg ) {
            // Update laser_scan object...
        }

    public:
        /* == WORLD POSITION == */
        double getX()     { return _x; }
        double getY()     { return _y; }
        double getTheta() { return _theta; }

        /* == ROBOT VELOCITY */
        double getVelX()       { return _dx; }
        double getVelY()       { return _dy; }
        double getVelTheta()   { return _dtheta; }
        double getVelForward() { return _dforward; }

        /* == BUMPERS == */
        bool checkBumperRight()  { return _bumper_right; }
        bool checkBumperLeft()   { return _bumper_left; }
        bool checkBumperCenter() { return _bumper_center; }
        bool checkAnyBumper()    { return (_bumper_right && _bumper_center && _bumper_left); }

        /* == LASER-SCAN == */
        // TODO: Check which fields of the sensor_msgs/LaserScan msg are important
        std::vector<double> laser_scan;

        /* == CYCLE ROS MESSAGES == */
        void spinOnceROS ( ) {
            // TODO: Check feesibility of this...
            ros::spinOnce();
        }

        /* == CONSTRUCTOR == */
        Team1Robot ( ros::NodeHandle node_handler ) {
            // Have larger queue in bumper_sub incase "simultaneous" trigger of more than 1 direction
            bumper_sub = node_handler.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
            laser_sub  = node_handler.subscribe("scan", 1, &scanCallback);
            // vel_sub    = node_handler

            vel_pub    = node_handler.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

            // Add variable initialization
        }

        /**
         * facePoint will send message to turn robot to rotate to face a point
         * BLOCKING
         *
         * @param x target x-coordinate
         * @param y target y-coordinate
        */
        void facePoint( float x, float y ) {
            // angleToPoint();
        }

        /**
         * rotateDegrees will send message to turn a desired angle
         * BLOCKING
         *
         * @param angle the angle to rotate by (clockwise)
        */
        void rotateDegrees( float angle ) {
            // jogRotate(...)
            // while ( not_at_desired_angle ) { wait... }
        }

        /**
         * jogRotate will start an unspecified duration of rotation
         * NON-BLOCKING
         *
         * @param speed the speed at which to rotate (clockwise)
        */
        void jogRotate( float speed ) {

        }

        /**
         * angleToPoint returns the angle to turn to face a point.
         *
         * @param x target x-coordinate
         * @param y target y-coordinate
        */
        float angleToPoint( float x, float y ) {

        }

        /**
         * moveForwards will travel forwards at at desired speed for a desired distance.
         * NON-BLOCKING
         *
         * @param distance distance to travel
         * @param speed    desired speed
        */
        void moveForwards( float distance, float speed ) {

        }

        /**
         * moveForwards will travel forwards a desired distance
         * BLOCKING
         *
         * @param distance distance to travel
        */
        void moveForwards( float distance ) {
            // jogForwards(...);
            // while ( not_at_destination ) { wait... }
        }

        /**
         * jogForwards will start forwards motion, and return...
         * NON-BLOCKING
         *
         * @param speed speed to travel at
        */
        void jogForwards( float speed ) {

        }

        /**
         * distanceToPoint returns the distance from current position to a point.
         *
         * @param x target x-coordinate
         * @param y target y-coordinate
        */
        float distanceToPoint( float x, float y ) {

        }

};


#endif // ~TEAM_1_ROBOT
