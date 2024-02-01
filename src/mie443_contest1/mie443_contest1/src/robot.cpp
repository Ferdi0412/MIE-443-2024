/* Team1Robot class defined here... */
#ifndef TEAM_1_ROBOT
#define TEAM_1_ROBOT

#include <vector>

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
        }

        /* == ROBOT-VELOCITY == */
        void _velCallback ( const void * msg ) { };

        /* == WORLD POSITION ==*/
        double _x, _y, _theta;

        /* == ROBOT VELOCITY == */
        double _dx, _dy, _dtheta, _dforward;

        /* == BUMPERS == */
        bool _bumper_right, _bumper_left, _bumper_center;
        void bumperCallback( const kobuki_msgs::BumperEvent::ConstPtr& msg ) {
            // stuff...
        }

        /* == LASER-SCAN == */
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
            bumper_sub = node_handler.subscribe("mobile_base/events/bumper", 1, &bumperCallback);
            laser_sub  = node_handler.subscribe("scan", 1, &scanCallback);
            // vel_sub    = node_handler

            vel_pub    = node_handler.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
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
