#ifndef ROBOT_PLAN_H
#define ROBOT_PLAN_H

#include <ros/ros.h>
#include <ros/service_client.h>

#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

// Add include for ros::ServiceClient
// Add include for tf::createQuaternionMsgFromYaw

#include "robot_pose.h"

#define ROBOT_PLAN_TOPIC "/move_base/NavfnROS/make_plan"
#define CLEAR_COSTMAPS_TOPIC "/move_base/clear_costmaps"
#define POSE_ESTIMATE_TOPIC "/initialpose"

class RobotPlan {
    private:
        // static std::string
        RobotPose&         robot_pose;
        ros::ServiceClient check_path;
        ros::ServiceClient movebase_clear;
        ros::Publisher     pose_estimate;

    public:
        nav_msgs::GetPlanResponse latest_response;

        RobotPlan( ros::NodeHandle& nh, RobotPose& robot_pose_ ); // Also setup check_path

        /**
         * get_plan will return true if a valid path to target is found
         * it will also update the latest_response field
        */
        bool get_plan( float x, float y, float phi );

        /**
         * clear_costmaps will return true if the costmap was successfully cleared
        */
        bool clear_costmaps( );

        /**
         * set_pose_estimate will set a new pose estimate
        */
        bool set_pose_estimate( float x, float y, float phi );

        /**
         * get_robot_pose returns a reference to robot_pose
        */
        RobotPose& get_robot_pose( );
};

#endif // ~ROBOT_PLAN_H
