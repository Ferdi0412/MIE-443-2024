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

class RobotPlan {
    private:
        // static std::string
        RobotPose& robot_pose;
        ros::ServiceClient check_path;

    public:
        nav_msgs::GetPlanResponse latest_response;

        RobotPlan( ros::NodeHandle& nh, RobotPose& robot_pose_ ); // Also setup check_path

        /**
         * get_plan will return true if a valid path to target is found
         * it will also update the latest_response field
        */
        bool get_plan( float x, float y, float phi );
};

#endif // ~ROBOT_PLAN_H
