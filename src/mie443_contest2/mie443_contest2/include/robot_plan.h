#ifndef ROBOT_PLAN_H
#define ROBOT_PLAN_H

#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

// Add include for ros::ServiceClient
// Add include for tf::createQuaternionMsgFromYaw

#include "robot_pose.h"

#define TOPIC "/move_base/NavfnROS/make_plan"

class RobotPlan {
    private:
        RobotPose robot_pose;
        ros::ServiceClient check_path;

    public:
        geometry_msgs::GetPlanResponse latest_response;

        RobotPlan( RobotPose robot_pose ); // Also setup check_path

        /**
         * get_plan will return true if a valid path to target is found
         * it will also update the latest_response field
        */
        bool get_plan( float x, float y, float phi );
}

#endif // ~ROBOT_PLAN_H
