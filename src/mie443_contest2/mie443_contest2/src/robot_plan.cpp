#include "robot_plan.h"
#include <tf/transform_datatypes.h>

RobotPlan::RobotPlan(const RobotPose* robot_pose) {
    this->robot_pose = robot_pose;
}

bool RobotPlan::get_plan( float x, float y, float phi ) {
    nav_msgs::GetPlan srv;
    geometry_msgs::Quaternion start_phi_q = tf::createQuaternionMsgFromYaw(robot_pose->phi),
                              goal_phi_q  = tf::createQuaternionMsgFromYaw(phi);

    // Set start from robot_pose
    srv.request.start.pose.position.x = robot_pose->x;
    srv.request.start.pose.position.y = robot_pose->y;
    srv.request.start.pose.position.z = 0.;
    srv.request.start.pose.orientation.x = 0.;
    srv.request.start.pose.orientation.y = 0.;
    srv.request.start.pose.orientation.z = start_phi_q.z;
    srv.request.start.pose.orientation.w = start_phi_q.w;

    // Set goal from input
    srv.request.goal.pose.position.x = x;
    srv.request.goal.pose.position.y = y;
    srv.request.goal.pose.position.z = 0.;
    srv.request.goal.pose.orientation.x = 0.;
    srv.request.goal.pose.orientation.y = 0.;
    srv.request.goal.pose.orientation.z = goal_phi_q.z;
    srv.request.goal.pose.orientation.w = goal_phi_q.w;

    // Send request
    check_path.call(srv); // TODO: Ensure this updates srv with the response...

    // Expose the latest response, cause why not...
    latest_response = srv.response;

    return srv.response.plan.poses.size() > 0;
}
