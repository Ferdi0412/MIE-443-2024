#include "robot_plan.h"

RobotPlan::RobotPlan(RobotPose robot_pose) {
    this->robot_pose = robot_pose;
}

bool get_plan( float x, float y, float phi ) {
    nav_msgs::GetPlan srv;
    geometry_msgs::Quaternion start_phi_q = tf::createQuaternionFromYaw(robot_pose.phi),
                              goal_phi_q  = tf::createQuaternionFromYaw(phi);

    // Set start from robot_pose
    srv.request.start.pose.position.x = robot_pose.x;
    srv.request.start.pose.position.y = robot_pose.y;
    srv.request.start.pose.position.z = 0.;
    srv.request.start.orientation.x = 0.;
    srv.request.start.orientation.y = 0.;
    srv.request.start.orientation.z = start_phi_q.z;
    srv.request.start.orientation.w = start_phi_q.w;

    // Set goal from input
    srv.request.goal.pose.position.x = x;
    srv.request.goal.pose.position.y = y;
    srv.request.goal.pose.position.z = 0.;
    srv.request.goal.orientation.x = 0.;
    srv.request.goal.orientation.y = 0.;
    srv.request.goal.orientation.z = goal_phi_q.z;
    srv.request.goal.orientation.w = goal_phi_q.w;

    // Send request
    check_path.call(srv); // TODO: Ensure this updates srv with the response...

    // Expose the latest response, cause why not...
    latest_response = srv.response;

    return srv.response.plan.poses.size() > 0;
}
