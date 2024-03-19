#include <iostream>

#include "robot_plan.h"

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <opencv2/core.hpp>
#include <tf/transform_datatypes.h>

RobotPlan::RobotPlan(ros::NodeHandle& nh, RobotPose& robot_pose_) : robot_pose(robot_pose_) {
    // this->robot_pose = robot_pose;
    check_path     = nh.serviceClient<nav_msgs::GetPlan>(ROBOT_PLAN_TOPIC);
    movebase_clear = nh.serviceClient<std_srvs::Empty>(CLEAR_COSTMAPS_TOPIC);
    pose_estimate  = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(POSE_ESTIMATE_TOPIC, 1);
}



// https://www.lloydbrombach.com/post/robot-path-planning-a-guide-to-the-ros-getplan-service-in-c-and-python
bool RobotPlan::get_plan( float x, float y, float phi ) {
    nav_msgs::GetPlan srv;
    geometry_msgs::Quaternion start_phi_q = tf::createQuaternionMsgFromYaw(robot_pose.phi),
                              goal_phi_q  = tf::createQuaternionMsgFromYaw(phi);

    // Set start from robot_pose
    srv.request.start.header.frame_id = "map";
    srv.request.start.pose.position.x = robot_pose.x;
    srv.request.start.pose.position.y = robot_pose.y;
    srv.request.start.pose.position.z = 0.;
    srv.request.start.pose.orientation.x = 0.;
    srv.request.start.pose.orientation.y = 0.;
    srv.request.start.pose.orientation.z = start_phi_q.z;
    srv.request.start.pose.orientation.w = start_phi_q.w;

    // Set goal from input
    srv.request.goal.header.frame_id = "map";
    srv.request.goal.pose.position.x = x;
    srv.request.goal.pose.position.y = y;
    srv.request.goal.pose.position.z = 0.;
    srv.request.goal.pose.orientation.x = 0.;
    srv.request.goal.pose.orientation.y = 0.;
    srv.request.goal.pose.orientation.z = goal_phi_q.z;
    srv.request.goal.pose.orientation.w = goal_phi_q.w;

    // Send request
    if (check_path.call(srv)) { //; // TODO: Ensure this updates srv with the response...
        latest_response = srv.response;
        return !srv.response.plan.poses.empty();
    }
    else
        std::cout << "ERROR: RobotPlan::get_plan -> Service call failed!!!" << std::endl;
    // Expose the latest response, cause why not...
    latest_response = srv.response;

    return srv.response.plan.poses.size() > 0;
}



// https://wiki.ros.org/move_base
bool RobotPlan::clear_costmaps( ) {
    std_srvs::Empty msg;

    if ( movebase_clear.call(msg) )
        return true;
    else
        return false;
}



bool RobotPlan::set_pose_estimate( float x, float y, float phi ) {
    geometry_msgs::PoseWithCovarianceStamped msg;
    
    // Get Quaternion to represent phi orientation
    geometry_msgs::Quaternion phi_q = tf::createQuaternionMsgFromYaw(phi);

    msg.header.frame_id = "map";
    msg.header.stamp    = ros::Time::now();

    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.orientation.z = phi_q.z;
    msg.pose.pose.orientation.w = phi_q.w;

    pose_estimate.publish(msg);
    ros::spinOnce();
    return true;
}




RobotPose& RobotPlan::get_robot_pose( ) {
    return robot_pose;
}