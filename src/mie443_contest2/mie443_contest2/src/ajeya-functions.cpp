#ifndef AJEYA_FUNCTIONS_CPP
#define AJEYA_FUNCTIONS_CPP

// #include <imagePipeline.h>
#include <boxes.h>
#include <nav_msgs/GetPlan.h>
#include <vector>
#include <iostream>

#include <navigation.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_datatypes.h>

bool Navigation::moveToExistingGoal(float xGoal, float yGoal, float phiGoal){

    // Make request to make_plan service
    ros::ServiceClient check_path = n.serviceClient<nav_msgs::GetPlan>("/move_base/NavfnROS/make_plan");
    nav_msgs::GetPlan srv;
    srv.request.start = start;
    srv.request.goal = goal;
    check_path.call(srv);
    // Return path reachability
    if  (srv.response.plan.poses.size() > 0){

            // Set up and wait for actionClient.
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
        while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
        }
        // Set goal.
        geometry_msgs::Quaternion phi = tf::createQuaternionMsgFromYaw(phiGoal);
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x =  xGoal;
        goal.target_pose.pose.position.y =  yGoal;
        goal.target_pose.pose.position.z =  0.0;
        goal.target_pose.pose.orientation.x = 0.0;
        goal.target_pose.pose.orientation.y = 0.0;
        goal.target_pose.pose.orientation.z = phi.z;
        goal.target_pose.pose.orientation.w = phi.w;
        ROS_INFO("Sending goal location ...");
        // Send goal and wait for response.
        ac.sendGoal(goal);
        ac.waitForResult();
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            ROS_INFO("You have reached the destination");
            return true;
        } else {
            ROS_INFO("The robot failed to reach the destination");
            return false;
        }

    }else {
        ROS_WARN("The robot is trying to go to an imaginary location...");
        return false;
    }

	
}


#endif