#include <navigation.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/GetPlan.h>
#include <robot_pose.h>


bool Navigation::moveToGoal(float xGoal, float yGoal, float phiGoal){
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
}

bool Navigation::getPlan(float xPos, float yPos, float phiGoal, ros::NodeHandle &nh, RobotPose robotPose){

    geometry_msgs::Quaternion phi = tf::createQuaternionMsgFromYaw(phiGoal);

    // Initialize service client for GetPlan service
    ros::ServiceClient check_path = nh.serviceClient<nav_msgs::GetPlan>("/move_base/NavfnROS/make_plan");

    nav_msgs::GetPlan plan_srv;

    plan_srv.request.start.header.frame_id = "map";
    plan_srv.request.start.pose.position.x = robotPose.x;
    plan_srv.request.start.pose.position.y = robotPose.y;
    plan_srv.request.start.pose.orientation = tf::createQuaternionMsgFromYaw(robotPose.phi);

    // Set goal position for GetPlan service
    plan_srv.request.goal.header.frame_id = "map";
    plan_srv.request.goal.pose.position.x = xPos;
    plan_srv.request.goal.pose.position.y = yPos;
    plan_srv.request.goal.pose.orientation = tf::createQuaternionMsgFromYaw(phiGoal);

    // Call the GetPlan service
    if (check_path.call(plan_srv)) {
        // Check if the plan is valid (contains poses)
        return plan_srv.response.plan.poses.size() > 0;
    } else {
        ROS_ERROR("Failed to call GetPlan service");
        return false;
    }
}

