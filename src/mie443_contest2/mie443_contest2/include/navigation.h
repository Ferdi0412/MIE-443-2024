#pragma once

class Navigation {
	public:
		static bool moveToGoal(float xGoal, float yGoal, float phiGoal);
		static bool getPlan(float xPos, float yPos, float phiGoal, ros::NodeHandle &nh, RobotPose robotPose);
};
