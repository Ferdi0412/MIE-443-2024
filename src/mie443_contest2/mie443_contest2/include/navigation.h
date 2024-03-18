#pragma once

#include <ros/service_client.h>
#include <robot_pose.h>

class Navigation {
	public:
		static bool moveToGoal(float xGoal, float yGoal, float phiGoal);
};
