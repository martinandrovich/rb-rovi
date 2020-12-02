#include "rovi_gazebo/rovi_gazebo.h"

#include <rovi_utils/rovi_utils.h>

#include <ros/ros.h>
#include <iostream>
#include <thread>

sensor_msgs::JointState
rovi_gazebo::get_current_robot_state()
{
	// create and detach a thread that atomically updates the static variable in here;
	// atomically return the variable when this method is called

	static std::mutex mtx_joint_states;
	static sensor_msgs::JointState joint_states;
	static auto t = rovi_utils::create_async_listener("/joint_states", joint_states, mtx_joint_states);

	// return mutexed value
	while (true)
	{
		std::lock_guard<std::mutex> lock(mtx_joint_states);
		if (not joint_states.name.empty())
			return joint_states;
		ros::Duration(0.1).sleep();
	}
}
