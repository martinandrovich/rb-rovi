#pragma once

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/CollisionObject.h>

namespace rovi_gazebo
{

	// should be moved to rovi_gazebo?
	// add in_base_frame = true bool
	
	sensor_msgs::JointState
	get_current_robot_state();
	
	sensor_msgs::JointState
	get_current_gripper_state();
	
	geometry_msgs::Pose
	get_current_link6_pose();

	geometry_msgs::Pose
	get_current_ee_pose();

	geometry_msgs::Pose
	get_current_tcp_pose();

	geometry_msgs::Pose
	get_link6_given_ee(const geometry_msgs::Pose& pose_ee);

	geometry_msgs::Pose
	get_link6_given_tcp(const geometry_msgs::Pose& pose_tcp);

	std::vector<moveit_msgs::CollisionObject>
	get_collision_objects();	
	
}
