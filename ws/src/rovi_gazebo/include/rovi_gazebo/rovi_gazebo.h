#pragma once

#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/CollisionObject.h>

namespace rovi_gazebo
{

	static constexpr auto NUM_JOINTS = 6;
	
	void
	set_projector(bool state);
	
	void
	get_camera_img(); // todo
	
	void // std::unordered_map<std::string, cv::Mat>
	get_stereo_camera_img(); // todo
	
	void
	get_point_cloud(); // todo
	
	gazebo_msgs::LinkStates
	get_link_states();

	gazebo_msgs::ModelStates
	get_model_states();

	sensor_msgs::JointState
	get_joint_states();
	
	geometry_msgs::Pose
	get_model_pose(const std::string& name);

	sensor_msgs::JointState
	get_current_robot_state();

	sensor_msgs::JointState
	get_current_gripper_state();
	
	geometry_msgs::Pose
	get_current_base_pose();

	geometry_msgs::Pose
	get_current_ee_pose(); // todo

	geometry_msgs::Pose
	get_current_tcp_pose(); // todo

	geometry_msgs::Pose
	get_current_link6_pose(bool in_world_frame = false);

	geometry_msgs::Pose
	get_link6_given_ee(const geometry_msgs::Pose& pose_ee); // todo

	geometry_msgs::Pose
	get_link6_given_tcp(const geometry_msgs::Pose& pose_tcp); // todo

	std::vector<moveit_msgs::CollisionObject>
	get_collision_objects(const std::string& planning_frame, const std::vector<std::string>& excludes = { "ur5", "camera_stereo", "openni_kinect", "ground_plane", "projector" });

}
