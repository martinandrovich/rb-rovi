#pragma once

#include <string>
#include <vector>
#include <unordered_map>

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/CollisionObject.h>

namespace rovi_gazebo
{

	// -- constants -----------------------------------------------------------

	static constexpr auto NUM_JOINTS = 6;

	static constexpr struct
	{
		double LENGTH             = 0.80; // [m] (x)
		double WIDTH              = 1.20; // [m] (y)
		double HEIGHT             = 0.75; // [m] (z)
		double MASS               = 10.0; // [kg]
		std::array<double, 3> POS = { 0.4, 0.6, 0.64 }; // [m]
	} TABLE;

	// -- transformations -----------------------------------------------------------

	Eigen::Isometry3d
	w_T_b();

	// -- gazebo --------------------------------------------------------------------

	void
	set_simulation(bool state);

	void
	set_projector(bool state);

	cv::Mat
	get_camera_img(); // todo
	
	cv::Mat
	get_camera_info(); // todo

	std::unordered_map<std::string, cv::Mat>
	get_stereo_camera_imgs();
	
	void
	get_stereo_camera_info(); // todo

	void
	get_point_cloud(); // todo
	
	std::vector<moveit_msgs::CollisionObject>
	get_collision_objects(const std::string& planning_frame, const std::vector<std::string>& excludes = { "ur5", "camera_stereo", "openni_kinect", "ground_plane", "projector" });

	gazebo_msgs::LinkStates
	get_link_states();

	gazebo_msgs::ModelStates
	get_model_states();

	sensor_msgs::JointState
	get_joint_states();

	geometry_msgs::Pose
	get_model_pose(const std::string& name);
	
	void
	spawn_model(const std::string& model, const std::string& name, const std::array<double, 3>& pos, const std::array<double, 3>& rpy = { 0, 0, 0 });
	
	void
	spawn_model(const std::string& model, const std::string& name, const geometry_msgs::Pose& pose);
	
	void
	move_model(const std::string& name, const std::array<double, 3>& pos, const std::array<double, 3>& rpy = { INFINITY, INFINITY, INFINITY});
	
	void
	remove_model(const std::string& name);
	
	// -- robot state ---------------------------------------------------------------

	sensor_msgs::JointState
	get_current_robot_state();

	sensor_msgs::JointState
	get_current_gripper_state();

	geometry_msgs::Pose
	get_current_base_pose();
	
	geometry_msgs::Pose
	get_current_link6_pose(bool in_world_frame = false);

	geometry_msgs::Pose
	get_current_ee_pose();

	geometry_msgs::Pose
	get_current_tcp_pose();
	
	geometry_msgs::Pose
	get_ee_given_pos(const geometry_msgs::Pose& pose_obj, const Eigen::Isometry3d& offset = Eigen::Translation3d(0.0, 0.0, 0.0) * Eigen::Isometry3d::Identity());
	
	geometry_msgs::Pose
	get_tcp_given_pos(const geometry_msgs::Pose& pose_obj, const Eigen::Isometry3d& offset = Eigen::Translation3d(0.0, 0.0, 0.0) * Eigen::Isometry3d::Identity());

}
