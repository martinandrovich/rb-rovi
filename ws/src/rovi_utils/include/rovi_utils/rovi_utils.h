#pragma once

#include <string>
#include <array>
#include <vector>

#include <Eigen/Eigen>

#include <moveit/planning_scene/planning_scene.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/CollisionObject.h>

namespace rovi_utils
{
	// -- geometry_msgs -----------------------------------------------------------

	geometry_msgs::Pose
	make_pose(const std::array<double, 3>& pos, const Eigen::Quaternion<double>& ori);

	geometry_msgs::Pose
	make_pose(const std::array<double, 6>& pose);

	geometry_msgs::Pose
	make_pose(const std::array<double, 3>& pos, const std::array<double, 3>& rpy);

	// -- gazebo ------------------------------------------------------------------

	geometry_msgs::Pose
	get_link6_given_ee(const geometry_msgs::Pose& pose_ee);

	geometry_msgs::Pose
	get_link6_given_tcp(const geometry_msgs::Pose& pose_tcp);

	geometry_msgs::Pose
	get_current_link6_pose();

	geometry_msgs::Pose
	get_current_ee_pose();

	geometry_msgs::Pose
	get_current_tcp_pose();

	// -- trajectories ------------------------------------------------------------

	template<typename T>
	void
	export_traj(T& traj, const std::string&& filename, const double resolution = 0.01 /* [s] */);

	std::vector<geometry_msgs::Pose>
	waypoints_from_traj(const robot_trajectory::RobotTrajectory& traj);

	// -- moveit ------------------------------------------------------------------

	moveit_msgs::CollisionObject
	make_mesh_cobj(const std::string& name, const std::string& frame, const std::array<double, 3>& pos, const std::array<double, 4>& ori = { 1, 0, 0, 0 });

	std::vector<moveit_msgs::CollisionObject>
	get_gazebo_obj(const std::string& frame, const std::vector<std::string>& excludes = { "ur5", "camera_stereo", "openni_kinect", "ground_plane" });

	void
	move_base(moveit::core::RobotState& state, const std::array<double, 3>& offset, const std::string& virtual_joint_name = "world_offset");

}