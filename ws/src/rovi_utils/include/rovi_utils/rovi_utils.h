#pragma once

#include <string>
#include <array>
#include <vector>

#include <Eigen/Eigen>

#include <geometry_msgs/Pose.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/planning_scene/planning_scene.h>

namespace rovi_utils
{
	// -- geometry_msgs -----------------------------------------------------------

	geometry_msgs::Pose
	make_pose(const std::array<double, 3>& pos, const Eigen::Quaternion<double>& ori);

	geometry_msgs::Pose
	make_pose(const std::array<double, 6>& pose);

	geometry_msgs::Pose
	make_pose(const std::array<double, 3>& pos, const std::array<double, 3>& rpy);

	// -- trajectories ------------------------------------------------------------

	template<typename T>
	void
	export_traj(T& traj, const std::string&& filename, const double resolution = 0.01 /* [s] */);

	// -- moveit ------------------------------------------------------------------

	moveit_msgs::CollisionObject
	make_mesh_cobj(const std::string& name, const std::string& frame, const std::array<double, 3>& pos, const std::array<double, 4>& ori = { 1, 0, 0, 0 });

	std::vector<moveit_msgs::CollisionObject>
	get_gazebo_obj(const std::string& frame, const std::vector<std::string>& excludes = { "ur5", "camera_stereo", "openni_kinect", "ground_plane" });

	void
	move_base(moveit::core::RobotState& state, const std::array<double, 3>& offset, const std::string& virtual_joint_name = "world_offset");

	void // DEPRECATED
	move_base(const std::string& frame_id, const std::string& child, const std::array<double, 3>& pos);

	void
	spawn_obj();

}