#pragma once

#include <string>
#include <array>
#include <vector>

#include <Eigen/Eigen>

#include <geometry_msgs/Pose.h>
#include <moveit_msgs/CollisionObject.h>

namespace rovi_utils
{
	geometry_msgs::Pose
	make_pose(const std::array<double, 3>& pos, const Eigen::Quaternion<double>& ori);

	// geometry_msgs::Pose
	// make_pose(const std::array<double, 3>& pos, const std::array<double, 4>& ori);

	geometry_msgs::Pose
	make_pose(const std::array<double, 3>& pos, const std::array<double, 3>& rpy);

	moveit_msgs::CollisionObject
	make_mesh_cobj(const std::string& name, const std::string& frame, const std::array<double, 3>& pos, const std::array<double, 4>& ori = { 1, 0, 0, 0 });

	std::vector<moveit_msgs::CollisionObject>
	get_gazebo_obj(const std::string& frame, const std::vector<std::string>& excludes = { "ur5", "camera_stereo", "openni_kinect", "ground_plane" });

	void 
	move_base(const std::string& frame_id, const std::string& child, const std::array<double, 3>& pos);

	void
	spawn_obj();
}