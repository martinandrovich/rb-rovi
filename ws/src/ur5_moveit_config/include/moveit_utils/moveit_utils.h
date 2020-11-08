#pragma once

#include <string>
#include <array>
#include <vector>

#include <moveit_msgs/CollisionObject.h>
#include <geometric_shapes/shape_operations.h>
#include <tf/transform_broadcaster.h>

#include <thread>
#include <pthread.h>

namespace moveit
{
	moveit_msgs::CollisionObject
	make_mesh_cobj(const std::string& name, const std::string& frame, const std::array<double, 3>& pos, const std::array<double, 4>& ori = { 1, 0, 0, 0 });

	std::vector<moveit_msgs::CollisionObject>
	get_gazebo_obj(const std::string& frame, const std::vector<std::string>& excludes = { "ur5", "camera_stereo", "openni_kinect", "ground_plane" });

	void 
	move_base(const std::string& frame_id, const std::string& child, const std::array<double, 3>& pos);
}