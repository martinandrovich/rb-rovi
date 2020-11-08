#pragma once

#include <string>
#include <array>

#include <moveit_msgs/CollisionObject.h>
#include <geometric_shapes/shape_operations.h>

namespace moveit
{
	moveit_msgs::CollisionObject
	make_mesh_cobj(const std::string& name, const std::array<double, 3>& pos, const std::array<double, 4>& ori = { 1, 0, 0, 0 });
}