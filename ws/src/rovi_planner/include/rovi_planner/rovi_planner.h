#pragma once

#include <vector>
#include <array>

#include <Eigen/Eigen>

#include <geometry_msgs/Pose.h>
#include <kdl/trajectory_composite.hpp>

geometry_msgs::Pose
make_pose(const std::array<double, 3>& pos, const Eigen::Quaternion<double>& ori);

// geometry_msgs::Pose
// make_pose(const std::array<double, 3>& pos, const std::array<double, 4>& ori);

geometry_msgs::Pose
make_pose(const std::array<double, 3>& pos, const std::array<double, 3>& rpy);

namespace rovi_planner
{
	KDL::Trajectory_Composite
	traj_linear(const std::vector<geometry_msgs::Pose>& waypoints);

	KDL::Trajectory_Composite
	traj_parabolic(const std::vector<geometry_msgs::Pose>& waypoints, double vel_max = 1.0, double acc_max = 1.0, double corner_radius = 1.0, double equiv_radius = 1.0);
} // namespace rovi_planner
