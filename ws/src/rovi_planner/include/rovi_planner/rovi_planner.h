#pragma once

#include <vector>
#include <array>

#include <Eigen/Eigen>

#include <geometry_msgs/Pose.h>
#include <kdl/trajectory_composite.hpp>

namespace rovi_planner
{

	KDL::Trajectory_Composite
	traj_linear(const std::vector<geometry_msgs::Pose>& waypoints, double vel_max = 1.0 /* [m/s] */, double acc_max = 1.0 /* [m/s^2] */);

	KDL::Trajectory_Composite
	traj_parabolic(const std::vector<geometry_msgs::Pose>& waypoints, double vel_max = 1.0 /* [m/s] */, double acc_max = 1.0 /* [m/s^2] */, double corner_radius = 1.0 /* [m] */, double equiv_radius = 1.0 /* [m] */);

} // namespace rovi_planner
