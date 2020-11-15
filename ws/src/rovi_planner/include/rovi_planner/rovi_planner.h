#pragma once

#include <vector>
#include <array>

#include <Eigen/Eigen>

#include <geometry_msgs/Pose.h>
#include <kdl/trajectory_composite.hpp>

namespace rovi_planner
{

	KDL::Trajectory_Composite
	traj_linear(const std::vector<geometry_msgs::Pose>& waypoints, double vel_max = 1.0 /* [m/s] */, double acc_max = 1.0 /* [m/s^2] */, double equiv_radius = 0.05 /* [m] */);

	KDL::Trajectory_Composite
	traj_parabolic(const std::vector<geometry_msgs::Pose>& waypoints, double vel_max = 1.0 /* [m/s] */, double acc_max = 1.0 /* [m/s^2] */, double corner_radius = 0.05 /* [m] */, double equiv_radius = 0.05 /* [m] */);

} // namespace rovi_planner
