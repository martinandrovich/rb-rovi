#pragma once

#include <vector>
#include <array>
#include <tuple>

#include <Eigen/Eigen>
#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
 
#include <moveit/planning_interface/planning_interface.h>
#include <kdl/trajectory_composite.hpp>

namespace rovi_planner
{
	// trajectory planners

	KDL::Trajectory_Composite
	traj_linear(const std::vector<geometry_msgs::Pose>& waypoints, double vel_max = 1.0 /* [m/s] */, double acc_max = 1.0 /* [m/s^2] */, double equiv_radius = 0.05 /* [m] */);
	
	std::array<KDL::Trajectory_Composite*, 6>
	traj_linear(const std::vector<sensor_msgs::JointState>& waypoints, double vel_max = 1.0 /* [m/s] */, double acc_max = 1.0 /* [m/s^2] */, double equiv_radius = 0.05 /* [m] */);

	KDL::Trajectory_Composite
	traj_parabolic(const std::vector<geometry_msgs::Pose>& waypoints, double vel_max = 1.0 /* [m/s] */, double acc_max = 1.0 /* [m/s^2] */, double corner_radius = 0.05 /* [m] */, double equiv_radius = 0.05 /* [m] */);

	planning_interface::MotionPlanResponse
	traj_moveit(const geometry_msgs::Pose& pose_des, const std::string& planner = "RRTConnect", std::vector<double> q = {} );

	// reachability planner

	void 
	reachability( const std::vector<std::array<double, 3>>& base_pts, const std::array<double, 3>& obj, const std::array<double, 3>& offset, const std::array<double, 3>& axis, ros::Publisher& planning_scene_pub, 
				  int resolution = 16, const std::string& obj_name = "bottle", const std::array<double, 3>& table = {0.4, 0.6, 0.64} );

} // namespace rovi_planner
