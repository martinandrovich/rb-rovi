#include "rovi_planner/rovi_planner.h"

#include <tf_conversions/tf_kdl.h>

#include <kdl/frames.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/path_point.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/trajectory_stationary.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/utilities/error.h>

KDL::Trajectory_Composite
rovi_planner::traj_linear(const std::vector<geometry_msgs::Pose>& waypoints, double vel_max, double acc_max)
{
	if (waypoints.empty())
		throw std::runtime_error("There must be at least one waypoint.");

	auto interpolator    = new KDL::RotationalInterpolation_SingleAxis();
	auto traj            = new KDL::Trajectory_Composite();
	auto path            = new KDL::Path_Composite();
	auto vel_profile     = new KDL::VelocityProfile_Trap(vel_max, acc_max);
	
	// convert vector<Pose> to vector<KDL::Frame>
	std::vector<KDL::Frame> frames;

	for (const auto& pt : waypoints)
	{
		static KDL::Frame frame;
		tf::poseMsgToKDL(pt, frame);
		frames.push_back(frame);
	}

	// try creating the trajectory; catch any errors
	try
	{
		// create a trajectory segment for each waypoint defined as a path point with some velocity profile
		for (auto pt : frames)
		{
			auto traj_seg = new KDL::Trajectory_Segment(new KDL::Path_Point(pt), vel_profile);
			traj->Add(traj_seg);
		}

	}
	catch (const KDL::Error& e)
	{
		ROS_ERROR("Could not plan trajectory.");

		ROS_INFO("Planning was attempted with following waypoints:\n\n");
		for (auto const& point : frames)
			std::cout << point << "\n\n";

		std::cerr << e.Description() << std::endl;
		std::cerr << e.GetType() << std::endl;

		exit(-1);
	}

	return *traj;
}

KDL::Trajectory_Composite
rovi_planner::traj_parabolic(const std::vector<geometry_msgs::Pose>& waypoints, double vel_max, double acc_max, double corner_radius, double equiv_radius)
{

	// https://github.com/DonSiMP/trajectory_generators

	// Trajectory_Composite implements a trajectory that is composed of underlying
	// trajectoria (Trajectory_Segment objects). A single trajectory segment is generated
	// by defining a path (Path_RoundedComposite), composed of waypoints (KDL::Frame(s))
	// with rounded corners and a trapezoidal velocity profile.

	if (waypoints.empty())
		throw std::runtime_error("There must be at least one waypoint.");

	auto interpolator    = new KDL::RotationalInterpolation_SingleAxis();
	auto traj            = new KDL::Trajectory_Composite();
	auto path            = new KDL::Path_RoundedComposite(corner_radius, equiv_radius, interpolator);
	auto vel_profile     = new KDL::VelocityProfile_Trap(vel_max, acc_max);

	// convert vector<Pose> to vector<KDL::Frame>
	std::vector<KDL::Frame> frames;

	for (const auto& pt : waypoints)
	{
		static KDL::Frame frame;
		tf::poseMsgToKDL(pt, frame);
		frames.push_back(frame);
	}

	// try creating the trajectory; catch any errors
	try
	{

	// there are multiple waypoints
	if (waypoints.size() > 1)
	{
		// add all waypoints (frames) to path
		for (auto pt : frames)
			path->Add(pt);

		// finish creating the path
		path->Finish();

		// configure velocity profile based on path start and end
		vel_profile->SetProfile(0, path->PathLength());

		// add trajectory segment from path and velocity profile to final trajectory
		auto traj_seg = new KDL::Trajectory_Segment(path, vel_profile);
		traj->Add(traj_seg);
	}
	// there is a single waypoint (endpoint)
	else
	{
		auto traj_seg = new KDL::Trajectory_Segment(new KDL::Path_Point(frames[0]), vel_profile);
		traj->Add(traj_seg);
	}

	// wait 0.5 seconds at the end of trajectory
	// traj->Add(new KDL::Trajectory_Stationary(0.5, frames.back()));

	}
	catch (const KDL::Error& e)
	{
		ROS_ERROR("Could not plan trajectory.");

		ROS_INFO("Planning was attempted with following waypoints:\n\n");
		for (auto const& point : frames)
			std::cout << point << "\n\n";

		std::cerr << e.Description() << std::endl;
		std::cerr << e.GetType() << std::endl;

		exit(-1);
	}

	return *traj;
}