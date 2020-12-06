#pragma once

#include <array>
#include <string>
#include <unordered_map>
#include <tuple>
#include <fstream>

#include <Eigen/Eigen>

#include <rovi_utils/rovi_utils.h>
#include <rovi_gazebo/rovi_gazebo.h>
#include <rovi_planner/rovi_planner.h>

using namespace rovi_utils;
using namespace rovi_gazebo;
using namespace rovi_planner;

template <typename T>
struct PlanningData
{
	size_t iteration;
	double planning_time;
	double traj_duration;
	T traj;
};

template <typename T>
void
export_planning_data(const std::vector<PlanningData<T>>& data, const std::string& filename)
{
	auto fs = std::ofstream(filename, std::ofstream::out);
	fs << "i, planning time [ms], traj_duration [s]" << std::endl;
	for (const auto& plan : data)
		fs << plan.iteration << ", " << plan.planning_time << ", " << plan.traj_duration << std::endl;
	fs.close();
}

static const auto PLACE_LOCATION = // in world frame
	make_pose({ 0.70, 0.11, 0.75, 0.0, 0.0, 0.0 });

static const auto PICK_LOCATIONS = std::array // in world frame
{
	make_pose({ 0.15, 1.05, 0.75, 0.0, 0.0, 0.0 }),
	make_pose({ 0.40, 1.05, 0.75, 0.0, 0.0, 0.0 }),
	make_pose({ 0.65, 1.05, 0.75, 0.0, 0.0, 0.0 }),
};

static std::unordered_map<std::string, geometry_msgs::Pose> VIA_POINTS = // in world frame
{
	{ "orient",    make_pose({ TABLE.LENGTH/2, TABLE.WIDTH/2,       1.20, 0.00, 0.00, 0.00 }) },
	{ "move-down", make_pose({ TABLE.LENGTH/2, TABLE.WIDTH/2,       1.00, 0.00, 0.00, 0.00 }) },
	{ "pre-fork",  make_pose({ TABLE.LENGTH/2, TABLE.WIDTH/2 + 0.1, 0.80, 0.00, 0.00, 0.00 }) },
	{ "fork",      make_pose({ TABLE.LENGTH/2, TABLE.WIDTH/2 + 0.2, 0.80, 0.00, 0.00, 0.00 }) },
};

static const auto PRE_PICK_OFFSET = // x, y, z
	Eigen::Translation3d(0.0, -0.1, 0.1) * Eigen::Isometry3d::Identity();

static const auto PICK_OFFSET = // x, y, z
	Eigen::Translation3d(0.0, 0.0, 0.1) * Eigen::Isometry3d::Identity();
