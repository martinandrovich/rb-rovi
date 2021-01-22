#include <fstream>
#include <tuple>
#include <chrono>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_kdl.h>
#include <kdl_conversions/kdl_msg.h>

#include <Eigen/Eigen>

#include <rovi_utils/rovi_utils.h>
#include <rovi_planner/rovi_planner.h>

int
main(int argc, char** argv)
{

	using namespace rovi_utils;
	using namespace std::chrono_literals;

	ros::init(argc, argv, "interpolation_kdl");
	ros::NodeHandle nh;

	std::vector<geometry_msgs::Pose> waypoints = 
	{
		rovi_utils::make_pose({ 0.0, 0.0, 0.0 }, { 0, 0, 0 }),
		rovi_utils::make_pose({ 1.0, 0.0, 0.0 }, { 0, 0, 0 }),
		rovi_utils::make_pose({ 2.0, 1.0, 0.0 }, { 0, 0, 0 }),
		rovi_utils::make_pose({ 1.0, 2.0, 0.0 }, { 0, 0, 0 }),
	};
	
	double VEL_MAX = 0.2, ACC_MAX = 0.05, EQUIV_RADIUS = 0.01, PAR_RADIUS = 0.2;
	
	auto traj_lin  = rovi_planner::traj_linear(waypoints, VEL_MAX, ACC_MAX, EQUIV_RADIUS);
	// auto traj_lin2 = rovi_planner::traj_linear2(waypoints, VEL_MAX, ACC_MAX, EQUIV_RADIUS);
	auto traj_par  = rovi_planner::traj_parabolic(waypoints, VEL_MAX, ACC_MAX, PAR_RADIUS, EQUIV_RADIUS);

	// export to files
	const std::string dir = get_experiment_dir("rovi_system");
	
	rovi_utils::export_traj(traj_lin, dir + "/traj_lin.csv", 0.01);
	rovi_utils::export_traj(traj_par, dir + "/traj_par.csv", 0.01);
	
	// export durations
	std::ofstream file(dir + "/dur.txt", std::ofstream::out);
	file << traj_lin->Duration()  << std::endl;
	file << traj_par->Duration()  << std::endl;
	file.close();

	return 0;
}