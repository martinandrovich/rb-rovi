#include <iostream>

#include <ros/ros.h>
#include <rovi_utils/rovi_utils.h>
#include <rovi_gazebo/rovi_gazebo.h>
#include <rovi_planner/rovi_planner.h>
#include <ur5_controllers/interface.h>
#include <ur5_dynamics/ur5_dynamics.h>

#include "planning_common.hpp"

int
main(int argc, char** argv)
{
	// init nodes
	ros::init(argc, argv, "planning_lin");
	ros::NodeHandle nh;
	
	// setup simulation and wait for it to settle
	rovi_gazebo::set_simulation(true);
	rovi_gazebo::set_projector(false);
	ur5_controllers::wsg::release();
	ros::Duration(1).sleep();
	
	// compute via points
	std::vector<geometry_msgs::Pose> waypoints = 
	{
		get_current_ee_pose(),                                // starting pose
		get_ee_given_pos(VIA_POINTS["orient"]),               // orient EE and center
		get_ee_given_pos(VIA_POINTS["move-down"]),            // move down
		get_ee_given_pos(VIA_POINTS["pre-fork"]),             // pre-fork
		get_ee_given_pos(VIA_POINTS["fork"]),                 // fork
		get_ee_given_pos(PICK_LOCATIONS[0], PRE_PICK_OFFSET), // pre-obj
		get_ee_given_pos(PICK_LOCATIONS[0], PICK_OFFSET)      // obj
	};
	
	// do interpolation
	auto traj_lin_ee = rovi_planner::traj_linear(waypoints, 0.1, 0.1, 0.001);
	auto traj_par_ee = rovi_planner::traj_parabolic(waypoints, 0.1, 0.1, 0.05, 0.001);
	
	// export to file
	rovi_utils::export_traj(traj_lin_ee, "traj_lin.csv", 0.01);
	rovi_utils::export_traj(traj_par_ee, "traj_par.csv", 0.01);
	
	// execute end-effector trajectory using Cartesian controller
	const auto& traj = traj_par_ee;
	ROS_INFO_STREAM("Executing trajectory in Gazebo (" << traj.Duration() << " s)...");
	ur5_controllers::ur5::execute_traj(traj, ur5_controllers::ur5::EXEC_FREQ);
	ROS_INFO_STREAM("Done!");
	
	// end program
	std::cin.ignore();
	return 0;
}