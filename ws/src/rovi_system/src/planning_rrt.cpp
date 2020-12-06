#include <iostream>

#include <ros/ros.h>
#include <rovi_utils/rovi_utils.h>
#include <rovi_gazebo/rovi_gazebo.h>
#include <rovi_planner/rovi_planner.h>

#include "planning_common.hpp"

int
main(int argc, char** argv)
{
	// init node
	ros::init(argc, argv, "planning_rrt");
	ros::NodeHandle nh;
	
	std::cout << "\nPress [ENTER] to start planning_rrt test..." << std::endl;
	std::cin.ignore();
	
	// define poses
	const auto obj_model  = "bottle";
	const auto obj_name   = obj_model + std::to_string(1);
	const auto pose_pick  = PICK_LOCATIONS[0];
	const auto pose_place = PLACE_LOCATION;
	
	// spawn object and box
	const auto& pos = pose_pick.position;
	rovi_gazebo::spawn_model(obj_model, obj_name, { pos.x, pos.y, pos.z });
	
	// start simulation
	rovi_gazebo::set_simulation(true);
	
	// end program
	std::cin.ignore();
	return 0;
}