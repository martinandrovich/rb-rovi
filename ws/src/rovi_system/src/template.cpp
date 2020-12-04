#include <iostream>

#include <ros/ros.h>
#include <rovi_utils/rovi_utils.h>
#include <rovi_gazebo/rovi_gazebo.h>
#include <rovi_planner/rovi_planner.h>
#include <rovi_pose_estimator/rovi_pose_estimator.h>

int
main(int argc, char** argv)
{
	// init node
	ros::init(argc, argv, "template");
	ros::NodeHandle nh;
	
	std::cout << "\nPress [ENTER] to start template test..." << std::endl;
	std::cin.ignore();
	
	// start simulation
	rovi_gazebo::set_simulation(true);
	
	// end program
	std::cin.ignore();
	return 0;
}