#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <rovi_utils/rovi_utils.h>
#include <rovi_gazebo/rovi_gazebo.h>
#include <rovi_planner/rovi_planner.h>

int
main(int argc, char** argv)
{
	
	using namespace rovi_gazebo;
	
	// init node
	ros::init(argc, argv, "reachability");
	ros::NodeHandle nh;

	// start async spinner for moveit
	ros::AsyncSpinner spinner(0);
	spinner.start();
	
	// unpause Gazebo simulation
	rovi_gazebo::set_simulation(true);

	// init moveit planner
	rovi_planner::moveit_planner::init(nh);
	rovi_planner::moveit_planner::start_planning_scene_publisher();

	// parameters
	constexpr auto OBJ_NAME      = "bottle";
	constexpr auto OBJ_POS       = std::array{ 0.5, 1.0, TABLE.HEIGHT };
	constexpr auto OBJ_OFFSET    = std::array{ 0.0, 0.0, 0.10 };
	constexpr auto OBJ_ROT_AXIS  = std::array{ 0.0, 0.0, 1.0 }; // Z
	constexpr auto RESOLUTION    = 180;
	constexpr auto VISUALIZE     = false;

	// generate base poses
	std::vector<std::array<double, 3>> base_positions;
	for (double y = 0; y < TABLE.WIDTH; y += 0.1)
		for (double x = 0.; x < TABLE.LENGTH; x += 0.1)
			base_positions.push_back({ x, y, 0.74 });
			
	std::cout << "Number of generated base poses: " << base_positions.size() << std::endl;
	std::cout << "Press [ENTER] to start reachability test..." << std::endl;
	std::cin.ignore();

	// iterate different base positions and do test
	std::vector<rovi_planner::moveit_planner::ReachabilityData> results;
	for (const auto& base_pos : base_positions)
	{
		auto data = rovi_planner::moveit_planner::reachability(base_pos, OBJ_NAME, OBJ_POS, OBJ_OFFSET, OBJ_ROT_AXIS, RESOLUTION, VISUALIZE);
		results.push_back(data);
		// std::cin.ignore();
	}
	
	// write results to file
	std::ofstream fs("reachability.csv", std::ofstream::out);
	for (const auto& result : results)
		fs << result.base_pos[0] << ", " << result.base_pos[1] << ", " << result.plausible_states << "\n";
	
	fs.close();
	ROS_INFO("Reachbility analysis written to file 'reachability.csv'.");
	
	// end program
	std::cin.ignore();
	return 0;

	// print results
	// std::cout << "results:\n\n";
	// for (const auto& result : results)
	// {
	// 	std::cout
	// 		<< "base position:\n\n" << rovi_utils::make_pose(result.base_pos, { 0, 0, 0 }).position << "\n"
	// 		<< "num iterations: "   << result.iterations << "\n"
	// 		<< "num collisions: "   << result.collisions << "\n"
	// 		<< "ratio:          "   << result.ratio << "\n"
	// 		<< std::endl;
	// }
}