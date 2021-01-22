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
	using namespace rovi_utils;
	
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

	// grasping positions
	std::unordered_map<std::string, Eigen::Isometry3d> PICK_OFFSET = // in world frame
	{
		{ "side", Eigen::Translation3d(0.0, 0.0, 0.1) * Eigen::Isometry3d::Identity() },
		{ "top",  Eigen::Translation3d(0.0, 0.0, 0.2) * 
		          Eigen::AngleAxisd(0,       Eigen::Vector3d::UnitZ()) * // yaw
		          Eigen::AngleAxisd(0,       Eigen::Vector3d::UnitY()) * // pitch
		          Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX()) } // roll
	};
	
	// parameters
	constexpr auto OBJ_NAME   = "bottle";
	constexpr auto OBJ_POS    = std::array{ 0.5, 1.0, TABLE.HEIGHT };
	constexpr auto RESOLUTION = 180;
	constexpr auto VISUALIZE  = false;

	// generate base poses
	std::vector<std::array<double, 3>> base_positions;
	for (double y = 0; y < TABLE.WIDTH; y += 0.1)
		for (double x = 0.; x < TABLE.LENGTH; x += 0.1)
			base_positions.push_back({ x, y, 0.74 });
			
	std::cout << "Number of generated base poses: " << base_positions.size() << std::endl;
	std::cout << "Press [ENTER] to start reachability test..." << std::endl;
	std::cin.ignore();
	
	// set experiment name and make dir
	const std::string dir = get_experiment_dir("rovi_system");
	
	// side and top grasping pos
	for (auto grasp_pos : { "top", "side" })
	{
		// define rotation axis relative to grasping position (bug)
		const auto ROT_AXIS = (grasp_pos == "side") ? std::array{ 0.0, 0.0, 1.0 } : std::array{ 0.0, 1.0, 0.0 }; // Z or Y
		
		// iterate different base positions and do test
		std::vector<rovi_planner::moveit_planner::ReachabilityData> results;
		for (const auto& base_pos : base_positions)
		{
			auto data = rovi_planner::moveit_planner::reachability(base_pos, OBJ_NAME, OBJ_POS, PICK_OFFSET[grasp_pos], ROT_AXIS, RESOLUTION, VISUALIZE);
			results.push_back(data);
		}
		
		// write results to file
		std::ofstream fs(dir + "/" + grasp_pos + ".csv", std::ofstream::out);
		for (const auto& result : results)
			fs << result.base_pos[0] << ", " << result.base_pos[1] << ", " << result.plausible_states << "\n";
		
		fs.close();
	}

	// end program
	ROS_INFO("Completed reachability analysis, press [ENTER] to terminate.");
	std::cin.ignore();
	return 0;
}