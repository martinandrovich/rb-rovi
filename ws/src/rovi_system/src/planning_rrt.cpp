#include <iostream>

#include <ros/ros.h>
#include <rovi_utils/rovi_utils.h>
#include <rovi_gazebo/rovi_gazebo.h>
#include <rovi_planner/rovi_planner.h>
#include <ur5_controllers/interface.h>

#include "planning_common.hpp"

int
main(int argc, char** argv)
{
	using namespace std::chrono_literals;
	
	// init node
	ros::init(argc, argv, "planning_rrt");
	ros::NodeHandle nh;
	
	// define poses
	const auto obj_model  = "bottle";
	const auto obj_name   = obj_model + std::to_string(1);
	const auto obj_pos    = PICK_LOCATIONS[0].position;
	
	// init moveit planner
	ROS_INFO_STREAM("Initializing moveit planner and scene...");
	rovi_planner::moveit_planner::init(nh);
	rovi_planner::moveit_planner::update_planning_scene();
	rovi_planner::moveit_planner::start_planning_scene_publisher();

	// setup simulation and wait for it to settle
	ROS_INFO_STREAM("Preparing simulation...");
	rovi_gazebo::set_simulation(true);
	rovi_gazebo::set_projector(false);
	
	ROS_INFO_STREAM("Releasing gripper...");
	ur5_controllers::wsg::release();
	std::this_thread::sleep_for(2s);
	
	// setup scence
	
	ROS_INFO_STREAM("Setting up scene...");
	rovi_gazebo::spawn_model("coffecan", "coffecan1", { 0.105778, 0.146090, 0.740000 }, { 0.000015, 0.0, 0.142340 });
	rovi_gazebo::spawn_model("mug", "mug1", { 0.238709, 0.223757, 0.740674 }, { -0.012724, -0.016048, 1.954001 });
	rovi_gazebo::spawn_model("crate", "crate1", { 0.641959, 0.676307, 0.739943 }, { 0.003127, -0.003012, 0.030163 });
	rovi_gazebo::spawn_model(obj_model, obj_name, { obj_pos.x, obj_pos.y, obj_pos.z });
	rovi_planner::moveit_planner::update_planning_scene();
	std::this_thread::sleep_for(1s);
	
	// estimate object pose

	// RRT is planning for TCP (not EE) !!!
	const auto pose_pick  = get_tcp_given_pos(PICK_LOCATIONS[0], PICK_OFFSET);
	const auto pose_place = get_tcp_given_pos(PLACE_LOCATION, PICK_OFFSET);
	const auto pose_home  = get_current_tcp_pose();
	
	// PICK
	{
	
	// plan and interpolate
	ROS_INFO_STREAM("Planning...");
	auto plan = rovi_planner::moveit_planner::plan(pose_pick, "RRTstar", 5.0, 10000);
	ROS_INFO_STREAM("Creating trajectory (interpolation)...");
	auto traj = rovi_planner::moveit_planner::get_traj(plan, 0.0001, 1/ur5_controllers::ur5::EXEC_FREQ, 0.2, 0.1);
	
	// execute in Gazebo
	ROS_INFO_STREAM("Executing trajectory in Gazebo...");
	ur5_controllers::ur5::execute_traj(traj, ur5_controllers::ur5::EXEC_FREQ);
	
	// update moving planning scene from gazebo
	ROS_INFO_STREAM("Updating planning scene...");
	rovi_planner::moveit_planner::update_planning_scene();

	// grasp
	ROS_INFO_STREAM("Grasping object...");
	ur5_controllers::wsg::grasp();
	std::this_thread::sleep_for(2s);
	
	// update moving planning scene from gazebo
	ROS_INFO_STREAM("Updating planning scene...");
	rovi_planner::moveit_planner::update_planning_scene();

	// attach object (and implcit update)
	ROS_INFO_STREAM("Attaching object to gripper...");
	rovi_planner::moveit_planner::attach_object_to_ee(obj_name);
	
	}
	
	// PLACE
	{
	
	// plan using moveit (to to place)
	ROS_INFO_STREAM("Planning...");
	auto plan = rovi_planner::moveit_planner::plan(pose_place, "RRTstar", 5.0, 10000);

	// get parabolic trajectory (time optimal)
	ROS_INFO_STREAM("Creating trajectory (interpolation)...");
	auto traj = rovi_planner::moveit_planner::get_traj(plan, 0.0001, 1/ur5_controllers::ur5::EXEC_FREQ, 0.2, 0.1);
	
	// execute in Gazebo
	ROS_INFO_STREAM("Executing trajectory in Gazebo...");
	ur5_controllers::ur5::execute_traj(traj, ur5_controllers::ur5::EXEC_FREQ);
	std::this_thread::sleep_for(1s);
	
	// update moving planning scene from gazebo (also detaches object)
	ROS_INFO_STREAM("Updating planning scene...");
	rovi_planner::moveit_planner::update_planning_scene();
	std::this_thread::sleep_for(1s);
	
	// release gripper
	
	ROS_INFO_STREAM("Releasing gripper...");
	ur5_controllers::wsg::release();
	std::this_thread::sleep_for(1s);
	
	// update moving planning scene from gazebo
	ROS_INFO_STREAM("Updating planning scene...");
	rovi_planner::moveit_planner::update_planning_scene();
	std::this_thread::sleep_for(1s);
	
	}
	
	// HOME
	{
	
	// plan and interpolate
	ROS_INFO_STREAM("Planning...");
	auto plan = rovi_planner::moveit_planner::plan(pose_home, "RRTstar", 5.0, 10000);
	ROS_INFO_STREAM("Creating trajectory (interpolation)...");
	auto traj = rovi_planner::moveit_planner::get_traj(plan, 0.0001, 1/ur5_controllers::ur5::EXEC_FREQ, 0.2, 0.1);
	
	// execute in Gazebo
	ROS_INFO_STREAM("Executing trajectory in Gazebo...");
	ur5_controllers::ur5::execute_traj(traj, ur5_controllers::ur5::EXEC_FREQ);
	
	// update moving planning scene from gazebo
	ROS_INFO_STREAM("Updating planning scene...");
	rovi_planner::moveit_planner::update_planning_scene();
	
	}
	
	// end program
	std::cin.ignore();
	return 0;
}