#include <ros/ros.h>
#include <gazebo_msgs/LinkStates.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ur5_dynamics/ur5_dynamics.h>

#include <pluginlib/class_loader.h>
#include <boost/scoped_ptr.hpp>

#include <rovi_utils/rovi_utils.h>

// arm and robot_description
constexpr auto ARM_GROUP    	 = "ur5_arm";
constexpr auto WSG_GROUP 	     = "wsg";
constexpr auto ROBOT_DESCRIPTION = "robot_description";
constexpr auto PLANNER_PLUGIN_NAME = "ompl_interface/OMPLPlanner";

//https://github.com/ros-planning/moveit_tutorials/blob/master/doc/creating_moveit_plugins/lerp_motion_planner/src/lerp_example.cpp

int
main(int argc, char** argv)
{
	// init node
	ros::init(argc, argv, "rrt_test");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	// we we will use movegroupinterface to interface the planning group
	// moveit::planning_interface::MoveGroupInterface move_group(ARM_GROUP);

	// we will use planning-scene interface to add and remove collision objects in our virtual world
	//moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	// load robot model and kinematic model, and use it to setup the planning scene
    robot_model_loader::RobotModelLoaderPtr robot_model_loader(new robot_model_loader::RobotModelLoader(ROBOT_DESCRIPTION));
    robot_model::RobotModelPtr robot_kinematic_model(robot_model_loader->getModel());

    // set up the planning scene for collision detection
    planning_scene::PlanningScene planning_scene(robot_kinematic_model);

	// visualization in rviz
	moveit_visual_tools::MoveItVisualTools visual_tools("ur5_link0");
	visual_tools.deleteAllMarkers();
	
    // get robot state, this is a raw reference and pointers for arm_group and wsg_group
    auto& robot_state    = planning_scene.getCurrentStateNonConst();
    const auto arm_group = robot_state.getJointModelGroup(ARM_GROUP);
    const auto wsg_group = robot_state.getJointModelGroup(WSG_GROUP);

	// set gripper default
    std::vector gripper_state{0.05, 0.05};
    robot_state.setJointGroupPositions(wsg_group, gripper_state);

	// make a scene message to update scene
	moveit_msgs::PlanningScene planning_scene_msg;

	// move the base of the robot
	rovi_utils::move_base(robot_state, {0.1, 0.1, 0.75});

	// insert table and other constant objects into the planning scene
	std::vector<moveit_msgs::CollisionObject> collision_objects
	{
		rovi_utils::make_mesh_cobj("table",  planning_scene.getPlanningFrame() , {0.4, 0.6, 0.64}),
		rovi_utils::make_mesh_cobj("bottle", planning_scene.getPlanningFrame() , {0.5, 1.0, 0.75})
	};

	robot_state.setToDefaultValues(arm_group, "home");
	robot_state.setToDefaultValues(wsg_group, "home");

	boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
	planning_interface::PlannerManagerPtr planner_instance;

	std::string planner_plugin_namexD;

	nh.getParam("planning_plugin", planner_plugin_namexD);
	
	ROS_INFO_STREAM(planner_plugin_namexD);

	ROS_INFO("lort");

	try
	{
		planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
	}
	catch (pluginlib::PluginlibException& ex)
	{
		ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
	}

	ROS_INFO("lort");
	/*

	ROS_INFO("lort");

	planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(PLANNER_PLUGIN_NAME));

	ROS_INFO("lort");

	*/


	/*

	{
		// set a desired pose for the end-effector
		geometry_msgs::Pose target_pose;
		target_pose.orientation.w = 1.0;
		target_pose.orientation.x = 0.0;
		target_pose.orientation.y = 0.0;
		target_pose.orientation.z = 0.0;
		target_pose.position.x = 0.3;
		target_pose.position.y = 0.3;
		target_pose.position.z = 0.5;

		// set a targetpose
		move_group.setPoseTarget(target_pose);

		// this is just planning, not moving the robot
		moveit::planning_interface::MoveGroupInterface::Plan my_plan;

		bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

		ROS_INFO_STREAM("The plan was " << success ? "successfull" : "not successfull" );

		visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
		visual_tools.trigger();
	}

	*/

	return 0;
}