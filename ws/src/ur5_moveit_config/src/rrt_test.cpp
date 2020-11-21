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
#include <rovi_planner/rovi_planner.h>

// arm and robot_description
constexpr auto ARM_GROUP    	 	= "ur5_arm";
constexpr auto WSG_GROUP 	    	= "wsg";
constexpr auto ROBOT_DESCRIPTION 	= "robot_description";
constexpr auto PLANNER_PLUGIN_NAME 	= "ompl_interface/OMPLPlanner";

//https://github.com/ros-planning/moveit_tutorials/blob/master/doc/creating_moveit_plugins/lerp_motion_planner/src/lerp_example.cpp

void 
probabalistic_planner();

int
main(int argc, char** argv)
{
	// init node
	ros::init(argc, argv, "rrt_test");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	// display publisher
	ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

	// load robot model and kinematic model, and use it to setup the planning scene
    robot_model_loader::RobotModelLoaderPtr robot_model_loader(new robot_model_loader::RobotModelLoader(ROBOT_DESCRIPTION));
    robot_model::RobotModelPtr robot_model(robot_model_loader->getModel());

    // set up the planning scene for collision detection
    planning_scene::PlanningScenePtr planning_scene( new planning_scene::PlanningScene(robot_model) );

	// visualization in rviz
	moveit_visual_tools::MoveItVisualTools visual_tools("ur5_link0");
	visual_tools.loadRobotStatePub("/disp_robot_state");
	visual_tools.enableBatchPublishing();
	visual_tools.deleteAllMarkers();
	visual_tools.trigger();
	
    // get robot state, this is a raw reference and pointers for arm_group and wsg_group
    auto& robot_state    = planning_scene->getCurrentStateNonConst();
    const auto arm_group = robot_state.getJointModelGroup(ARM_GROUP);
    const auto wsg_group = robot_state.getJointModelGroup(WSG_GROUP);

	// set gripper default
    std::vector gripper_state{0.05, 0.05};
    robot_state.setJointGroupPositions(wsg_group, gripper_state);

	// make a scene message to update scene
	moveit_msgs::PlanningScene planning_scene_msg;

	// move the base of the robot
	// rovi_utils::move_base(robot_state, {0.1, 0.1, 0.75});

	// insert table and other constant objects into the planning scene
	std::vector<moveit_msgs::CollisionObject> collision_objects
	{
		rovi_utils::make_mesh_cobj("table",  planning_scene->getPlanningFrame() , {0.4, 0.6, 0.64}),
		rovi_utils::make_mesh_cobj("bottle", planning_scene->getPlanningFrame() , {0.6, 0.999023, 0.75})
	};

	// set default state
	robot_state.setToDefaultValues(arm_group, "home");
	robot_state.setToDefaultValues(wsg_group, "home");

	// setup a plugin

	boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
	planning_interface::PlannerManagerPtr planner_instance;
	planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
	planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(PLANNER_PLUGIN_NAME));

	// success ?

	if (!planner_instance->initialize(robot_model, nh.getNamespace()))
	{
    	ROS_FATAL_STREAM("Could not initialize planner instance");
		return -1;
	}

	ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");

	planning_interface::MotionPlanRequest req;
	planning_interface::MotionPlanResponse res;

	// desired pose
	geometry_msgs::PoseStamped pose;

	// w.r.t this frame - default value
	pose.header.frame_id = "ur5_link0";

	// end-effector
	pose.pose.position.x = 0.3;
	pose.pose.position.y = 0.4;
	pose.pose.position.z = 0.75;
	pose.pose.orientation.w = 1.0;

	// specify the tolerances
	std::vector<double> tol_pos(3, 0.005);
	std::vector<double> tol_ori(3, 0.005);

	// kinematic_constraint
	moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints("ee_tcp", pose, tol_pos, tol_ori);

	// specify plan group and pushback the goal_constraints?
	req.group_name = ARM_GROUP;
	req.goal_constraints.push_back(pose_goal);

	planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
	context->solve(res);

	if (res.error_code_.val != res.error_code_.SUCCESS)
	{
		ROS_ERROR("Could not compute plan successfully");
		return 0;
	}

	// export waypoints to file
	rovi_utils::export_traj(*res.trajectory_, "traj_rrt_waypoints.csv");

	// create trajectory using linear interpolation + export to file
	auto waypoints = rovi_utils::waypoints_from_traj(*res.trajectory_);
	auto traj_lin = rovi_planner::traj_linear(waypoints, 0.1, 0.1, 0.05);
	rovi_utils::export_traj(traj_lin, "traj_rrt_lin.csv");

	// put the orbot into 
	visual_tools.publishRobotState(robot_state, rviz_visual_tools::GREEN);
	visual_tools.trigger();
	
	// visualize the result
	moveit_msgs::DisplayTrajectory display_trajectory;

	// prep for visualize the trajectory
	moveit_msgs::MotionPlanResponse response;
	res.getMessage(response);

	display_trajectory.trajectory_start = response.trajectory_start;
	display_trajectory.trajectory.push_back(response.trajectory);
	visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), arm_group);
	visual_tools.trigger();
	display_publisher.publish(display_trajectory);

	//  set the state in the planning scene to the final state of the last plan
	robot_state.setJointGroupPositions(arm_group, response.trajectory.joint_trajectory.points.back().positions);

	visual_tools.publishRobotState(robot_state, rviz_visual_tools::GREEN);
	visual_tools.publishAxisLabeled(pose.pose, "goal_2");
	visual_tools.trigger();

	return 0;
}

void 
probabalistic_planner()
{

}