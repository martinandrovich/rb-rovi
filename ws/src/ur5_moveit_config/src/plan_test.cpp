#include <ros/ros.h>
#include <gazebo_msgs/LinkStates.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/CollisionObject.h>

#include <rovi_utils/rovi_utils.h>

// arm and robot_description
const std::string PLANNING_GROUP    = "ur5_arm";
const std::string ROBOT_DESCRIPTION = "robot_description";

int
main(int argc, char** argv)
{
	using namespace rovi_utils;
	
	// init node
	ros::init(argc, argv, "plan_test");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	// load robot model
	robot_model_loader::RobotModelLoaderPtr robot_model_loader(new robot_model_loader::RobotModelLoader(ROBOT_DESCRIPTION));
	robot_model::RobotModelPtr robot_kinematic_model = robot_model_loader->getModel();

	robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_kinematic_model));
	// robot_state->setToDefaultValues();

	// get joint model group
	Eigen::VectorXd q;
	const robot_state::JointModelGroup* joint_model_group = robot_kinematic_model->getJointModelGroup(PLANNING_GROUP);

	// // get and print joint values
	// robot_state->copyJointGroupPositions(joint_model_group, q);
	// ROS_INFO_STREAM(q);

	// robot_state->setToRandomPositions();

	// // get and print joint values
	// robot_state->copyJointGroupPositions(joint_model_group, q);
	// ROS_INFO_STREAM(q);

	// we we will use movegroupinterface to interface the planning group
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
	auto state = move_group.getCurrentState();
	state->printStatePositions(); std::cout << "\n\n";
	state->setToRandomPositions();
	state->printStatePositions();

	move_group.setStartState(*state);
	geometry_msgs::Pose target_pose1;
	target_pose1.orientation.w = 1.0;
	target_pose1.position.x = 0.28;
	target_pose1.position.y = -0.2;
	target_pose1.position.z = 0.5;
	move_group.setPoseTarget(target_pose1);

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	move_group.plan(my_plan);
	move_group.execute(my_plan);

	return 0;

	// move_group.plan()

	// we will use planning-scene interface to add and remove collision objects in our virtual world
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	// raw pointer to joint model group
	// const auto joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

	// add collision objects
	std::vector<moveit_msgs::CollisionObject> collision_objects
	{
		make_mesh_cobj("table",  move_group.getPlanningFrame() , {0.4, 0.6, 0.64}),
		make_mesh_cobj("bottle", move_group.getPlanningFrame() , {0.5,   1, 0.75})
	};

	planning_scene_interface.addCollisionObjects(collision_objects);

	

	// set start state
	// robot_state::RobotState start_state(*move_group.getCurrentState());
	// geometry_msgs::Pose start_pose;
	// start_pose.orientation.w = 1.0;
	// start_pose.position.x = 0.55;
	// start_pose.position.y = -0.05;
	// start_pose.position.z = 0.8;
	// start_state.setFromIK(joint_model_group, start_pose);

	// move_group.setStartState(start_state);

	// set target state
	geometry_msgs::Pose target_pose;
	target_pose.orientation.w = 1.0;
	target_pose.position.x = 0.28;
	target_pose.position.y = -0.2;
	target_pose.position.z = 0.5;

	move_group.setPoseTarget(target_pose);

	return 0;
}