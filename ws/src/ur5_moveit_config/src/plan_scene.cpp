#include <gazebo_msgs/LinkStates.h>
#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PlanningScene.h>

#include <rovi_utils/rovi_utils.h>

// arm and robot_description
constexpr auto PLANNING_GROUP = "ur5_arm";
constexpr auto ROBOT_DESCRIPTION = "robot_description";
constexpr auto SCENE_TOPIC = "/planning_scene";

int
main(int argc, char** argv)
{
	using namespace rovi_utils;

	// init node
	ros::init(argc, argv, "plan_test");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	// publish for planning_scene diffs
	ros::Publisher pub_planning_scene_diff = nh.advertise<moveit_msgs::PlanningScene>(SCENE_TOPIC, 1);

	// load model from robot description
	robot_model_loader::RobotModelLoader robot_model_loader(ROBOT_DESCRIPTION);
	robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

	// create a planning scene which maintains the state of the world (including the robot)
	planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

	// create a RobotState and JointModelGroup to keep track of the current robot pose and planning group
	auto& robot_state = planning_scene->getCurrentStateNonConst();
	const robot_state::JointModelGroup* joint_model_group = robot_state.getJointModelGroup(PLANNING_GROUP);

	// --------------------------------------------------------------------------------------------------------

	// configure to valid state
	// planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");

	// move robot base
	rovi_utils::move_base(robot_state, { 0.5, 0.5, 0.70 });
	// robot_state.update();
	// robot_state.printStatePositions();
	
	// create planning scene msg from planning scene obj
	moveit_msgs::PlanningScene planning_scene_msg;
	planning_scene->getPlanningSceneMsg(planning_scene_msg);

	// keep publishing
	ros::Rate lp(10);
	while(ros::ok())
	{
		pub_planning_scene_diff.publish(planning_scene_msg);
		lp.sleep();
	}

	return 0;
}