#include <fstream>
#include <tuple>

#include <ros/ros.h>
#include <rovi_planner/rovi_planner.h>
#include <geometry_msgs/Pose.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_kdl.h>
#include <ur5_controllers/PoseTwist.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit_msgs/Grasp.h>

#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotState.h>

#include <pluginlib/class_loader.h>
#include <boost/scoped_ptr.hpp>

#include <Eigen/Eigen>

#include <rovi_utils/rovi_utils.h>

constexpr auto ARM_GROUP    	 	= "ur5_arm";
constexpr auto WSG_GROUP 	    	= "wsg";
constexpr auto ROBOT_DESCRIPTION 	= "robot_description";
constexpr auto PLANNER_PLUGIN_NAME 	= "ompl_interface/OMPLPlanner";
constexpr auto PLANNING_SCENE_TOPIC = "/planning_scene_gazebo";
constexpr auto TRAJECTORY_TOPIC     = "/move_group/display_planned_path";

const std::vector<double> HOME_ARM{0, -1.57, 1.57, 1.57, 1.57, 0};
const std::vector<double> HOME_WSG{0.05, 0.05};
const std::array<double, 3> TABLE_POS{0.4, 0.6, 0.64};
const std::array<double, 3> BOTTLE_POS{0.6, 0.999023, 0.75};
const std::array<double, 3> BASE_OFFSET{0.1, 0.5, 0.75};




int
main(int argc, char** argv)
{
	using namespace rovi_utils;

	ros::init(argc, argv, "traj_test");
	ros::NodeHandle nh;

	// publisher_scene
	ros::Publisher planning_scene_pub 	= nh.advertise<moveit_msgs::PlanningScene>(PLANNING_SCENE_TOPIC, 1);
	ros::Publisher traj_pub 			= nh.advertise<moveit_msgs::DisplayTrajectory>(TRAJECTORY_TOPIC, 1);

	// set the planner
	ros::param::set("/ur5_arm/default_planner_config", "RRTstar");

	// create the model of the robot
	robot_model_loader::RobotModelLoaderPtr robot_model_loader(new robot_model_loader::RobotModelLoader(ROBOT_DESCRIPTION));
    robot_model::RobotModelPtr 				robot_model(robot_model_loader->getModel());

	// create the planning-scene
    planning_scene::PlanningScenePtr planning_scene( new planning_scene::PlanningScene(robot_model) );

	// get robot state, this is a raw reference and pointers for arm_group and wsg_group
    auto& robot_state    = planning_scene->getCurrentStateNonConst();
    const auto arm_group = robot_state.getJointModelGroup(ARM_GROUP);
    const auto wsg_group = robot_state.getJointModelGroup(WSG_GROUP);

	// set the robot state
	robot_state.setJointGroupPositions(ARM_GROUP, HOME_ARM);
	robot_state.setJointGroupPositions(WSG_GROUP, HOME_WSG);

	rovi_utils::move_base(robot_state, BASE_OFFSET);

	// add the collisions to the scene
	std::vector<moveit_msgs::CollisionObject> collision_objects
	{
		rovi_utils::make_mesh_cobj("table",  planning_scene->getPlanningFrame() , TABLE_POS),
		rovi_utils::make_mesh_cobj("bottle", planning_scene->getPlanningFrame() , BOTTLE_POS)
	};

	ROS_INFO_STREAM("Size of collision_objects: " << collision_objects.size());

	moveit_msgs::PlanningScene planning_scene_msg;
	planning_scene->getPlanningSceneMsg(planning_scene_msg);
	planning_scene_msg.world.collision_objects = collision_objects;
	planning_scene->setPlanningSceneMsg(planning_scene_msg);

	// setup the runtime plugin
	boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
	planning_interface::PlannerManagerPtr planner_instance;
	planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
	planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(PLANNER_PLUGIN_NAME));

	// check for success
	if (!planner_instance->initialize(robot_model, ros::this_node::getNamespace()))
	{	
    	ROS_FATAL_STREAM("Could not initialize planner instance");
		return -1;
	}

	// make a motion plan request / response msg
	planning_interface::MotionPlanRequest req;
	planning_interface::MotionPlanResponse res;

	// // desired pose with timestamp
	geometry_msgs::PoseStamped pose;
	pose.header.frame_id = "ur5_link0";
	pose.pose = make_pose({0.5, 0.499, 0.1, 0, 0, 0});

	ROS_INFO_STREAM("\n---------------------------\nThe desired grasp pose: " << pose << "---------------------------");

	// specify the tolerances for the pose
	std::vector<double> tol_pos(3, 0.005);
	std::vector<double> tol_ori(3, 0.005);

	// set the kinematic_constraint
	moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints("ee_tcp", pose, tol_pos, tol_ori);

	// specify plan group and pushback the goal_constraints?
	req.group_name = ARM_GROUP;
	req.allowed_planning_time = 1;
	req.num_planning_attempts = 5000;
	req.max_acceleration_scaling_factor = 1.0;
	req.max_velocity_scaling_factor = 1.0;
	req.goal_constraints.push_back(pose_goal);

	// get the planningcontext
	planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
	context->solve(res);

	if (res.error_code_.val != res.error_code_.SUCCESS)
	{
		ROS_FATAL_STREAM("It was not possible to generate a trajectory");
	}

	// trajectory display
	moveit_msgs::DisplayTrajectory traj_msg;
	moveit_msgs::MotionPlanResponse res_msg;
	res.getMessage(res_msg);

	// incorporate
	traj_msg.trajectory_start = res_msg.trajectory_start;
	traj_msg.trajectory.push_back(res_msg.trajectory);

	// set the robot state to the desided pose
	auto & q_last = res_msg.trajectory.joint_trajectory.points.back();
	robot_state.setJointGroupPositions(arm_group, q_last.positions);

	// collision_objects.pop_back();

	// see if it is actuallyed updated
	planning_scene->getPlanningSceneMsg(planning_scene_msg);
	planning_scene_msg.world.collision_objects = collision_objects;
	planning_scene->setPlanningSceneMsg(planning_scene_msg);

	auto execute = [&](double duration)
	{
		ros::Time begin = ros::Time::now();
		ros::Duration timeout(duration);
		ros::Rate lp(2);
		while(ros::Time::now() - begin < timeout)
		{
			traj_pub.publish(traj_msg);
			planning_scene_pub.publish(planning_scene_msg);
			lp.sleep();
		}
	};

	execute(2);

	// attach the object to the end-effector
	moveit_msgs::AttachedCollisionObject object_to_attach;
	object_to_attach.link_name = "ee_tcp";
	object_to_attach.object = collision_objects[1];
	object_to_attach.object.operation = object_to_attach.object.ADD;

	// update the planning scene
	planning_scene->getPlanningSceneMsg(planning_scene_msg);
	collision_objects[1].operation = collision_objects[1].REMOVE;
	planning_scene_msg.world.collision_objects = collision_objects;
	planning_scene_msg.robot_state.attached_collision_objects.push_back(object_to_attach);
	planning_scene->setPlanningSceneMsg(planning_scene_msg);

	execute(2);

	// make a plan
	pose.pose = make_pose({0.52, -0.35, 0.1, 0, 0, 0});
	// pose.pose = make_pose({0.5, 0.2, 0.3, 0, 0, 0});

	ROS_INFO_STREAM("\n---------------------------\nThe desired grasp pose: " << pose << "---------------------------");

	// set the kinematic_constraint
	pose_goal = kinematic_constraints::constructGoalConstraints("ee_tcp", pose, tol_pos, tol_ori);

	// specify plan group and pushback the goal_constraints?
	req.allowed_planning_time = 1;
	req.num_planning_attempts = 10000;
	req.goal_constraints.clear();
	req.goal_constraints.push_back(pose_goal);

	// get the planningcontext
	context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
	context->solve(res);

	if (res.error_code_.val != res.error_code_.SUCCESS)
	{
		ROS_FATAL_STREAM("It was not possible to generate a trajectory");
		return -1;
	}

	// trajectory display
	res.getMessage(res_msg);

	// incorporate
	traj_msg.trajectory_start = res_msg.trajectory_start;
	traj_msg.trajectory.clear();
	traj_msg.trajectory.push_back(res_msg.trajectory);

	//q_last = res_msg.trajectory.joint_trajectory.points.back();
	//robot_state.setJointGroupPositions(arm_group, q_last.positions);

	execute(5);



	return 0;
}