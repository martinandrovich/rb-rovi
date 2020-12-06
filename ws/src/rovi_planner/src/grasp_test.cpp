#include <fstream>
#include <tuple>

#include <ros/ros.h>
#include <rovi_planner/rovi_planner.h>
#include <geometry_msgs/Pose.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_kdl.h>

#include <moveit_msgs/Grasp.h>

#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotState.h>

#include <std_msgs/Float64.h>
#include <trajectory_interface/trajectory_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <thread>

#include <rovi_utils/rovi_utils.h>
#include <rovi_gazebo/rovi_gazebo.h>
#include <ur5_controllers/interface.h>

constexpr auto ARM_GROUP            = "ur5_arm";
constexpr auto WSG_GROUP            = "wsg";
constexpr auto ROBOT_DESCRIPTION    = "robot_description";
constexpr auto PLANNER_PLUGIN_NAME  = "ompl_interface/OMPLPlanner";
constexpr auto PLANNING_SCENE_TOPIC = "/planning_scene_gazebo";
constexpr auto TRAJECTORY_TOPIC     = "/move_group/display_planned_path";
constexpr auto FREQ                 = 1000.0; // Hz

constexpr auto BASE_OFFSET          = std::array{ 0.1, 0.5, 0.75 };
constexpr auto POS_TABLE            = std::array{ 0.4, 0.6, 0.64 };
constexpr auto POSE_BOTTLE_PICK     = std::array{  0.50,  0.53, 0.10, 0.0, 0.0, 0.0 };
constexpr auto POSE_BOTTLE_PLACE    = std::array{ 0.575, -0.35, 0.12, 0.0, 0.0, 0.0 };

int
main(int argc, char** argv)
{
	using namespace rovi_utils;

	// init node
	ros::init(argc, argv, "grasp_test");
	ros::NodeHandle nh;
	
	// image
	auto imgs = rovi_gazebo::get_stereo_camera_imgs();
	cv::imshow("left", imgs["left"]);
	cv::waitKey(0);

	// init moveit planner
	ROS_INFO_STREAM("Initializing moveit planner and scene...");
	rovi_planner::moveit_planner::init(nh);
	rovi_planner::moveit_planner::update_planning_scene();
	rovi_planner::moveit_planner::start_planning_scene_publisher();
	
	// define poses
	auto pose_bottle_pick  = make_pose(POSE_BOTTLE_PICK);
	auto pose_bottle_place = make_pose(POSE_BOTTLE_PLACE);
	
	// PICK
	{
	
	// disable projector
	ROS_INFO_STREAM("Disabling projector...");
	rovi_gazebo::set_projector(false);
	ros::Duration(1).sleep();

	// release gripper
	ROS_INFO_STREAM("Releasing gripper...");
	ur5_controllers::wsg::release();
	ros::Duration(1).sleep();

	// update moving planning scene from gazebo
	ROS_INFO_STREAM("Updating planning scene...");
	rovi_planner::moveit_planner::update_planning_scene();

	// plan using moveit (to object)
	ROS_INFO_STREAM("Planning...");
	auto plan = rovi_planner::moveit_planner::plan(pose_bottle_pick, "RRTstar");

	// get parabolic trajectory (time optimal)
	ROS_INFO_STREAM("Creating trajectory (interpolation)...");
	auto traj = rovi_planner::moveit_planner::get_traj(plan, 0.0001, 1/FREQ, 0.2, 0.1);

	// execute in Gazebo
	ROS_INFO_STREAM("Executing trajectory in Gazebo...");
	ur5_controllers::ur5::execute_traj(traj, FREQ);

	// update moving planning scene from gazebo
	ROS_INFO_STREAM("Updating planning scene...");
	rovi_planner::moveit_planner::update_planning_scene();

	// grasp
	ROS_INFO_STREAM("Grasping object...");
	ur5_controllers::wsg::grasp();
	ros::Duration(2).sleep();

	// update moving planning scene from gazebo
	ROS_INFO_STREAM("Updating planning scene...");
	rovi_planner::moveit_planner::update_planning_scene();

	// attach object (and implcit update)
	ROS_INFO_STREAM("Attaching object to gripper...");
	rovi_planner::moveit_planner::attach_object_to_ee("bottle");
	
	}
	
	// PLACE
	{
	
	// plan using moveit (to to place)
	ROS_INFO_STREAM("Planning...");
	auto plan = rovi_planner::moveit_planner::plan(pose_bottle_place, "RRTstar");

	// get parabolic trajectory (time optimal)
	ROS_INFO_STREAM("Creating trajectory (interpolation)...");
	auto traj = rovi_planner::moveit_planner::get_traj(plan, 0.0001, 1/FREQ, 0.2, 0.1);
	
	// execute in Gazebo
	ROS_INFO_STREAM("Executing trajectory in Gazebo...");
	ur5_controllers::ur5::execute_traj(traj, FREQ);
	ros::Duration(1).sleep();
	
	// update moving planning scene from gazebo
	ROS_INFO_STREAM("Updating planning scene...");
	rovi_planner::moveit_planner::update_planning_scene();
	ros::Duration(1).sleep();
	
	// release gripper
	ROS_INFO_STREAM("Releasing gripper...");
	ur5_controllers::wsg::release();
	ros::Duration(1).sleep();
	
	// update moving planning scene from gazebo
	ROS_INFO_STREAM("Updating planning scene...");
	rovi_planner::moveit_planner::update_planning_scene();
	ros::Duration(1).sleep();
	
	}
	
	// HOME
	{
	
	ROS_INFO_STREAM("Preparing to go home...");
	ros::Duration(2).sleep();

	// plan using moveit (to object)
	ROS_INFO_STREAM("Planning...");
	auto plan = rovi_planner::moveit_planner::plan(pose_bottle_pick, "RRTstar");

	// get parabolic trajectory (time optimal)
	ROS_INFO_STREAM("Creating trajectory (interpolation)...");
	auto traj = rovi_planner::moveit_planner::get_traj(plan, 0.0001, 1/FREQ, 0.2, 0.1);

	// execute in Gazebo
	ROS_INFO_STREAM("Executing trajectory in Gazebo...");
	ur5_controllers::ur5::execute_traj(traj, FREQ);

	// update moving planning scene from gazebo
	ROS_INFO_STREAM("Updating planning scene...");
	rovi_planner::moveit_planner::update_planning_scene();
	
	}

	// exit
	std::cout << "\nPress any key to continue..." << std::endl;
	std::cin.ignore();
	rovi_planner::moveit_planner::destruct();
	exit(0);















	//rovi_utils::export_traj(traj_joints, "traj_jnt_rrt_lin.csv");

	// trajectory display
	// moveit_msgs::DisplayTrajectory traj_msg;
	// moveit_msgs::MotionPlanResponse res_msg;
	// request.getMessage(res_msg);

	// // incorporate
	// traj_msg.trajectory_start = res_msg.trajectory_start;
	// traj_msg.trajectory.push_back(res_msg.trajectory);

	// // publish in rviz
	// traj_pub.publish(traj_msg);

	// moveit::core::RobotStatePtr robot_state;


	// traj_msg.

	// set the robot state to the desided pose
	// auto & q_last = res_msg.trajectory.joint_trajectory.points.back();
	// robot_state.setJointGroupPositions("arm_group", q_last.positions);

	// // collision_objects.pop_back();

	// // see if it is actuallyed updated
	// planning_scene->getPlanningSceneMsg(planning_scene_msg);
	// planning_scene_msg.world.collision_objects = collision_objects;
	// planning_scene->setPlanningSceneMsg(planning_scene_msg);

	// auto execute = [&](double duration)
	// {
	// 	ros::Time begin = ros::Time::now();
	// 	ros::Duration timeout(duration);
	// 	ros::Rate lp(2);
	// 	while(ros::Time::now() - begin < timeout)
	// 	{
	// 		traj_pub.publish(traj_msg);
	// 		pub_planning_scene.publish(planning_scene_msg);
	// 		lp.sleep();
	// 	}
	// };

	// execute(2);

	// // attach the object to the end-effector
	// moveit_msgs::AttachedCollisionObject object_to_attach;
	// object_to_attach.link_name = "ee_tcp";
	// object_to_attach.object = collision_objects[1];
	// object_to_attach.object.operation = object_to_attach.object.ADD;

	// // update the planning scene
	// planning_scene->getPlanningSceneMsg(planning_scene_msg);
	// collision_objects[1].operation = collision_objects[1].REMOVE;
	// planning_scene_msg.world.collision_objects = collision_objects;
	// planning_scene_msg.robot_state.attached_collision_objects.push_back(object_to_attach);
	// planning_scene->setPlanningSceneMsg(planning_scene_msg);

	// execute(2);

	// // make a plan
	// pose.pose = make_pose({0.52, -0.35, 0.1, 0, 0, 0});
	// // pose.pose = make_pose({0.5, 0.2, 0.3, 0, 0, 0});

	// ROS_INFO_STREAM("\n---------------------------\nThe desired grasp pose: " << pose << "---------------------------");

	// // set the kinematic_constraint
	// pose_goal = kinematic_constraints::constructGoalConstraints("ee_tcp", pose, tol_pos, tol_ori);

	// // specify plan group and pushback the goal_constraints?
	// req.allowed_planning_time = 1;
	// req.num_planning_attempts = 10000;
	// req.goal_constraints.clear();
	// req.goal_constraints.push_back(pose_goal);

	// // get the planningcontext
	// context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
	// context->solve(res);

	// if (res.error_code_.val != res.error_code_.SUCCESS)
	// {
	// 	ROS_FATAL_STREAM("It was not possible to generate a trajectory");
	// 	return -1;
	// }

	// // trajectory display
	// res.getMessage(res_msg);

	// // incorporate
	// traj_msg.trajectory_start = res_msg.trajectory_start;
	// traj_msg.trajectory.clear();
	// traj_msg.trajectory.push_back(res_msg.trajectory);

	// //q_last = res_msg.trajectory.joint_trajectory.points.back();
	// //robot_state.setJointGroupPositions(arm_group, q_last.positions);

	// execute(5);



	return 0;
}