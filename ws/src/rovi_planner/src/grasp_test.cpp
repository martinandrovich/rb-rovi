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

constexpr auto ARM_GROUP    	 	= "ur5_arm";
constexpr auto WSG_GROUP 	    	= "wsg";
constexpr auto ROBOT_DESCRIPTION 	= "robot_description";
constexpr auto PLANNER_PLUGIN_NAME 	= "ompl_interface/OMPLPlanner";
constexpr auto PLANNING_SCENE_TOPIC = "/planning_scene_gazebo";
constexpr auto TRAJECTORY_TOPIC     = "/move_group/display_planned_path";
constexpr auto JOINT_POS_TOPIC 		= "/ur5_joint_position_controller/command";
constexpr auto WSG_TORQUE_TOPIC 	= "/wsg_hybrid_controller/command";

const std::vector<double>   HOME_ARM{0, -1.57, 1.57, 1.57, 1.57, 0};
const std::vector<double>   HOME_WSG{0.05, 0.05};
const std::array<double, 3> TABLE_POS{0.4, 0.6, 0.64};
const std::array<double, 3> BOTTLE_POS{0.6, 0.999023, 0.75};
const std::array<double, 3> BASE_OFFSET{0.1, 0.5, 0.75};

int
main(int argc, char** argv)
{
	using namespace rovi_utils;

	ros::init(argc, argv, "traj_test");
	ros::NodeHandle nh;
	ros::AsyncSpinner spin(2);
	spin.start();

	// publisher_scene
	ros::Publisher joint_state_pub	    = nh.advertise<sensor_msgs::JointState>(JOINT_POS_TOPIC, 1);
	ros::Publisher wsg_state_pub 		= nh.advertise<std_msgs::Float64>(WSG_TORQUE_TOPIC, 1);

	// init moveit_object
	rovi_planner::moveit_planner::init(nh);

	auto req = rovi_planner::moveit_planner::traj_moveit(
														make_pose({0.50, 0.40, 0.1, 0, 0, 0}), 
														"RRTConnect"
														);

	rovi_planner::moveit_planner::execution(
											req, 
											joint_state_pub
											);

	rovi_planner::moveit_planner::destruct();

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

	// create joint trajectory using linear interpolation
	// auto joint_states = rovi_utils::joint_states_from_traj(*request.trajectory_);
	// auto traj_joints  = rovi_planner::traj_linear(joint_states, 0.1, 0.1, 0.01);
	// rovi_utils::export_traj(traj_joints, "traj_jnt_rrt_lin.csv");

	// for (int i = 0; request.trajectory_->getWayPointCount() > i; i++)
	// {
	// 	ROS_INFO_STREAM( request.trajectory_->getWayPointDurationFromStart(i));
	// }

	// moveit::core::RobotStatePtr robot_state;


	// terminate planner instance
	// planner_instance->terminate();
	

	//traj_msg.

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
	// 		planning_scene_pub.publish(planning_scene_msg);
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