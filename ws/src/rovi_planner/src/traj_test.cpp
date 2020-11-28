#include <fstream>
#include <tuple>

#include <ros/ros.h>
#include <rovi_planner/rovi_planner.h>
#include <geometry_msgs/Pose.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_kdl.h>
#include <kdl_conversions/kdl_msg.h>
#include <ur5_controllers/PoseTwist.h>

#include <Eigen/Eigen>

#include <rovi_utils/rovi_utils.h>

int
main(int argc, char** argv)
{

	using namespace rovi_utils;

	ros::init(argc, argv, "traj_test");
	ros::NodeHandle nh;

	// joint interpolation test

	sensor_msgs::JointState state1, state2;
	// state1.position = { 0.5, 1.0, 0.45, 3.14, 1.12, 1.23 };
	state1.position = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.5 };
	state2.position = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

	auto traj_joint_lin = rovi_planner::traj_linear({ state1, state2 }, 1.0, 1.0, 0.001);
	rovi_utils::export_traj(traj_joint_lin, "traj_jnt_test_lin.csv", 0.01);
	
	auto traj_joint_par = rovi_planner::traj_parabolic({ state1, state2 }, 1.0, 1.0, 0.001, 0.001);
	rovi_utils::export_traj(traj_joint_par, "traj_jnt_test_par.csv", 0.01);

	// create a linear, parabolic and rrt trajectories

	Eigen::Quaternion ori(1.0, 0.0, 0.0, 0.0);

	auto pose_ee_current = rovi_utils::get_current_tcp_pose();
	auto pose_ee_desired = rovi_utils::make_pose({ 0.1, 0.15, 0.90 }, ori);

	std::vector<geometry_msgs::Pose> waypoints = 
	{
		pose_ee_current,
		pose_ee_desired
	};

	auto traj_lin = rovi_planner::traj_linear(waypoints, 0.1, 0.1, 0.05);
	auto traj_par = rovi_planner::traj_parabolic(waypoints, 0.1, 0.1, 0.05, 0.5);
	//auto traj_rrt = rovi_planner::traj_moveit(pose_ee_desired, "RRTstar");

	// export to file
	rovi_utils::export_traj(traj_lin, "traj_lin.csv", 0.01);
	rovi_utils::export_traj(traj_par, "traj_par.csv", 0.01);
	//rovi_utils::export_traj(traj_rrt, "traj_rrt.csv", 0.01);

	// // command trajectory to robot at 100 Hz
	// std::cout << "Press [ENTER] to execute trajectory...\n";
	// std::cin.ignore();

	// const auto pub = nh.advertise<ur5_controllers::PoseTwist>("/ur5_cartesian_pose_controller/command", 1);
	// ur5_controllers::PoseTwist msg;
	// ros::Rate lr(100); // Hz

	// for (auto [t, traj] = std::tuple{ 0.0, traj_rrt }; t < traj.Duration() and ros::ok(); t += 0.01)
	// {
		
	// 	ROS_INFO_STREAM_ONCE("Executing trajectory with duration: " << traj.Duration() << " sec");

	// 	// KDL Frame
	// 	const auto& frame = traj.Pos(t);
	// 	const auto& twist = traj.Vel(t);

	// 	// Convert from KDL to Eigen
	// 	tf::poseKDLToMsg(frame, msg.pose);
	// 	tf::twistKDLToMsg(twist, msg.twist);

	// 	pub.publish(msg);

	// 	lr.sleep();
	// }

	return 0;
}