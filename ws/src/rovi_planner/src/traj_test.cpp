#include <fstream>

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

	// create a parabolic trajectory

	Eigen::Quaternion ori(1.0, 0.0, 0.0, 0.0);
	Eigen::Quaternion ori2(0.707, 0.707, 0.0, 0.0);
	Eigen::Quaternion ori3(0.707, 0.0, 0.707, 0.0);

	std::vector<geometry_msgs::Pose> waypoints = 
	{
		make_pose({ 0.09, 0.12, 0.90 }, ori),
		make_pose({ 0.50, 0.26, 0.50 }, ori),
		// make_pose({ 0.07, 0.50, 0.50 }, ori2),
		// make_pose({ 0.09, 0.12, 0.90 }, ori),
		// make_pose({ 0.52, 0.26, 0.90 }, ori),
		// make_pose({ 0.30, 0.56, 0.50 }, ori),
		// make_pose({ 0.60, 0.99, 0.74 }, ori),
	};

	auto traj = rovi_planner::traj_parabolic(waypoints, 0.1, 0.1, 0.05, 0.05);
	// auto traj = rovi_planner::traj_linear(waypoints, 0.1, 0.1);

	ROS_INFO_STREAM("Generated trajectory with duration: " << traj.Duration() << " sec");

	// export to file
	rovi_utils::export_traj(traj, "test.dat", 0.01);

	// command trajectory to robot at 100 Hz

	std::cout << "Press [ENTER] to execute trajectory...\n";
	std::cin.ignore();

	const auto pub = nh.advertise<ur5_controllers::PoseTwist>("/ur5_cartesian_pose_controller/command", 1);
	ur5_controllers::PoseTwist msg;
	ros::Rate lr(100); // Hz

	for (double t = 0.0; t < traj.Duration(); t += 0.01)
	{
		// KDL Frame
		const auto& frame = traj.Pos(t);
		const auto& twist = traj.Vel(t);

		// Convert from KDL to Eigen
		tf::poseKDLToMsg(frame, msg.pose);
		tf::twistKDLToMsg(twist, msg.twist);

		pub.publish(msg);

		lr.sleep();
	}

	return 0;
}