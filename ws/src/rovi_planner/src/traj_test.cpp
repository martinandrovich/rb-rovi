#include <fstream>

#include <ros/ros.h>
#include <rovi_planner/rovi_planner.h>
#include <geometry_msgs/Pose.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_kdl.h>

#include <Eigen/Eigen>

int
main(int argc, char** argv)
{
	// define ROS node
	// https://yuzhangbit.github.io/tools/several-ways-of-writing-a-ros-node/

	ros::init(argc, argv, "traj_test");
	ros::NodeHandle nh;

	// create a parabolic trajectory
	
	Eigen::Quaternion ori(0.706825, 0.0, 0.706825, 0.0);
	// Eigen::Quaternion ori(0.0, 0.0, 0.0, 0.0);
	// Eigen::Quaternion ori(-0.3273, -0.6513, -0.3273, -0.6012);

	std::vector<geometry_msgs::Pose> waypoints = 
	{
		make_pose({ 0.12, 0.26, 0.90 }, ori),
		make_pose({ 0.15, 0.26, 0.90 }, ori),
		make_pose({ 0.20, 0.26, 0.90 }, ori),
		// make_pose({ 0.52, 0.26, 0.90 }, ori),
		// make_pose({ 0.30, 0.56, 0.50 }, ori),
		// make_pose({ 0.60, 0.99, 0.74 }, ori),
	};

	auto traj = rovi_planner::traj_parabolic(waypoints);

	ROS_INFO_STREAM("Generated trajectory with duration: " << traj.Duration() << " sec");

	auto fs = std::ofstream("traj_test.dat", std::ofstream::out);
	for (double t = 0.0; t < traj.Duration(); t += 0.01)
	{

		const auto& frame = traj.Pos(t);
		auto T = Eigen::Affine3d();
	
		tf::transformKDLToEigen(frame, T);

		// create flattened vector from Eigen (row major)
		auto T_ = T.matrix();
		// std::cout << T_ << "\n\n";
		T_.transposeInPlace();
		Eigen::VectorXd v(Eigen::Map<Eigen::VectorXd>(T_.data(), T_.size()));
		
		for (size_t i = 0; i < v.size(); ++i)
			fs << v(i) << ((i != v.size() - 1) ? ", " : "");

		fs << "\n";
	}

	return 0;
}