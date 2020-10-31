#include <ros/ros.h>
#include <rovi_planner/rovi_planner.h>
#include <geometry_msgs/Pose.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_kdl.h>

#include <Eigen/Eigen>

int
main(int argc, char** argv)
{
	ros::init(argc, argv, "goto_pose");
	ros::NodeHandle nh;

	// get current pose
	// ros::topic::waitForMessage("")
	// // define waypoints
	// // generate trajectory
	// // publish

	// std::vector<geometry_msgs::Pose> waypoints = 
	// {
	// 	make_pose({ 0.12, 0.26, 0.90 }, ori),
	// 	// make_pose({ 0.15, 0.26, 0.90 }, ori),
	// 	make_pose({ 0.20, 0.26, 0.90 }, ori2),
	// 	// make_pose({ 0.52, 0.26, 0.90 }, ori),
	// 	// make_pose({ 0.30, 0.56, 0.50 }, ori),
	// 	// make_pose({ 0.60, 0.99, 0.74 }, ori),
	// };

	// auto traj = rovi_planner::traj_parabolic(waypoints);

	// ROS_INFO_STREAM("Generated trajectory with duration: " << traj.Duration() << " sec");

	// for (double t = 0.0; t < traj.Duration(); t += 0.01)
	// {
	// }

	return 0;
}