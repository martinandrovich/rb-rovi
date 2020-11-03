#include <ros/ros.h>
#include <ur5_dynamics/ur5_dynamics.h>
#include <wsg_dynamics/wsg_dynamics.h>

int
main(int argc, char** argv)
{
	// define ROS node
	// https://yuzhangbit.github.io/tools/several-ways-of-writing-a-ros-node/

	ros::init(argc, argv, "test_node");

	ros::NodeHandle nh;

	/*

	Eigen::Vector6d q = (Eigen::Vector6d() << 0, -1.57, 0, 0, 0, 0).finished();

    Eigen::Matrix4d T = ur5_dynamics::fwd_kin<Eigen::Matrix4d>(q);

	geometry_msgs::Pose T_geo = ur5_dynamics::fwd_kin<geometry_msgs::Pose>(q);

	std::cout << T << std::endl;

	std::cout << T_geo << std::endl;

	std::cout << ur5_dynamics::inv_kin(T, q).transpose() << std::endl;

	std::cout << ur5_dynamics::inv_kin<geometry_msgs::Pose>(T_geo, q).transpose() << std::endl;

	// this jacobian is only for ur5_end-effector!!
	Eigen::Matrix<double, 6, 6> jac = ur5_dynamics::jac(q);

	Eigen::Matrix<double, 6, 6> pinv_jac = ur5_dynamics::pinv_jac(jac);

	std::cout << jac*pinv_jac << std::endl;

	std::cout << jac << std::endl;

	requires some qdot
	Eigen::MatrixXd jac_dot = ur5_dynamics::jac_dot(q, q);
	
	*/

	wsg_dynamics::init();

	return 0;
}