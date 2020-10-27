#include <ros/ros.h>
#include <ur5_dynamics/ur5_dynamics.h>

int
main(int argc, char** argv)
{
	// define ROS node
	// https://yuzhangbit.github.io/tools/several-ways-of-writing-a-ros-node/

	ros::init(argc, argv, "example_node");

	ros::NodeHandle nh;

	Eigen::Vector6d q = (Eigen::Vector6d() << 0, -1.57, 0, 0, 0, 0).finished();

    Eigen::Matrix4d T = ur5_dynamics::fwd_kin(q);

	std::cout << ur5_dynamics::inv_kin(T, q).transpose() << std::endl;

	Eigen::MatrixXd jac = ur5_dynamics::geometric_jacobian(q);

	std::cout << jac << std::endl;

	Eigen::MatrixXd jac_dot = ur5_dynamics::geometric_jacobian_dot(q, q);

	return 0;
}