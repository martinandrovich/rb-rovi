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
	std::cout << q.transpose() << std::endl;
    std::cout << ur5_dynamics::fwd_kin(q) << std::endl;

    Eigen::Matrix4d T = ur5_dynamics::fwd_kin(q);

	Eigen::Matrix4d T_x, T_cord;

	//std::cout << T << std::endl;

	T_x <<  1, 0, 0, 0,
			0, 0, 1, 0,
			0, -1, 0, 0,
			0, 0, 0, 1;

	T_cord << 	-1, 0, 0, 0,
				0, -1, 0, 0,
				0, 0, 1, 0,
				0, 0, 0, 1;

	std::cout << T_cord * T * T_x << std::endl;

	std::cout << ur5_dynamics::inv_kin(T_cord * T * T_x, q) << std::endl;

	return 0;
}