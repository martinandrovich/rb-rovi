#include <interpolator/ur5_interpolator.h>
#include <ros/ros.h>

int
main(int argc, char** argv)
{
	// define ROS node
	// https://yuzhangbit.github.io/tools/several-ways-of-writing-a-ros-node/

	ros::init(argc, argv, "example_node");

	ros::NodeHandle nh;

    std::cout << "Hello world" << std::endl;

    ur5_interp::rounded_composite();


    return 0;
}