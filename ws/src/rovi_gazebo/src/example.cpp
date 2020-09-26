#include <ros/ros.h>
#include <rovi_gazebo/rovi_gazebo.h>

int
main(int argc, char** argv)
{
	// define ROS node
	// https://yuzhangbit.github.io/tools/several-ways-of-writing-a-ros-node/

	ros::init(argc, argv, "example_node");
	ros::NodeHandle nh;

	// log information
	ROS_INFO("Initialized a single-thread ROS example node.");

	// use rovi_gazebo library
	rovi_gazebo::test("test");
	
	// give full control over to ROS to handle callbacks etc.
	ros::spin();
}