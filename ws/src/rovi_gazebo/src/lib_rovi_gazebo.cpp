#include "rovi_gazebo/rovi_gazebo.h"

#include <iostream>
#include <ros/ros.h>

void
rovi_gazebo::test(const std::string& str)
{
	ROS_INFO_STREAM("hello: " << str);
}