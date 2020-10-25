#include "rovi_pose_estimator/rovi_pose_estimator.h"
#include <iostream>
#include <ros/ros.h>

void
rovi_pose_estimator::test(const std::string& str)
{
	ROS_INFO_STREAM("hello: " << str);
}