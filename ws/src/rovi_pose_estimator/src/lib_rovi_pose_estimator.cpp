#include "rovi_pose_estimator/rovi_pose_estimator.h"
#include <iostream>
#include <ros/ros.h>

void
rovi_pose_estimator::test(const std::string& str)
{
	ROS_INFO_STREAM("hello: " << str);
}


namespace rovi_pose_estimator

{
	namespace M2
	{
		void
        get_point_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud)
		{
			//const auto msg = ros::topic::waitForMessage<sensor_msgs::Image>("/rbrovi/camera_stereo/left/image_raw");

		}
		
		void
		get_depth_image()
		{
			
		}
	}

	namespace TEST
	{

	}
	
}