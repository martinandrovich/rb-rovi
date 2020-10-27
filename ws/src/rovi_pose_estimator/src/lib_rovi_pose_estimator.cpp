#include "rovi_pose_estimator/rovi_pose_estimator.h"
#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "pcl_ros/point_cloud.h"

#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/passthrough.h>
#include <pcl/conversions.h>



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
			const auto point_cloud = ros::topic::waitForMessage<pcl::PointCloud<pcl::PointXYZRGB>>("/rbrovi/kinect_sensor_ir/rbrovi/kinect_sensor/depth/points");

			pcl::PCDWriter writer;
  			writer.write<pcl::PointXYZRGB> ("kinect_cloud.pcd", *point_cloud, false);
			*output_cloud = *point_cloud;
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