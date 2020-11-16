#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/topic.h>
#include <image_transport/image_transport.h>
#include <pcl/io/pcd_io.h>
#include <rovi_pose_estimator/rovi_pose_estimator.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <chrono>
#include <thread>


void imageCallback(const sensor_msgs::ImageConstPtr& msg, const std::string& im_window)
{
  try
  {
    cv::imshow(im_window, cv_bridge::toCvShare(msg, "bgr8")->image );
  }
  
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}


void pose_estimation_example()
{

	typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ColorHandlerT;

	pcl::PointCloud<pcl::PointXYZ>::Ptr scene (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr obj (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed (new pcl::PointCloud<pcl::PointXYZ>);

	rovi_pose_estimator::M2::get_point_cloud(scene, true);
	ROS_INFO("Scene loaded..");
	rovi_pose_estimator::M2::load_model("bottle", obj);
	ROS_INFO("Model loaded..");

	
	rovi_pose_estimator::CheckforNans(obj, obj);

	
	pcl::visualization::PCLVisualizer visu("Scene and model");
    visu.addPointCloud (scene, ColorHandlerT (scene, 0.0, 255.0, 0.0), "scene");
    visu.addPointCloud (obj, ColorHandlerT (obj, 0.0, 0.0, 255.0), "object");
	visu.spin();

	rovi_pose_estimator::CheckforNans(scene, scene);
	visu.updatePointCloud(scene, "scene");
	visu.spin();

	rovi_pose_estimator::spatialFilter(scene, scene);
	visu.updatePointCloud(scene, "scene");
	visu.spin();

	float leaf_size = 0.015;

	rovi_pose_estimator::voxelGrid(scene, scene, leaf_size);
	visu.updatePointCloud(scene, "scene");
	rovi_pose_estimator::voxelGrid(obj, obj, leaf_size);
	visu.updatePointCloud(obj, "object");
	visu.spin();

	pcl::ModelCoefficients::Ptr plane_coeff (new pcl::ModelCoefficients);
 	pcl::PointIndices::Ptr plane_inliers (new pcl::PointIndices);

	rovi_pose_estimator::plane_segmentation(scene, plane_inliers, plane_coeff);

	// pcl::PointCloud<pcl::PointXYZ>::Ptr segmented_plane (new pcl::PointCloud<pcl::PointXYZ>);
	rovi_pose_estimator::extract_indices(scene, plane_inliers, scene, true);
	// visu.addPointCloud(segmented_plane, ColorHandlerT (obj, 255.0, 0.0, 0.0), "plane");
	visu.updatePointCloud(scene, "scene");
	ROS_INFO("Points after plane removal... %i", scene->height*scene->width);
	visu.spin();




	//rovi_pose_estimator::outlierRemoval(scene, scene);
	//visu.updatePointCloud(scene, "scene");
	//visu.spin();


	//rovi_pose_estimator::smoothing(scene, scene);
	//visu.updatePointCloud(scene, "scene");
	//visu.spin();

	rovi_pose_estimator::M2::global_pose_est(scene, obj, transformed, leaf_size, 5000);
	visu.addPointCloud (transformed, ColorHandlerT (transformed, 0.0, 0.0, 255.0), "object_aligned");
	pcl::PCDWriter writer;

	writer.write<pcl::PointXYZ>("transformed.pcd", *transformed);
	ROS_INFO("Done performing global pose est...");
	
	visu.spin();
}




int
main(int argc, char** argv)
{
	// define ROS node
	// https://yuzhangbit.github.io/tools/several-ways-of-writing-a-ros-node/

	ros::init(argc, argv, "example_node");
	ros::NodeHandle nh;

	ROS_INFO("Initialized a single-thread ROS example node.");

	pose_estimation_example();


	ros::Rate loop_rate(1000);
	



	//while (ros::ok())
	//{
	//	const auto msg = ros::topic::waitForMessage<sensor_msgs::Image>("/rbrovi/camera_stereo/left/image_raw");
	//	// const auto msg = ros::topic::waitForMessage<sensor_msgs::Image>("/rbrovi/camera_stereo/left/image_raw", ros::Duration(1/30.0));
//
	//	if (msg)
	//	{
	//		ROS_INFO("Got new image!");
	//		const auto img = cv_bridge::toCvShare(msg, "bgr8")->image;
	//		cv::imshow("stereo_right", img);
	//		cv::waitKey(1);
	//	}
	//	loop_rate.sleep();
//
	//}
	
	//nh.subscribe("/rbrovi/camera_stereo/left/image_raw");
	
	// give full control over to ROS to handle callbacks etc.
	//ros::spin();
}

