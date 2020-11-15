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



typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ColorHandlerT;

int
main(int argc, char** argv)
{
	// define ROS node
	// https://yuzhangbit.github.io/tools/several-ways-of-writing-a-ros-node/

	ros::init(argc, argv, "example_node");
	ros::NodeHandle nh;
	// cv::namedWindow("img");
	// cv::waitKey(0);

	// log information
	ROS_INFO("Initialized a single-thread ROS example node.");
	//cv::namedWindow("stereo_left");
	//cv::namedWindow("stereo_right");
	//cv::startWindowThread();

	// image_transport::ImageTransport it(nh);
	// //image_transport::Subscriber sub = it.subscribe("rbrovi/camera_stereo/left/image_raw", 1, &imageCallback, image_transport::TransportHints("left_image"));
	// image_transport::Subscriber sub2 = it.subscribe("rbrovi/camera_stereo/right/image_raw", 1, &imageCallback);
	// ros::spin();
	// cv::destroyAllWindows();

	// const auto& sub = nh.subscribe("rbrovi/camera_stereo/left/image_raw", 1, &imageCallback);
	// use rovi_gazebo library
	// rovi_pose_estimator::test("test");
	// cv::Mat img;

	ros::Rate loop_rate(1000);
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

	rovi_pose_estimator::voxelGrid(scene, scene, 0.01q);
	visu.updatePointCloud(scene, "scene");
	visu.spin();

	//rovi_pose_estimator::outlierRemoval(scene, scene);
	//visu.updatePointCloud(scene, "scene");
	//visu.spin();


	//rovi_pose_estimator::smoothing(scene, scene);
	//visu.updatePointCloud(scene, "scene");
	//visu.spin();

	rovi_pose_estimator::M2::global_pose_est(scene, obj, transformed);
	visu.addPointCloud (transformed, ColorHandlerT (transformed, 0.0, 0.0, 255.0), "object_aligned");
	pcl::PCDWriter writer;

	writer.write<pcl::PointXYZ>("transformed.pcd", *transformed);
	ROS_INFO("Done performing global pose est...");
	
	visu.spin();





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

