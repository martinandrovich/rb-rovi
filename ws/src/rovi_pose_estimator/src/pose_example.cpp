#include <ros/ros.h>
#include <rovi_pose_estimator/rovi_pose_estimator.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/topic.h>
#include <image_transport/image_transport.h>

void imageCallback(const sensor_msgs::ImageConstPtr& msg, const std::string& im_window)
{
  try
  {
    cv::imshow(im_window, cv_bridge::toCvShare(msg, "bgr8")->image );
	cv::waitKey(1);
  }
  
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int
main(int argc, char** argv)
{
	// define ROS node
	// https://yuzhangbit.github.io/tools/several-ways-of-writing-a-ros-node/

	ros::init(argc, argv, "example_node");
	ros::NodeHandle nh;
	

	// log information
	ROS_INFO("Initialized a single-thread ROS example node.");
	cv::namedWindow("stereo_left");
	cv::namedWindow("stereo_right");
	cv::startWindowThread();

	image_transport::ImageTransport it(nh);
	//image_transport::Subscriber sub = it.subscribe("rbrovi/camera_stereo/left/image_raw", 1, &imageCallback, image_transport::TransportHints("left_image"));
	image_transport::Subscriber sub2 = it.subscribe("rbrovi/camera_stereo/right/image_raw", 1, &imageCallback);
	ros::spin();
	cv::destroyAllWindows();

	// const auto& sub = nh.subscribe("rbrovi/camera_stereo/left/image_raw", 1, &imageCallback);
	// use rovi_gazebo library
	// rovi_pose_estimator::test("test");

	//while(ros::ok())
	//{
	//	auto msg = ros::topic::waitForMessage<sensor_msgs::Image>( "rbrovi/camera_stereo/left/image_raw", nh);
	//	//imageCallback(msg);
	//	cv::imshow("stereo_left2", cv_bridge::toCvShare(msg, "bgr8")->image );
	//	cv::waitKey(0);
//
	//}
	

	//nh.subscribe("/rbrovi/camera_stereo/left/image_raw");
	
	// give full control over to ROS to handle callbacks etc.
	//ros::spin();
}