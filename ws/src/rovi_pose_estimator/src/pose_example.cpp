#include <ros/ros.h>
#include <rovi_pose_estimator/rovi_pose_estimator.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/topic.h>
#include <image_transport/image_transport.h>

#define test 1

#if !test
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

	cv::namedWindow("stereo_right");
	cv::startWindowThread();
	ros::Rate loop_rate(1000);
	while (ros::ok())
	{
		const auto msg = ros::topic::waitForMessage<sensor_msgs::Image>("/rbrovi/camera_stereo/left/image_raw");
		// const auto msg = ros::topic::waitForMessage<sensor_msgs::Image>("/rbrovi/camera_stereo/left/image_raw", ros::Duration(1/30.0));

		if (msg)
		{
			ROS_INFO("Got new image!");
			const auto img = cv_bridge::toCvShare(msg, "bgr8")->image;
			cv::imshow("stereo_right", img);
			cv::waitKey(1);
		}
		loop_rate.sleep();

	}
	
	//nh.subscribe("/rbrovi/camera_stereo/left/image_raw");
	
	// give full control over to ROS to handle callbacks etc.
	//ros::spin();
}

#endif

#if test

#include "std_msgs/String.h"

#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}


#endif

