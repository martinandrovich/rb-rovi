#include <ros/ros.h>
#include <rovi_pose_estimator/rovi_pose_est.h>
#include <Eigen/Eigen>
#include <cv_bridge/cv_bridge.h>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/stereo/matching.hpp>
#include <opencv4/opencv2/ml.hpp>
#include <opencv4/opencv2/core/eigen.hpp>
#include <rovi_gazebo/rovi_gazebo.h>
#include <eigen_conversions/eigen_msg.h>
#include <iostream>

#define DEBUG 1

int main(int argc, char** argv)
{
    using namespace rovi_pose_estimator;

    // init the node
	ros::init(argc, argv, "pose_M1");
	ros::NodeHandle nh;
	ros::AsyncSpinner spin(2);
	spin.start();  

    geometry_msgs::Pose pose = M3::pipeline(1);
    
    return 0;
}