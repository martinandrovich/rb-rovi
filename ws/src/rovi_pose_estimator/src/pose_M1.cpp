#include <ros/ros.h>
#include <ros/package.h>
#include <rovi_pose_estimator/rovi_pose_est.h>
#include <Eigen/Eigen>
#include <cv_bridge/cv_bridge.h>
#include <rovi_gazebo/rovi_gazebo.h>
#include <eigen_conversions/eigen_msg.h>

int main(int argc, char** argv)
{
    using namespace rovi_pose_estimator;
    
    if ( argc != 2)
    {
        ROS_INFO_STREAM("How to use, insert number of it : <it>");
        return -1;
    }

    int iterations = std::stoi(argv[1]);

    // init the node
	ros::init(argc, argv, "pose_M1");
	ros::NodeHandle nh;
	ros::AsyncSpinner spin(4);
	spin.start();  

    auto pose = M1::estimate_pose(iterations);

    ROS_INFO_STREAM(pose);

    return 0;
}