#include <ros/ros.h>
#include <rovi_pose_estimator/rovi_pose_est_M1.h>
#include <Eigen/Eigen>
#include <cv_bridge/cv_bridge.h>
#include <rovi_gazebo/rovi_gazebo.h>
#include <eigen_conversions/eigen_msg.h>

// all matrices, that need to be known.
// Eigen::Matrix<double, 4, 4, Eigen::RowMajor> P_left(cam_info_arr[0].P.data());
// Eigen::Matrix<double, 4, 4, Eigen::RowMajor> P_right(cam_info_arr[1].P.data());
// Eigen::Matrix<double, 3, 3, Eigen::RowMajor> K_right(cam_info_arr[1].K.data());
// Eigen::Matrix<double, 3, 3, Eigen::RowMajor> R_left(cam_info_arr[0].R.data());
// Eigen::Matrix<double, 3, 3, Eigen::RowMajor> R_right(cam_info_arr[1].R.data());

// all matrices 
// std::cout << P_left << std::endl;
// std::cout << P_right << std::endl;
// std::cout << R_left << std::endl;
// std::cout << R_right << std::endl;

int main(int argc, char** argv)
{
    using namespace rovi_pose_estimator;

	ros::init(argc, argv, "pose_M1");
	ros::NodeHandle nh;
	ros::AsyncSpinner spin(4);
	spin.start();  
    ros::Duration(1).sleep();

    // turn projector on/off
    M1::set_structed_light(nh, false);

    // intrinsic/extrinstinc paramters
    auto cam_info_arr = M1::get_image_info();

    // compute the intrinsic parameters to compute Q
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> K_left(cam_info_arr[0].K.data());
    ROS_INFO_STREAM("K_left \n" << K_left);

    // statically define the Q matrix, change this later.
    constexpr auto BASELINE = 0.08;
    cv::Mat Q = (cv::Mat_<double>(4,4) <<   1., 0.,  0.,            -K_left(0,2),
                                            0., 1.,  0.,            -K_left(1,2),
                                            0., 0.,  0.,             K_left(1,1),
                                            0., 0., -1/BASELINE,               0.
                );
    ROS_INFO_STREAM("Q: \n" << Q);

    // get stereo images
    auto cam_images = M1::get_image_data();
    auto ROI = M1::get_ROI(cam_images[0], cam_images[1], Q);

    ROS_INFO_STREAM("ROI: \n" << ROI);

    // set the structured light
    M1::set_structed_light(nh, true);

    // get the images once again to compute disparity pts
    cam_images = M1::get_image_data();
    
    // write the stereo images
    // cv::imwrite("left.jpg", cam_images[0]);
    // cv::imwrite("right.jpg", cam_images[1]);

    // compute the disparity map
    cv::Mat point_cloud = M1::compute_disparitymap(cam_images[0], cam_images[1], Q);

    cv::imwrite("point_cloud.jpg", point_cloud);

    // compute P_left
    Eigen::Affine3d affine;
    tf::poseMsgToEigen(rovi_gazebo::get_model_pose("camera_stereo"), affine);    
    Eigen::Matrix4f w_T_b = affine.matrix().cast<float>();

    // compute the point cloud
    M1::compute_pointcloud(point_cloud, cam_images[0], ROI, w_T_b);

    return 0;
}