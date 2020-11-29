#include <ros/ros.h>
#include <rovi_pose_estimator/rovi_pose_est_M1.h>
#include <Eigen/Eigen>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pose_M1");
	ros::NodeHandle nh;
	ros::AsyncSpinner spin(2);
	spin.start();  

    //
    cv::Mat Q = (cv::Mat_<double>(4,4) <<   1., 0.,  0.,   -320.000,
                                            0., 1.,  0.,   -240.000,
                                            0., 0.,  0.,    514.682,
                                            0., 0., -1/0.2, 000.000
                );


    // all matrices, that need to be known.
    // Eigen::Matrix<double, 4, 4, Eigen::RowMajor> P_left(cam_info_arr[0].P.data());
    // Eigen::Matrix<double, 4, 4, Eigen::RowMajor> P_right(cam_info_arr[1].P.data());
    // Eigen::Matrix<double, 3, 3, Eigen::RowMajor> K_left(cam_info_arr[0].K.data());
    // Eigen::Matrix<double, 3, 3, Eigen::RowMajor> K_right(cam_info_arr[1].K.data());
    // Eigen::Matrix<double, 3, 3, Eigen::RowMajor> R_left(cam_info_arr[0].R.data());
    // Eigen::Matrix<double, 3, 3, Eigen::RowMajor> R_right(cam_info_arr[1].R.data());

    // all matrices 
    // std::cout << P_left << std::endl;
    // std::cout << P_right << std::endl;
    // std::cout << R_left << std::endl;
    // std::cout << R_right << std::endl;

    // turn projector on/off
    rovi_pose_estimator::M1::set_structed_light(nh, false);

    // intrinsic/extrinstinc paramters
    auto cam_info_arr = rovi_pose_estimator::M1::get_image_info();

    // get stereo images
    auto cam_images = rovi_pose_estimator::M1::get_image_data();
    rovi_pose_estimator::M1::get_ROI(cam_images[0], cam_images[1], Q);

    // set the structured light
    rovi_pose_estimator::M1::set_structed_light(nh, true);

    // get the images once again to compute disparity pts
    cam_images = rovi_pose_estimator::M1::get_image_data();

    // 
    

    return 0;
}