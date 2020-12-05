#pragma once
#include <string>
#include <array>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <Eigen/Eigen>

// #include <pcl/point_types.h>
// #include <pcl/point_cloud.h>
// #include <pcl/registration/transformation_estimation_svd.h>

namespace rovi_pose_estimator
{
    namespace M1
    {

        std::array<cv::Mat, 2> 
        get_image_data(const std::string & ns_ros = "/rbrovi/camera_stereo");

        std::array<sensor_msgs::CameraInfo, 2>
        get_image_info(const std::string & ns_ros = "/rbrovi/camera_stereo");

        void 
        set_structed_light(ros::NodeHandle & nh, const bool & state);

        cv::Mat
        get_ROI(const cv::Mat & img_left, const cv::Mat & img_right, const cv::Mat & Q);

        cv::Mat 
        compute_disparitymap(const cv::Mat & img_left, const cv::Mat & img_right, const cv::Mat & Q);

        void
        compute_pointcloud(const cv::Mat & point_cloud, const cv::Mat & left_img, const cv::Mat & ROI, const Eigen::Matrix4f & trans);

    }

    namespace M3
    {
        std::array<cv::Mat, 2> 
        get_image_data(const std::string & ns_ros = "/rbrovi/camera_stereo");         

        std::array<sensor_msgs::CameraInfo, 2>
        get_image_info(const std::string & ns_ros = "/rbrovi/camera_stereo");

        cv::Mat create_mask(const cv::Mat & img, const std::vector<cv::Point> & pts);

        cv::Mat tsh_mask(const cv::Mat & img_color, 
                         const std::array<double, 3> & lower_rgb = {230.f, 230.f, 230.f}, 
                         const std::array<double, 3> & upper_rgb = {255.f, 255.f, 255.f}
                         );

        cv::Mat find_contour(const cv::Mat & mask);

        double L2_stereo(const cv::Point2d & left, const cv::Point2d & right, const double y_tsh = 10);

        cv::Mat stereo_triangulation(const std::vector<std::array<cv::Point2d, 2>> & pts, const cv::Mat & Q, const bool & k_means = false);
    }
}
