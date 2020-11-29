#pragma once
#include <string>
#include <array>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/CameraInfo.h>
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

        // class SparseStereo
        // {
        //     public:

        //         SparseStereo() = delete;
        //         ~SparseStereo();
        //         SparseStereo(const std::vector<double> cam_left, const std::vector<double> cam_right);

        //     private:

        // };

    }
}
