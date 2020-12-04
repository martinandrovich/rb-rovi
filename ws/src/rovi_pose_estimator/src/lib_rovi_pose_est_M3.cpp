#include <iostream>
#include <vector>
#include <tuple>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int32.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/imgcodecs.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/flann.hpp>
#include <opencv4/opencv2/calib3d.hpp>

#include <rovi_pose_estimator/rovi_pose_est.h>
#include <rovi_utils/rovi_utils.h>

namespace rovi_pose_estimator
{

std::array<cv::Mat, 2> 
M3::get_image_data(const std::string & ns_ros)
{
    static std::mutex mutex_image_l;
    static sensor_msgs::Image image_l;
    static auto thread_left = rovi_utils::create_async_listener(ns_ros + "/left/image_raw", image_l, mutex_image_l);

    static std::mutex mutex_image_r;
    static sensor_msgs::Image image_r;
    static auto thread_right = rovi_utils::create_async_listener(ns_ros + "/right/image_raw", image_r, mutex_image_r);

    static bool first = true;

    std::array<cv::Mat, 2> arr;
    cv::Mat temp_l, temp_r;

    std::lock_guard<std::mutex> lock_l(mutex_image_l);
    std::lock_guard<std::mutex> lock_r(mutex_image_r);

    auto ptr_image_l = boost::make_shared<const sensor_msgs::Image>(image_l);
    auto ptr_image_r = boost::make_shared<const sensor_msgs::Image>(image_r);

    cv_bridge::toCvShare(ptr_image_l, "bgr8")->image.copyTo(temp_l);
    arr[0] = temp_l;
    cv_bridge::toCvShare(ptr_image_r, "bgr8")->image.copyTo(temp_r);
    arr[1] = temp_r;

    return arr;
}

std::array<sensor_msgs::CameraInfo, 2>
M3::get_image_info(const std::string & ns_ros)
{
    std::array<sensor_msgs::CameraInfo, 2> arr;

    for (const auto & [idx, cam] : std::array{std::tuple(0, std::string("/left/camera_info")), std::tuple(1, std::string("/right/camera_info"))})
    {
        // get the message
        auto msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(ns_ros + cam);
        arr[idx] = *msg;
    }
    return arr;
}

cv::Mat 
M3::create_mask(const cv::Mat & img, std::vector<cv::Point> & pts)
{
    cv::Mat mask(img.rows, img.cols, CV_8UC1, cv::Scalar(0));
    std::vector<std::vector<cv::Point>> cords;
    cords.push_back(pts);
    cv::drawContours(mask, cords, 0, cv::Scalar(255), -1, 8 );
    return mask;
}

cv::Mat 
M3::tsh_mask(const cv::Mat & img_color, const std::array<double, 3> & lower_rgb, const std::array<double, 3> & upper_rgb)
{
    cv::Mat mask;
    std::vector<cv::Mat> masks;
    cv::split(img_color, masks);
    for(auto i = 0; lower_rgb.size() > i; ++i)
        cv::threshold(masks[i], masks[i], lower_rgb[i], upper_rgb[i], cv::THRESH_BINARY);
    cv::bitwise_and(masks[0], masks[1], mask, masks[2]);
    return mask;
}

}