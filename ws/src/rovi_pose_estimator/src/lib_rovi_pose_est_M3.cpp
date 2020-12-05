#include <iostream>
#include <vector>
#include <tuple>

#include <Eigen/Eigen>

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
#include <opencv4/opencv2/core/eigen.hpp>

#include <rovi_pose_estimator/rovi_pose_est.h>
#include <rovi_utils/rovi_utils.h>

#include <rovi_gazebo/rovi_gazebo.h>
#include <eigen_conversions/eigen_msg.h>

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
M3::create_mask(const cv::Mat & img, const std::vector<cv::Point> & pts)
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

cv::Mat
M3::find_contour(const cv::Mat & mask)
{   
    cv::Mat img_cpy;
    std::vector<std::vector<cv::Point>> contour_pts;
    cv::findContours(mask, contour_pts, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    cv::Mat contour_mat = cv::Mat::zeros(mask.size(), mask.type());
    if (contour_pts.size() == 0)
        return contour_mat;
    for (auto i = 0; contour_pts.size() > i; i++)
        cv::drawContours(contour_mat, contour_pts, i, cv::Scalar(255), cv::FILLED);
    return contour_mat;
};

double
M3::L2_stereo(const cv::Point2d & left, const cv::Point2d & right, const double y_tsh)
{
    if (left.x < right.x || abs(left.y - right.y) > y_tsh)
        return std::numeric_limits<double>::max();
    else
        return std::pow((left.x - right.x), 2) + std::pow((left.y - right.y), 2);
}

cv::Mat
M3::stereo_triangulation(const std::vector<std::array<cv::Point2d, 2>> & pts, const cv::Mat & Q, const bool & kmeans)
{
    cv::Mat T = ( cv::Mat_<double>(4,4) <<  0.f, 0.f,  1.f, 0.f,
                                            -1.f, 0.f,  0.f, 0.f,
                                            0.f, -1.f, 0.f, 0.f,
                                            0.f,  0.f, 0.f, 1.f );

    // Create disparity points (x, y, d(x,y), 1)
    cv::Mat m( cv::Size( 4, pts.size() ), CV_64FC1);

    // Get the pose of the camera.
    cv::Mat w_T_c;
    Eigen::Affine3d affine;
    tf::poseMsgToEigen(rovi_gazebo::get_model_pose("camera_stereo"), affine);
    cv::eigen2cv(affine.matrix(), w_T_c);

    // Calculate the disparity mean, this assumes 4 corner max, not a lot of corners!
    // if (kmeans == true)
    // {
    //     double k_means = 0;
    //     for(auto i = 0; i < pts.size(); i++)
    //         k_means += (double)(pts[i][0].x - pts[i][1].x);
    //     k_means /= pts.size();
    // }

    // Calculate disparity points
    for(auto i = 0; i < pts.size(); i++)
    {
        m.at<double>(0, i) = pts[i][0].x;
        m.at<double>(1, i) = pts[i][0].y;
        m.at<double>(2, i) = pts[i][0].x - pts[i][1].x;
        m.at<double>(3, i) = 1.f;
    }

    // Constant transformation matrix
    cv::Mat M = w_T_c * T * Q * m;

    // Determine M
    for (auto i = 0; i < pts.size(); i++)
    {
        const double w = M.at<double>(3, i);
        M.at<double>(0, i) = M.at<double>(0, i) / w;
        M.at<double>(1, i) = M.at<double>(1, i) / w;
        M.at<double>(2, i) = M.at<double>(2, i) / w;
    }

    return M;
}

}