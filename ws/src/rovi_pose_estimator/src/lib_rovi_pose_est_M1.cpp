#include <iostream>
#include <vector>
#include <tuple>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int32.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/imgcodecs.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/flann.hpp>

#include <rovi_pose_estimator/rovi_pose_est_M1.h>

namespace rovi_pose_estimator
{

std::array<cv::Mat, 2> 
M1::get_image_data(const std::string & ns_ros)
{
    std::array<cv::Mat, 2> arr;

    for (const auto & [idx, cam] : std::array{std::tuple(0, std::string("/left/image_raw")), std::tuple(1, std::string("/right/image_raw"))})
    {   
        cv::Mat temp;
        auto msg = ros::topic::waitForMessage<sensor_msgs::Image>(ns_ros + cam);
        cv_bridge::toCvShare(msg, "bgr8")->image.copyTo(temp);
        arr[idx] = temp;
    }

    return arr;
}

void
M1::set_structed_light(ros::NodeHandle & nh, const bool & state)
{
    auto handler = nh.advertise<std_msgs::Int32>("/projector_controller/projector", 1);
    ros::Rate lp(50);
    auto tic = ros::Time::now();
    while (0.1 > (ros::Time::now() - tic).toSec())
    {
        std_msgs::Int32 msg;
        msg.data = (int)state;
        handler.publish(msg);
        lp.sleep();
    }
}

std::array<sensor_msgs::CameraInfo, 2>
M1::get_image_info(const std::string & ns_ros)
{
    std::array<sensor_msgs::CameraInfo, 2> arr;

    for (const auto & [idx, cam] : std::array{std::tuple(0, std::string("/left/camera_info")), std::tuple(1, std::string("/right/camera_info"))})
    {
        // clear buffer quickfix
        ros::Rate lp(50);
        auto tic = ros::Time::now();
        while( 0.1 > (ros::Time::now()-tic).toSec())
        {
            ros::topic::waitForMessage<sensor_msgs::CameraInfo>(ns_ros + cam);
            lp.sleep();
        }

        // get the message
        auto msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(ns_ros + cam);
        arr[idx] = *msg;
    }
    return arr;
}

cv::Mat
M1::get_ROI(const cv::Mat & img_left, const cv::Mat & img_right, const cv::Mat & Q)
/*
*   this method finds the RoI 4 points needed from the table.
*/
{
    // time it for benchmarking
    auto tic = ros::Time::now();

    // the current method is not robust to noise
    cv::Mat temp_left, temp_right;

    // threshhold the images, done in matlab
    cv::inRange(img_left,  cv::Scalar(120, 170, 170), cv::Scalar(180, 255, 255), temp_left);
    cv::inRange(img_right, cv::Scalar(120, 170, 170), cv::Scalar(180, 255, 255), temp_right);

    // stage one convert image to grayscale
    cv::Canny(temp_left,  temp_left, 50, 200, 3);
    cv::Canny(temp_right, temp_right, 50, 200, 3);

    cv::imshow("temp_left", temp_left);
    cv::imshow("temp_right", temp_right);
    cv::waitKey(0);

    // contours, hulls, hier
    std::vector<std::vector<cv::Point>> contours;
    std::vector<std::vector<cv::Point>> hull;
    std::vector<cv::Point> roi_left, roi_right;
    std::vector<cv::Vec4i> hier;

    for (auto & [ img, roi ] : std::array{std::tuple(&temp_left, &roi_left), std::tuple(&temp_right, &roi_right)} )
    {
        cv::findContours(*img, contours, hier, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        hull.resize(contours.size());

        for (size_t i = 0; i < contours.size(); i++)
        {
            cv::convexHull(contours[i], hull[i]);
            cv::drawContours(*img, hull, 0, cv::Scalar(255));
        }

        int xMin = std::numeric_limits<int>::max();
        int xMax = std::numeric_limits<int>::min();
        int yMin = std::numeric_limits<int>::max();
        int yMax = std::numeric_limits<int>::min();

        for (const auto &hull_ele : hull)
        {
            for (const auto &pt : hull_ele)
            {
                xMin = xMin > pt.x ? pt.x : xMin;
                xMax = xMax < pt.x ? pt.x : xMax;
                yMin = yMin > pt.y ? pt.y : yMin;
                yMax = yMax < pt.y ? pt.y : yMax;
            }
        }

        roi->push_back(cv::Point(xMin, yMin));
        roi->push_back(cv::Point(xMin, yMax));
        roi->push_back(cv::Point(xMax, yMax));
        roi->push_back(cv::Point(xMax, yMin));
    }

    ROS_INFO_STREAM(roi_left.size());
    ROS_INFO_STREAM(roi_right.size());

    // save the roi_element
    for (const auto & roi_ele : roi_left)
    {
        ROS_INFO_STREAM(roi_ele);
        // cv::drawMarker(temp_left, roi_ele, cv::Scalar(255), 0, 40, 10);
    }

    cv::imshow("roi_left", temp_left);
    cv::waitKey(0);

    // disp
    std::vector<double> disparity(roi_left.size());
    cv::Mat p(cv::Size(roi_left.size(), roi_left.size()), CV_64F);
    for (size_t i = 0; i < roi_left.size(); i++)
    {
        disparity[i] = roi_left[i].x - roi_left[i].x;
        p.at<double>(0, i) = (double)roi_left[i].x;
        p.at<double>(1, i) = (double)roi_left[i].y;
        p.at<double>(2, i) = (double)disparity[i];
        p.at<double>(3, i) = 1.0f;
    }

    cv::Mat roi = Q * p;

    for (size_t i = 0; i < roi_left.size(); i++)
    {
        roi.at<double>(0, i) = roi.at<double>(0, i) / roi.at<double>(3, i);
        roi.at<double>(1, i) = roi.at<double>(1, i) / roi.at<double>(3, i);
        roi.at<double>(2, i) = roi.at<double>(2, i) / roi.at<double>(3, i);
        roi.at<double>(3, i) = 1.f;
    }

    // end benchmark
    auto toc = ros::Time::now();
    auto dur = toc - tic;
    ROS_INFO_STREAM("Preprocessing took: " << dur.toSec());

    return roi;
}

}