#include <ros/ros.h>
#include <rovi_pose_estimator/rovi_pose_est.h>
#include <Eigen/Eigen>
#include <cv_bridge/cv_bridge.h>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/stereo/matching.hpp>
#include <rovi_gazebo/rovi_gazebo.h>
#include <eigen_conversions/eigen_msg.h>
#include <iostream>

#define DEBUG 1

cv::RNG rng(12345);

int main(int argc, char** argv)
{
    using namespace rovi_pose_estimator;

    if(argc != 2)
    {
        ROS_INFO_STREAM("Write the quality param for corner detection: <qualityparam>");
        return -1;
    }

    // conversion
    double qual = atof(argv[1]);

	ros::init(argc, argv, "pose_M1");
	ros::NodeHandle nh;
	ros::AsyncSpinner spin(2);
	spin.start();  
    ros::Duration(1).sleep();

    // intrinsic/extrinstinc paramters
    auto cam_info_arr = M3::get_image_info();
    auto cam_images   = M3::get_image_data();

    // ROS_INFO_STREAM("Do you want to compute new ROI : y/n");
    // char answer;
    // std::cin >> answer;

    // if(answer == 'y')
    // {
    //     cv::imshow("win_left", cam_images[0]);
    //     cv::imshow("win_right",cam_images[1]);
    //     cv::waitKey(0);
    //     return 0;
    // }

    // define the ROI's
    std::vector<cv::Point> ROI_left  { cv::Point{191, 394}, cv::Point{609, 394}, cv::Point{658, 530}, cv::Point{143, 530} };
    std::vector<cv::Point> ROI_right { cv::Point{149, 394}, cv::Point{568, 394}, cv::Point{606, 530}, cv::Point{92,  530} };

    // define ROI masks
    cv::Mat ROI_mask_left  = M3::create_mask(cam_images[0], ROI_left);
    cv::Mat ROI_mask_right = M3::create_mask(cam_images[1], ROI_right);

    // define threshhold masks
    cv::Mat tsh_mask_left  = M3::tsh_mask(cam_images[0], {220, 220, 220}, {255, 255, 255});
    cv::Mat tsh_mask_right = M3::tsh_mask(cam_images[1], {220, 220, 220}, {255, 255, 255});

    // write masks
    // #if DEBUG == 1
    //     cv::imwrite("roi_left_mask.jpg",    ROI_mask_left);
    //     cv::imwrite("roi_right_mask.jpg",   ROI_mask_right);
    //     cv::imwrite("tsh_mask_left.jpg",    tsh_mask_left);
    //     cv::imwrite("tsh_mask_right.jpg",   tsh_mask_right);
    // #endif

    // and the masks
    cv::Mat left_mask, right_mask;
    cv::bitwise_and(ROI_mask_left, tsh_mask_left, left_mask);
    cv::bitwise_and(ROI_mask_right, tsh_mask_right, right_mask);

    // convert color to grayscale
    cv::cvtColor(cam_images[0], cam_images[0], cv::COLOR_BGR2GRAY);
    cv::cvtColor(cam_images[1], cam_images[1], cv::COLOR_BGR2GRAY);

    // do some gaussian filtering 
    cv::GaussianBlur(cam_images[0], cam_images[0], cv::Size(3, 3), 0.1);
    cv::GaussianBlur(cam_images[1], cam_images[1], cv::Size(3, 3), 0.1);

    // // track the contours
    // auto get_contour_tracker = [](const cv::Mat & img, const cv::Mat & mask)
    // {   
    //     cv::Mat img_cpy;
    //     std::vector<std::vector<cv::Point>> contour_pts;
    //     cv::bitwise_and(img, mask, img_cpy);
    //     cv::findContours(img_cpy, contour_pts, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
    //     cv::Mat contour_mat = cv::Mat::zeros(img_cpy.size(), img_cpy.type());
    //     if (contour_pts.size() == 0)
    //     {   
    //         ROS_INFO_STREAM("No contour pts are found.");
    //         return contour_mat;
    //     }
    //     cv::drawContours(contour_mat, contour_pts, 0, cv::Scalar(128), 1, 8);
    //     cv::imwrite("contours.jpg", contour_mat);
    //     return contour_mat;
    // };

    // get_contour_tracker(cam_images[0], left_mask);

    // get some nice corners
    auto good_feature_tracker = [&qual](cv::Mat & img, const cv::Mat & mask, bool draw, double min_dist = 30)
    {
        std::vector<cv::Point2f> corners;
        cv::goodFeaturesToTrack( mask, corners, 0, qual, min_dist);
        if(draw)
            for (const auto & corner : corners)
                cv::circle(img, corner, 10, cv::Scalar(128), 2, 8);
        return corners;
    };

    // good feature tracker
    auto corner_left  = good_feature_tracker(cam_images[0], left_mask, 1);
    auto corner_right = good_feature_tracker(cam_images[1], right_mask, 1);
    
    // extract the region of interest, by just assuming rectangle
    cv::Mat img_left = cv::Mat::zeros(cam_images[0].size(), cam_images[0].type());
    cam_images[0].copyTo(img_left, left_mask);

    cv::Mat img_right = cv::Mat::zeros(cam_images[1].size(), cam_images[1].type());
    cam_images[1].copyTo(img_right, right_mask);

    ROS_INFO_STREAM("The amount of corners left found: " << corner_left.size());
    ROS_INFO_STREAM("The amount of corners right found: " << corner_right.size());

    #if DEBUG == 1
        cv::imwrite("left.jpg",  img_left);
        cv::imwrite("right.jpg", img_right);
    #endif

    ROS_INFO_STREAM("The images has been written");

    for(const auto & cam : {corner_left, corner_right})
    {
        for(const auto & corner : cam )
        {
            ROS_INFO_STREAM(corner);
        }
    }

    return 0;
}