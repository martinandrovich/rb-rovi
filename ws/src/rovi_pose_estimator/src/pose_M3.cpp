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
    auto cam_images_color = M3::get_image_data();
    auto cam_images_gray = cam_images_color;

    // define the ROI's
    std::vector<cv::Point> ROI_left  { cv::Point{0, 0}, cv::Point{0, 799}, cv::Point{799, 799}, cv::Point{799, 0} };
    std::vector<cv::Point> ROI_right { cv::Point{0, 0}, cv::Point{0, 799}, cv::Point{799, 799}, cv::Point{799, 0} };

    // define ROI masks
    cv::Mat ROI_mask_left  = M3::create_mask(cam_images_gray[0], ROI_left);
    cv::Mat ROI_mask_right = M3::create_mask(cam_images_gray[1], ROI_right);

    // define threshhold masks
    cv::Mat tsh_mask_left  = M3::tsh_mask(cam_images_gray[0], {220, 220, 220}, {255, 255, 255});
    cv::Mat tsh_mask_right = M3::tsh_mask(cam_images_gray[1], {220, 220, 220}, {255, 255, 255});

    // and the masks
    cv::Mat left_mask, right_mask;
    cv::bitwise_and(ROI_mask_left, tsh_mask_left, left_mask);
    cv::bitwise_and(ROI_mask_right, tsh_mask_right, right_mask);

    // convert color to grayscale
    cv::cvtColor(cam_images_gray[0], cam_images_gray[0], cv::COLOR_BGR2GRAY);
    cv::cvtColor(cam_images_gray[1], cam_images_gray[1], cv::COLOR_BGR2GRAY);

    // do some gaussian filtering 
    cv::GaussianBlur(cam_images_gray[0], cam_images_gray[0], cv::Size(3, 3), 0.1);
    cv::GaussianBlur(cam_images_gray[1], cam_images_gray[1], cv::Size(3, 3), 0.1);

    // dilate the images, find the contour
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3,3), cv::Point(1, 1));
    cv::morphologyEx(left_mask, left_mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(right_mask, right_mask, cv::MORPH_OPEN, kernel);

    // perform dilation
    cv::dilate(left_mask, left_mask, kernel);
    cv::dilate(right_mask, right_mask, kernel);

    // perform canny
    cv::Canny(left_mask,  left_mask,  0, 100, 3, false);
    cv::Canny(right_mask, right_mask, 0, 100, 3, false);

    // write masks
    #if DEBUG == 1
        cv::imwrite("left_mask.jpg", left_mask);
        cv::imwrite("right_mask.jpg", right_mask);
    #endif

    // track the contours
    auto get_contour_tracker = [](const cv::Mat & mask)
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

    // contour_mat_left / right
    cv::Mat contour_mat_left  = get_contour_tracker(left_mask);
    cv::Mat contour_mat_right = get_contour_tracker(right_mask);

    // write masks
    #if DEBUG == 1
        cv::imwrite("final_l_mask.jpg", contour_mat_left);
        cv::imwrite("final_r_mask.jpg", contour_mat_right);
    #endif

    // get some nice corners
    auto good_feature_tracker = [&qual](cv::Mat & img, std::vector<cv::Point2f> & corners, const cv::Mat & mask, bool draw, double min_dist = 50)
    {
        cv::goodFeaturesToTrack( mask, corners, 4, qual, min_dist, cv::Mat(), 5, false);
        if(draw)
            for (const auto & corner : corners)
                cv::circle(img, corner, 10, cv::Scalar(128), 2, 8);
        return corners;
    };

    // good feature tracker
    std::vector<cv::Point2f> corner_pts_left;
    std::vector<cv::Point2f> corner_pts_right;
    auto corner_left  = good_feature_tracker(cam_images_gray[0], corner_pts_left, contour_mat_left,   1);
    auto corner_right = good_feature_tracker(cam_images_gray[1], corner_pts_right, contour_mat_right, 1);

    #if DEBUG == 1
        cv::imwrite("left.jpg",  cam_images_gray[0]);
        cv::imwrite("right.jpg", cam_images_gray[1]);
    #endif

    // euclidean lambda
    auto eu_dist = [](const cv::Point2f & left, const cv::Point2f & right, const double y_tsh = 10)
    {
        if (left.x < right.x || abs(left.y - right.y) > y_tsh)
            return std::numeric_limits<double>::max();
        else
            return std::pow((left.x - right.x), 2) + std::pow((left.y - right.y), 2);
    };

    // brute-force with bias
    std::vector<std::array<cv::Point2f, 2>> pts;
    for ( const auto & left_pt : corner_pts_left )
    {
        int k = 0;
        double dist = std::numeric_limits<double>::max();
        for ( int i = 0; i < corner_pts_right.size(); ++i )
        {
            if (auto eu_cmp = eu_dist(left_pt, corner_pts_right[i]); eu_cmp < dist)
            {
                dist = eu_cmp;
                k = i;
            }
        }
        pts.push_back(std::array<cv::Point2f, 2>{left_pt, corner_pts_right[k]});
    }

    // [ RIGHT | LEFT ]
    // stitch the images to see if the results are correct.
    const double offset_x = cam_images_color[1].cols-1;
    cv::Mat img_stitched = cv::Mat::zeros(cv::Size(cam_images_gray[0].cols * 2, cam_images_gray[0].rows), CV_8UC3);
    cam_images_color[0].copyTo(img_stitched(cv::Rect(0, 0, cam_images_color[0].cols, cam_images_color[0].rows)));
    cam_images_color[1].copyTo(img_stitched(cv::Rect(offset_x, 0, cam_images_color[1].cols, cam_images_color[1].rows)));

    cv::putText(img_stitched, "Left Image", cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2, cv::LINE_AA);
    cv::putText(img_stitched, "Right Image", cv::Point(50 + offset_x, 50), cv::FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2, cv::LINE_AA);

    for(auto pt : pts)
    {
        pt[1].x += offset_x;
        cv::line(img_stitched, pt[0], pt[1], cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)));
    }

    // Not a part of the debugging, this is for the report
    cv::imwrite("img_stitched.jpg", img_stitched );

    // define the Q matrix
    constexpr auto BASELINE = -0.08;
    
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> K_left(cam_info_arr[0].K.data());
    
    cv::Mat Q = ( cv::Mat_<double>(4,4) << 1., 0.,  0.,         -K_left(0,2),
                                           0., 1.,  0.,         -K_left(1,2),
                                           0., 0.,  0.,          K_left(1,1),
                                           0., 0., -1/BASELINE,          0.0 );
    
    ROS_INFO_STREAM("\nQ" << Q);

    cv::Mat m(cv::Size(4, pts.size()), CV_64FC1);

    double k_means = 0;
    for(auto i = 0; i < pts.size(); i++)
        k_means += (double)(pts[i][0].x - pts[i][1].x);
    k_means /= pts.size();

    for(auto i = 0; i < pts.size(); i++)
    {
        m.at<double>(0, i) = (double)(pts[i][0].x);
        m.at<double>(1, i) = (double)(pts[i][0].y);
        m.at<double>(2, i) = (double)(pts[i][0].x - pts[i][1].x);
        m.at<double>(3, i) = 1.f;
        // ROS_INFO_STREAM( m.at<double>(0, i)  << ", " << m.at<double>(1, i) << ", " << m.at<double>(2, i));
    }

    // Get the pose of the camera.
    cv::Mat w_T_c;
    Eigen::Affine3d affine;
    tf::poseMsgToEigen(rovi_gazebo::get_model_pose("camera_stereo"), affine);
    cv::eigen2cv(affine.matrix(), w_T_c);

    cv::Mat T = ( cv::Mat_<double>(4,4) <<  0., 0.,  1., 0.,
                                           -1., 0.,  0., 0.,
                                            0., -1,  0., 0.,
                                            0., 0.,  0., 1. );

    cv::Mat M = w_T_c * T * Q * m;
    
    // Determine M
    for (auto i = 0; i < pts.size(); i++)
    {
        const double w = M.at<double>(3, i);
        M.at<double>(0, i) = M.at<double>(0, i) / w;
        M.at<double>(1, i) = M.at<double>(1, i) / w;
        M.at<double>(2, i) = M.at<double>(2, i) / w;
        // ROS_INFO_STREAM( M.at<double>(0, i)  << ", "<< M.at<double>(1, i) << ", " << M.at<double>(2, i));
    }

    // Take an arbitrary point
    cv::Point2d origo(M.at<double>(0, 0));
    int longest_idx = 0;

    auto longest = [&M, &pts, &origo, &longest_idx]()
    {    
        double dist = std::numeric_limits<double>::min();
        for (auto i = 1; i < pts.size(); i++)
        {  
            if (auto distance = cv::norm( origo - cv::Point2d(M.at<double>(0, i), M.at<double>(1, i) ) ); dist < distance)
            {
                dist = distance;
                longest_idx = i;
            }
        }
    };
    
    longest();

    std::vector<int> indices;
    for (auto i = 1; i < 3; i++)
    {
        if (i != longest_idx)
        {
            
        }
        
    }
    
    
    return 0;
}