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

static constexpr auto LEFT = 0;
static constexpr auto RIGHT = 1;
static constexpr auto IMG_SIZE_W = 800;
static constexpr auto IMG_SIZE_H = 800;
static constexpr auto GAUSS_BLUR_STD    = 0.10;
static const auto GAUSS_BLUR_KERNEL = cv::Size(3, 3);
static const std::array<double, 3> TSH_LOWER = {220, 220, 220};
static const std::array<double, 3> TSH_UPPER = {255, 255, 255};
static const std::vector<cv::Point> ROI_left  { cv::Point{0, 0}, cv::Point{0, IMG_SIZE_H-1}, cv::Point{IMG_SIZE_W-1, IMG_SIZE_H-1}, cv::Point{IMG_SIZE_W-1, 0} };
static const std::vector<cv::Point> ROI_right { cv::Point{0, 0}, cv::Point{0, IMG_SIZE_H-1}, cv::Point{IMG_SIZE_W-1, IMG_SIZE_H-1}, cv::Point{IMG_SIZE_W-1, 0} };
static constexpr double MIN_DIST_GOOD_FEATURE = 50;
static constexpr int MAX_CORNERS_GOOD_FEATURE = 4; 
static constexpr auto BASELINE = -0.08;

int main(int argc, char** argv)
{
    using namespace rovi_pose_estimator;

    if(argc != 2)
    {
        ROS_INFO_STREAM("Write the quality param for corner detection: <qualityparam>");
        return -1;
    }

    // init the node
	ros::init(argc, argv, "pose_M1");
	ros::NodeHandle nh;
	ros::AsyncSpinner spin(2);
	spin.start();  

    // conversion
    double qual = atof(argv[1]);

    // Get intrinsic / extrinsic parameters / get images
    auto cam_info_arr = M3::get_image_info();
    auto cam_images_color = M3::get_image_data();
    auto cam_images_gray = cam_images_color;

    auto tic = ros::Time::now();

    // Define ROI masks
    cv::Mat ROI_mask_left  = M3::create_mask(cam_images_gray[LEFT],  ROI_left);
    cv::Mat ROI_mask_right = M3::create_mask(cam_images_gray[RIGHT], ROI_right);

    // Define threshhold masks
    cv::Mat tsh_mask_left  = M3::tsh_mask(cam_images_gray[LEFT],  TSH_LOWER, TSH_UPPER);
    cv::Mat tsh_mask_right = M3::tsh_mask(cam_images_gray[RIGHT], TSH_LOWER, TSH_UPPER);

    // Bitwise and the masks
    cv::Mat left_mask, right_mask;
    cv::bitwise_and(ROI_mask_left, tsh_mask_left, left_mask);
    cv::bitwise_and(ROI_mask_right, tsh_mask_right, right_mask);

    // Convert color to Grayscale
    cv::cvtColor(cam_images_gray[0], cam_images_gray[0], cv::COLOR_BGR2GRAY);
    cv::cvtColor(cam_images_gray[1], cam_images_gray[1], cv::COLOR_BGR2GRAY);

    // Remove noise before canny, always.
    cv::GaussianBlur(cam_images_gray[0], cam_images_gray[0], GAUSS_BLUR_KERNEL, GAUSS_BLUR_STD);
    cv::GaussianBlur(cam_images_gray[1], cam_images_gray[1], GAUSS_BLUR_KERNEL, GAUSS_BLUR_STD);

    // Morph the images with open, do a extra dilate to ensure.
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3,3), cv::Point(1, 1));
    cv::morphologyEx(left_mask, left_mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(right_mask, right_mask, cv::MORPH_OPEN, kernel);
    cv::dilate(left_mask, left_mask, kernel);
    cv::dilate(right_mask, right_mask, kernel);

    // Perform canny.
    cv::Canny(left_mask,  left_mask,  0, 100, 3, false);
    cv::Canny(right_mask, right_mask, 0, 100, 3, false);

    // Find contour left_right
    cv::Mat contour_left = M3::find_contour(left_mask);
    cv::Mat contour_right = M3::find_contour(right_mask);

    // Track the good features, this is lambda, very specific no reason for making this a function
    auto good_feature_tracker = [&qual](cv::Mat & img, std::vector<cv::Point2d> & corners, const cv::Mat & mask, bool draw, double min_dist = 50, int max_corner = 4)
    {
        cv::goodFeaturesToTrack( mask, corners, max_corner, qual, min_dist, cv::Mat(), 5, false);
        if(draw)
            for (const auto & corner : corners)
                cv::circle(img, corner, 10, cv::Scalar(128), 2, 8);
        return corners;
    };

    // Find the good corner features
    std::vector<cv::Point2d> corner_pts_left;
    std::vector<cv::Point2d> corner_pts_right;
    auto corner_left = good_feature_tracker(cam_images_gray[0], corner_pts_left, contour_left, 1, MIN_DIST_GOOD_FEATURE, MAX_CORNERS_GOOD_FEATURE);
    auto corner_right = good_feature_tracker(cam_images_gray[1], corner_pts_right, contour_right, 1, MIN_DIST_GOOD_FEATURE, MAX_CORNERS_GOOD_FEATURE);

    // Brute force matching
    std::vector<std::array<cv::Point2d, 2>> pts;
    for ( const auto & left_pt : corner_pts_left )
    {
        int k = 0;
        double dist = std::numeric_limits<double>::max();
        for ( int i = 0; i < corner_pts_right.size(); ++i )
            if (auto eu_cmp = M3::L2_stereo(left_pt, corner_pts_right[i]); eu_cmp < dist)
            {
                dist = eu_cmp;
                k = i;
            }
        pts.push_back(std::array<cv::Point2d, 2>{left_pt, corner_pts_right[k]});
    }

    if (pts.size() != 4)
    {
        ROS_INFO_STREAM("No solution was found");
        return -1;
    }

    // Stich the images
    const double dx = cam_images_color[1].cols-1;

    cv::Mat img_stitched = cv::Mat::zeros ( cv::Size(cam_images_gray[0].cols * 2, cam_images_gray[0].rows), CV_8UC3 );

    cam_images_color[0].copyTo(img_stitched(cv::Rect(0, 0, cam_images_color[0].cols, cam_images_color[0].rows)));
    cam_images_color[1].copyTo(img_stitched(cv::Rect(dx, 0, cam_images_color[1].cols, cam_images_color[1].rows)));

    cv::putText(img_stitched, "Left Image", cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2, cv::LINE_AA);
    cv::putText(img_stitched, "Right Image", cv::Point(50 + dx, 50), cv::FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2, cv::LINE_AA);

    for(auto pt : pts)
    {
        pt[1].x += dx;
        cv::line(img_stitched, pt[0], pt[1], cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)));
    }

    // Define the Q matrix
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> K_left(cam_info_arr[0].K.data());

    cv::Mat Q = ( cv::Mat_<double>(4,4) << 1., 0.,  0.,         -K_left(0,2),
                                           0., 1.,  0.,         -K_left(1,2),
                                           0., 0.,  0.,          K_left(1,1),
                                           0., 0., -1/BASELINE,          0.0);
    
    // Stereo triangulation
    cv::Mat M = M3::stereo_triangulation(pts, Q);

    // Take an arbitrary point, find the diagonal.
    cv::Point2d origo(M.at<double>(0, 0), M.at<double>(1, 0));

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

    // Calculate the coordinate system for pose generation
    std::vector<int> pt_idx;
    for (int i = 1; i < 4; ++i)
    {
        if (i != longest_idx)
        {
            pt_idx.push_back(i);
        }
    }

    // Validate the coordinate system
    cv::Point2d l1 = cv::Point2d( M.at<double>(0, pt_idx[0]) - origo.x, M.at<double>(1, pt_idx[0]) - origo.y);
    cv::Point2d l2 = cv::Point2d( M.at<double>(0, pt_idx[1]) - origo.x, M.at<double>(1, pt_idx[1]) - origo.y);
    double cross_val = l1.cross(l2);
    
    if(cross_val > 0)
    {
        std::swap(pt_idx[0], pt_idx[1]);
        std::swap(l1, l2);
    }
    
    // Draw the pose_estimation, rhs-coordinate system
    cv::arrowedLine(img_stitched, pts[0][0], pts[pt_idx[0]][0], cv::Scalar(0, 255, 0), 2);
    cv::arrowedLine(img_stitched, pts[0][0], pts[pt_idx[1]][0], cv::Scalar(0, 0, 255), 2);
    cv::arrowedLine(img_stitched, cv::Point2d(pts[0][1].x + dx, pts[0][1].y), cv::Point2d(pts[pt_idx[0]][1].x + dx, pts[pt_idx[0]][1].y), cv::Scalar(0, 255, 0), 2);
    cv::arrowedLine(img_stitched, cv::Point2d(pts[0][1].x + dx, pts[0][1].y), cv::Point2d(pts[pt_idx[1]][1].x + dx, pts[pt_idx[1]][1].y), cv::Scalar(0, 0, 255), 2);
    
    // Get the longest line, two gripping poses will be available, compute them both.

    // Return the pose estimation
    auto toc = ros::Time::now();
    ROS_INFO_STREAM("Time spent: " << (toc-tic).toSec() );

    cv::imwrite("img_stitched.jpg", img_stitched );
    
    return 0;
}