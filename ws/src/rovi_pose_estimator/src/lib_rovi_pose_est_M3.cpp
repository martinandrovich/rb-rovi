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
    cv::Mat T = ( cv::Mat_<double>(4,4) <<  0.f, 0.f, 1.f, 0.f,
                                           -1.f, 0.f, 0.f, 0.f,
                                            0.f,-1.f, 0.f, 0.f,
                                            0.f, 0.f, 0.f, 1.f );

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
        M.at<double>(3, i) = M.at<double>(3, i) / w;
    }

    return M;
}

geometry_msgs::Pose 
M3::estimate_pose(const bool & draw, const std::string & img_name, const double & noise, const double & qual, const int & max_number_of_corners, const double & min_dist_features)
{
    geometry_msgs::Pose pose;

    // Get intrinsic / extrinsic parameters / get images
    auto cam_info_arr     = M3::get_image_info();
    auto cam_images_color = M3::get_image_data();
    auto cam_images_gray  = cam_images_color;
    
    cv::Mat gaussian_noise_left = cv::Mat::zeros(cam_images_color[0].size(), cam_images_color[0].type());
    cv::Mat gaussian_noise_right = gaussian_noise_left;
    
    std::vector<double> mean = {0, 0, 0};
    std::vector<double> std = {noise, noise, noise};

    cv::theRNG().state = cv::getTickCount();

    cv::randn(gaussian_noise_left, mean, std);
    cv::randn(gaussian_noise_right, mean, std);

    cv::imwrite("left.jpg", gaussian_noise_left);

    cam_images_color[0] += gaussian_noise_left;
    cam_images_color[1] += gaussian_noise_right;
    cam_images_gray[0] += gaussian_noise_left;
    cam_images_gray[1] += gaussian_noise_right;

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
    cv::cvtColor(cam_images_gray[LEFT], cam_images_gray[LEFT], cv::COLOR_BGR2GRAY);
    cv::cvtColor(cam_images_gray[RIGHT], cam_images_gray[RIGHT], cv::COLOR_BGR2GRAY);

    // Remove noise before canny, always.
    cv::GaussianBlur(cam_images_gray[LEFT], cam_images_gray[LEFT], GAUSS_BLUR_KERNEL, GAUSS_BLUR_STD);
    cv::GaussianBlur(cam_images_gray[RIGHT], cam_images_gray[RIGHT], GAUSS_BLUR_KERNEL, GAUSS_BLUR_STD);

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
    auto good_feature_tracker = [&qual](cv::Mat & img, std::vector<cv::Point2d> & corners, const cv::Mat & mask, const bool & draw, const double & min_dist = 50, const int & max_corner = 4)
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
    auto corner_left = good_feature_tracker(cam_images_gray[LEFT], corner_pts_left, contour_left, draw, min_dist_features, max_number_of_corners);
    auto corner_right = good_feature_tracker(cam_images_gray[RIGHT], corner_pts_right, contour_right, draw, min_dist_features, max_number_of_corners);

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

    // Sort it 
    std::sort(pts.begin(), pts.end(), [](const std::array<cv::Point2d, 2> & a, const std::array<cv::Point2d, 2> & b) 
    {
        if( a[0].y < b[0].y )
            return true;
        else
            return false;
    });

    if (pts.size() == 0)
    {
        ROS_INFO_STREAM("No solution was found");
        return pose;
    }

    // Stich the images
    const double dx = cam_images_color[LEFT].cols-1;

    // Define the Q matrix
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> K_left(cam_info_arr[LEFT].K.data());

    cv::Mat Q = ( cv::Mat_<double>(4,4) <<  1.0, 0.0, 0.0,-K_left(0,2),
                                            0.0, 1.0, 0.0,-K_left(1,2),
                                            0.0, 0.0, 0.0, K_left(1,1),
                                            0.0, 0.0,-1/BASELINE, 0.0);
    
    // Stereo triangulation
    cv::Mat M = M3::stereo_triangulation(pts, Q);

    //
    ROS_INFO_STREAM("\n"<< M);

    // Take an arbitrary point, find the diagonal.
    cv::Point2d origo(M.at<double>(0, 0), M.at<double>(1, 0));

    int longest_idx = 0;

    double dist = std::numeric_limits<double>::min();
    for (auto i = 1; i < pts.size(); i++)
    {  
        if (auto distance = cv::norm( origo - cv::Point2d(M.at<double>(0, i), M.at<double>(1, i) ) ); dist < distance)
        {
            dist = distance;
            longest_idx = i;
        }
    }

    // Calculate the coordinate system for pose generation by getting the indices
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

    // Update l1/v2
    auto L1_norm = sqrt(std::pow(l1.x, 2) + std::pow(l1.y, 2));
    auto L2_norm = sqrt(std::pow(l2.x, 2) + std::pow(l2.y, 2));

    l1 /= sqrt(l1.dot(l1));
    l2 /= sqrt(l2.dot(l2));

    auto error = l1.dot(l2);
    auto y_orth = l1 - (error/2)*l2;
    auto x_orth = l2 - (error/2)*l1;

    l1 = 0.5*(3.0-x_orth.dot(x_orth))*x_orth;
    l2 = 0.5*(3.0-y_orth.dot(y_orth))*y_orth;
    
    if (draw)
    {

        // Imgage stiched
        cv::Mat img_stitched = cv::Mat::zeros(cv::Size(cam_images_gray[LEFT].cols * 2, cam_images_gray[LEFT].rows), CV_8UC3);

        cam_images_color[0].copyTo(img_stitched(cv::Rect(0, 0, cam_images_color[LEFT].cols, cam_images_color[LEFT].rows)));
        cam_images_color[1].copyTo(img_stitched(cv::Rect(dx, 0, cam_images_color[RIGHT].cols, cam_images_color[RIGHT].rows)));

        cv::putText(img_stitched, "Left Image", cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2, cv::LINE_AA);
        cv::putText(img_stitched, "Right Image", cv::Point(50 + dx, 50), cv::FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2, cv::LINE_AA);

        for(auto pt : pts)
        {
            pt[1].x += dx;
            cv::line(img_stitched, pt[0], pt[1], cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)));
        }

        auto pt = cv::Point2d(pts[0][0].x + l1.x * 50, pts[0][0].y + l1.y * 50);

        // Draw the pose_estimation, rhs-coordinate system and with a blue arrow
        cv::arrowedLine( img_stitched, pts[0][0], pts[pt_idx[0]][0], cv::Scalar(0, 255, 0), 2 );
        cv::arrowedLine( img_stitched, pts[0][0], pts[pt_idx[1]][0], cv::Scalar(0, 0, 255), 2 );
        cv::arrowedLine( img_stitched, pts[0][0], pts[longest_idx][0], cv::Scalar(255, 0, 0), 2 );
        cv::arrowedLine( img_stitched, cv::Point2d(pts[0][1].x + dx, pts[0][1].y), cv::Point2d(pts[pt_idx[0]][1].x + dx, pts[pt_idx[0]][1].y), cv::Scalar(0, 255, 0), 2 );
        cv::arrowedLine( img_stitched, cv::Point2d(pts[0][1].x + dx, pts[0][1].y), cv::Point2d(pts[pt_idx[1]][1].x + dx, pts[pt_idx[1]][1].y), cv::Scalar(0, 0, 255), 2 );
        cv::arrowedLine( img_stitched, cv::Point2d(pts[0][1].x + dx, pts[0][1].y), cv::Point2d(pts[longest_idx][1].x + dx, pts[longest_idx][1].y), cv::Scalar(255, 0, 0), 2 );

        // Get the longest line, two gripping poses will be available, compute them both.
        cv::imwrite(img_name, img_stitched);
    }

    geometry_msgs::Pose pose_T;

    Eigen::Matrix4d eig_pose = ( Eigen::Matrix4d() << l1.x, l2.x, 0.0, M.at<double>(0, 0) + (M.at<double>(0, longest_idx) - M.at<double>(0, 0))/2.0,
                                                      l1.y, l2.y, 0.0, M.at<double>(1, 0) + (M.at<double>(1, longest_idx) - M.at<double>(1, 0))/2.0,
                                                       0.0, 0.0, 1.0, 0.75,
                                                       0.0, 0.0, 0.0, 1.0 ).finished();

    Eigen::Affine3d affine(eig_pose);
    tf::poseEigenToMsg(affine, pose);

    ROS_INFO_STREAM("\n" << eig_pose);
    // ROS_INFO_STREAM("\n" << eig_pose);

    return pose;
}

}