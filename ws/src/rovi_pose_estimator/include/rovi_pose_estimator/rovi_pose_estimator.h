#pragma once

#include <string>
#include <array>

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Eigen>

namespace rovi_pose_estimator
{
	static cv::RNG rng(12345);
	static constexpr auto LEFT = 0;
	static constexpr auto RIGHT = 1;
	static constexpr auto BASELINE = -0.08;
	static constexpr auto IMG_SIZE_W = 800;
	static constexpr auto IMG_SIZE_H = 800;
	static constexpr auto GAUSS_BLUR_STD = 0.10;
	static const auto GAUSS_BLUR_KERNEL = cv::Size(3, 3);
	static const std::array<double, 3> TSH_LOWER = {220, 220, 220};
	static const std::array<double, 3> TSH_UPPER = {255, 255, 255};
	static const std::vector<cv::Point> ROI_left  { cv::Point{0, 0}, cv::Point{0, IMG_SIZE_H-1}, cv::Point{IMG_SIZE_W-1, IMG_SIZE_H-1}, cv::Point{IMG_SIZE_W-1, 0} };
	static const std::vector<cv::Point> ROI_right { cv::Point{0, 0}, cv::Point{0, IMG_SIZE_H-1}, cv::Point{IMG_SIZE_W-1, IMG_SIZE_H-1}, cv::Point{IMG_SIZE_W-1, 0} };
	static constexpr double MIN_DIST_GOOD_FEATURE = 50;
	static constexpr int MAX_CORNERS_GOOD_FEATURE = 4; 

	namespace M1
	{
		// constexpr auto leaf_size = 0.008f;
		constexpr auto leaf_size = 0.01f;

		std::array<cv::Mat, 2> 
		get_image_data(const std::string & ns_ros = "/rbrovi/camera_stereo");

		std::array<sensor_msgs::CameraInfo, 2>
		get_image_info(const std::string & ns_ros = "/rbrovi/camera_stereo");

		void
		set_structed_light(ros::NodeHandle & nh, const bool & state);

		cv::Mat 
		compute_disparitymap(const cv::Mat & img_left, const cv::Mat & img_right);

		void
		compute_pointcloud_scene(const cv::Mat & point_cloud, const cv::Mat & left_img);

		bool
		read_compute_features_object(const std::string & obj);

		void 
		match_features();

		Eigen::Matrix4f 
		ransac_features(const int & max_it = 10000);

		geometry_msgs::Pose
		estimate_pose(const int & it, const bool & draw = false, const double & noise = 0.0);
	}

	namespace M3
	{
		std::array<cv::Mat, 2> 
		get_image_data(const std::string & ns_ros = "/rbrovi/camera_stereo");         

		std::array<sensor_msgs::CameraInfo, 2>
		get_image_info(const std::string & ns_ros = "/rbrovi/camera_stereo");

		cv::Mat 
		create_mask(const cv::Mat & img, const std::vector<cv::Point> & pts);

		cv::Mat 
		tsh_mask(
			const cv::Mat & img_color, 
			const std::array<double, 3> & lower_rgb = { 230.f, 230.f, 230.f }, 
			const std::array<double, 3> & upper_rgb = { 255.f, 255.f, 255.f }
		);

		cv::Mat 
		find_contour(const cv::Mat & mask);

		double 
		L2_stereo(const cv::Point2d & left, const cv::Point2d & right, const double y_tsh = 10);

		cv::Mat 
		stereo_triangulation(const std::vector<std::array<cv::Point2d, 2>> & pts, const cv::Mat & Q, const bool & k_means = false);

		geometry_msgs::Pose 
		estimate_pose(
			const bool & draw = 1,
			const std::string & img_name = "img_stitched.jpg",
			const double & noise = 0.0,
			const double & qual = 0.01,
			const int & max_number_of_corners = 4,
			const double & min_dist_features = 50.0f
		);
	}
}
