#pragma once
#include <string>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/transformation_estimation_svd.h>

namespace rovi_pose_estimator
{
    namespace M2
    {
        void
        get_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud, bool save_to_disk=false);

        void 
        compute_spin_image_features(const pcl::PointCloud<pcl::PointXYZ>::Ptr& scene, const pcl::PointCloud<pcl::PointXYZ>::Ptr& obj, const pcl::PointCloud<pcl::Histogram<153>>::Ptr& scene_features, const pcl::PointCloud<pcl::Histogram<153>>::Ptr& obj_features);

        void 
        match_spin_image_features(const pcl::PointCloud<pcl::Histogram<153>>::Ptr& scene, const pcl::PointCloud<pcl::Histogram<153>>::Ptr& obj, std::vector<int>& scene_match_indices);
        
        const pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4 
        RANSAC_pose_est(const pcl::PointCloud<pcl::PointXYZ>::Ptr& scene, const pcl::PointCloud<pcl::PointXYZ>::Ptr& obj, std::vector<int>& scene_match_indices, int max_iterations, double inlier_threshold);

        void 
        global_pose_est(const pcl::PointCloud<pcl::PointXYZ>::Ptr& scene, const pcl::PointCloud<pcl::PointXYZ>::Ptr& obj, pcl::PointCloud<pcl::PointXYZ>::Ptr& output, pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4& transformation, float inlier_tresh=0.005, int ransac_iterations=5000);

        void 
        load_model(const std::string& model_path, const pcl::PointCloud<pcl::PointXYZ>::Ptr& dst);
    }
    
    void 
    compute_normals(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input, const pcl::PointCloud<pcl::Normal>::Ptr& normals);
    
    void 
    voxelGrid(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud, float leaf_size=0.02);

    void
    outlierRemoval(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud);

    void 
	spatialFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud);
    
    void 
    smoothing(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud);

    void
    plane_segmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, pcl::PointIndices::Ptr& inlier_idices, pcl::ModelCoefficients::Ptr& plane_coeff, float leaf_size);

    void
    extract_indices(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, pcl::PointIndices::Ptr& inlier_idices, pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud, bool inverse_extraction=false);

    void
    extract_indices(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud, pcl::PointIndices::Ptr& inlier_idices, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud, bool inverse_extraction);

    void
    CheckforNans(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud);

    void
    ICP(const pcl::PointCloud<pcl::PointXYZ>::Ptr& scene, const pcl::PointCloud<pcl::PointXYZ>::Ptr& model, pcl::PointCloud<pcl::PointXYZ>::Ptr& transformed_model, pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4& final_trans, float leaf_size=0.05, int max_iterations=50);
    
    void
    get_final_pose(const pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4& global_rot, const pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4& local_rot, const pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4& sensor_pose, pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4& final_pose);

	void
    test(const std::string& str);


    namespace M4
    {
        void 
        Harris_keypoints_example(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& model, pcl::PointIndices::Ptr& inlier_idices);

        void 
        Harris_corners_2d(const cv::Mat& image, std::vector<cv::Point2f>& corner_points, float quality_level=0.015, float min_dist=5.0, bool useharris=false, float filter_sigma=1.15);

        void 
        permute_point_matches(const pcl::PointCloud<pcl::PointXYZRGB>& model_corners, const std::vector<cv::Point2f>& image_corners, std::vector<cv::Point3f>& model_matches, std::vector<cv::Point2f>& image_matches);
    
        std::vector<cv::Point3f> 
		PCL_pointcloud_to_OPENCV_Point3d(const pcl::PointCloud<pcl::PointXYZRGB>& pointcloud);
        
        void 
		RANSAC_pose_estimation(const std::vector<cv::Point3f>& model_corners, const std::vector<cv::Point2f>& image_corners ,const std::vector<cv::Point3f>& model_matches, const std::vector<cv::Point2f>& image_matches, cv::Mat& pose_estimation, int max_iterations, const float inlier_radius=0.015f, bool approximate_camera_matrix=false, const cv::Mat* img=nullptr);
        
    }



   
    //void rovi_pose_estimator::
}
