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
        global_pose_est(const pcl::PointCloud<pcl::PointXYZ>::Ptr& scene, const pcl::PointCloud<pcl::PointXYZ>::Ptr& obj, pcl::PointCloud<pcl::PointXYZ>::Ptr& output, float inlier_tresh=0.005, int ransac_iterations=5000);

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
    plane_segmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, pcl::PointIndices::Ptr& inlier_idices, pcl::ModelCoefficients::Ptr& plane_coeff);

    void
    extract_indices(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, pcl::PointIndices::Ptr& inlier_idices, pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud, bool inverse_extraction=false);

    void
    CheckforNans(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud);

    


	void
    test(const std::string& str);



   
    //void rovi_pose_estimator::
}
