#pragma once
#include <string>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/passthrough.h>

namespace rovi_pose_estimator
{
    namespace M2
    {
        //using namespace rovi_pose_estimator::M2;

        void
        get_depth_image();

        void
        get_point_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud);



    }
    
    void voxelfilter();
    void get_features();
    void match_features();
    void ICP();
    void global_pose_est();


	void
    test(const std::string& str);



   
    //void rovi_pose_estimator::
}
