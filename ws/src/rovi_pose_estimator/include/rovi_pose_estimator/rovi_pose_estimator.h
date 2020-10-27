#pragma once
#include <string>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

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
