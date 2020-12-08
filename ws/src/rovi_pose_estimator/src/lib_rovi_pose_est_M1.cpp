#include <iostream>
#include <vector>
#include <tuple>

#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int32.h>

#include <Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>

#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/imgcodecs.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/flann.hpp>
#include <opencv4/opencv2/calib3d.hpp>
#include <opencv4/opencv2/stereo.hpp>
#include <opencv4/opencv2/ximgproc/disparity_filter.hpp>
#include <opencv4/opencv2/core/eigen.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/spin_image.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/icp.h>

// #include <pcl/surface/mls.h>
// #include <pcl/surface/impl/mls.hpp>
// #include <pcl/surface/bilateral_upsampling.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <pcl/common/random.h>
#include <pcl/common/time.h>
#include <pcl/common/transforms.h>

#include <rovi_pose_estimator/rovi_pose_est.h>
#include <rovi_gazebo/rovi_gazebo.h>
#include <rovi_utils/rovi_utils.h>

namespace rovi_pose_estimator
{

// Scene
static pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_scene_ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
static pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr filtered_scene_ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
static pcl::PointCloud<pcl::Histogram<153>>::Ptr features_scene_ptr(new pcl::PointCloud<pcl::Histogram<153>>());

// Object
static pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_object_ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
static pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr filtered_object_ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
static pcl::PointCloud<pcl::Histogram<153>>::Ptr features_object_ptr(new pcl::PointCloud<pcl::Histogram<153>>());

// Correspondences
static pcl::Correspondences corr;

std::array<cv::Mat, 2> 
M1::get_image_data(const std::string & ns_ros)
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

void
M1::set_structed_light(ros::NodeHandle & nh, const bool & state)
{
    static auto handler = nh.advertise<std_msgs::Int32>("/projector_controller/projector", 1);
    
    // spam the buffer
    ros::Rate lp(200);
    auto tic = ros::Time::now();
    while (1.0 > (ros::Time::now() - tic).toSec())
    {
        // ROS_INFO_STREAM((ros::Time::now() - tic).toSec());
        std_msgs::Int32 msg;
        msg.data = ( state == 1 ) ? 1 : 0;
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
        // get the message
        auto msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(ns_ros + cam);
        arr[idx] = *msg;
    }
    return arr;
}

cv::Mat
M1::compute_disparitymap(const cv::Mat & img_left, const cv::Mat & img_right)
{   
    // FileStorage
    // cv::FileStorage fs("stereosgbm_config.yaml", cv::FileStorage::READ);

    static auto min_disp          = 0;
    static auto num_disp          = 16*8;
    static auto block_size        = 3; // has to be odd
    static auto cn                = 3;
    static auto p1_smoothness     = 0;
    static auto p2_smoothness     = 0; // he larger the values are, the smoother the disparity is
    static auto disp_12_max       = 1;
    static auto unique_ratio      = 5;
    static auto speckle_win_size  = 0;
    static auto specke_range      = 1;
    static auto filter_gap        = 2;
    static auto written           = 0;
    static auto lambda            = 0.5;
    static auto sigma             = 0.5;

    // // Check if the file exists
    // fs["written"] >> written;
    // ROS_INFO_STREAM_ONCE("Checking if the file exists stereosgbm_config.yaml exists, the state is: " << written);
    // ROS_INFO_STREAM_ONCE("If not, write to the file");

    // if (fs.isOpened() && written == 1)
    // {   
    //     //read from file
    //     fs["min_disp"]         >> min_disp;
    //     fs["num_disp"]         >> num_disp;
    //     fs["block_size"]       >> block_size;
    //     fs["cn"]               >> cn;
    //     fs["p1_smoothness"]    >> p1_smoothness;
    //     fs["p2_smoothness"]    >> p2_smoothness;
    //     fs["disp_12_max"]      >> disp_12_max;
    //     fs["unique_ratio"]     >> unique_ratio;
    //     fs["speckle_win_size"] >> speckle_win_size;
    //     fs["specke_range"]     >> specke_range;
    //     fs["filter_gap"]       >> filter_gap;
    //     fs["lambda"]           >> lambda;
    //     fs["sigma"]            >> sigma;
    // }
    // else
    // {
    //     // write to file
    //     cv::FileStorage fs1("stereosgbm_config.yaml", cv::FileStorage::WRITE);
    //     fs1 << "written"          << 1;
    //     fs1 << "min_disp"         << min_disp;
    //     fs1 << "num_disp"         << num_disp;
    //     fs1 << "block_size"       << block_size;
    //     fs1 << "cn"               << cn;
    //     fs1 << "p1_smoothness"    << p1_smoothness;
    //     fs1 << "p2_smoothness"    << p2_smoothness;
    //     fs1 << "disp_12_max"      << disp_12_max;
    //     fs1 << "unique_ratio"     << unique_ratio;
    //     fs1 << "speckle_win_size" << speckle_win_size;
    //     fs1 << "specke_range"     << specke_range;
    //     fs1 << "filter_gap"       << filter_gap;
    //     fs1 << "sigma"            << sigma;
    //     fs1 << "lambda"           << lambda;
    // }  

    // // Write the configurations
    // ROS_INFO_STREAM( min_disp           << ", " << 
    //                  num_disp           << ", " <<  
    //                  block_size         << ", " <<
    //                  cn                 << ", " << 
    //                  p1_smoothness      << ", " <<
    //                  p2_smoothness      << ", " <<
    //                  disp_12_max        << ", " <<
    //                  unique_ratio       << ", " <<
    //                  speckle_win_size   << ", " <<
    //                  specke_range       << ", " <<
    //                  filter_gap
    //                 );
    
    // Setup the matchers
    cv::Ptr<cv::StereoSGBM> left_matcher = cv::StereoSGBM::create(
                                                                min_disp, 
                                                                num_disp, 
                                                                block_size, 
                                                                p1_smoothness, 
                                                                p2_smoothness, 
                                                                disp_12_max, 
                                                                filter_gap, 
                                                                unique_ratio, 
                                                                speckle_win_size, 
                                                                specke_range, 
                                                                cv::StereoSGBM::MODE_SGBM_3WAY
                                                              );

    // Use the recomended solvers from OpenCV
    cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls = cv::ximgproc::createDisparityWLSFilter(left_matcher);
    cv::Ptr<cv::StereoMatcher> right_matcher = cv::ximgproc::createRightMatcher(left_matcher);

    // For plotting the sets, define 5 cv::Mat
    cv::Mat disp_left, disp_right, disp_filtered, img_left_temp, img_right_temp, point_cloud;

    // Color convert
    cv::cvtColor(img_left,  img_left_temp, cv::COLOR_BGR2GRAY);
    cv::cvtColor(img_right, img_right_temp, cv::COLOR_BGR2GRAY);

    // Remove noise before canny, always.
    cv::GaussianBlur(img_left_temp, img_left_temp, GAUSS_BLUR_KERNEL, GAUSS_BLUR_STD);
    cv::GaussianBlur(img_right_temp, img_right_temp, GAUSS_BLUR_KERNEL, GAUSS_BLUR_STD);

    // Compute left and right disparity
    left_matcher->compute(img_left_temp, img_right_temp, disp_left);
    right_matcher->compute(img_right_temp, img_left_temp, disp_right);

    // Computed weighted least squares optimization
    wls->setLambda(lambda);
    wls->setSigmaColor(sigma);
    wls->filter(disp_left, img_left, disp_filtered, disp_right);

    // Compute Disparity Map
    cv::ximgproc::getDisparityVis(disp_filtered, disp_filtered, 1.0);
    // cv::imwrite("disparity_map.jpg", disp_filtered);

    // Define the Camera matrix
    static auto cam_info_arr = M1::get_image_info();
    static Eigen::Matrix<double, 3, 3, Eigen::RowMajor> K_left(cam_info_arr[LEFT].K.data());

    // Compute the Q-matrix
    static cv::Mat Q = (cv::Mat_<double>(4, 4)<< 1.f, 0.f,  0.f, -K_left(0,2),
                                                 0.f, 1.f,  0.f, -K_left(1,2),
                                                 0.f, 0.f,  0.f,  K_left(1,1),
                                                 0.f, 0.f, -1.f/BASELINE, 0.f );

    // Reproject the image to 3D
    cv::reprojectImageTo3D(disp_filtered, point_cloud, Q, false);

    return point_cloud;
};

void
M1::compute_pointcloud_scene(const cv::Mat & point_cloud, const cv::Mat & left_img)
{
    // timing
    auto tic = ros::Time::now();

    // cloud_ptr set width etc
    cloud_scene_ptr->width = point_cloud.cols;
    cloud_scene_ptr->height = point_cloud.rows;
    cloud_scene_ptr->is_dense = false;
    cloud_scene_ptr->resize(cloud_scene_ptr->width * cloud_scene_ptr->height);

    // these are reuqired for the for loop
    for (int i = 0, m = 0, k = 0; i < point_cloud.rows; ++i)
    {
        const float* point_cloud_ele = point_cloud.ptr<float>(i);
        const uchar* rbg_left = left_img.ptr<uchar>(i);
        m = 0;

        for(int j = 0; j < point_cloud.cols * 3; )
        {
            

            std::uint8_t b = (std::uint8_t) rbg_left[m++];
            std::uint8_t g = (std::uint8_t) rbg_left[m++];
            std::uint8_t r = (std::uint8_t) rbg_left[m++];

            cloud_scene_ptr -> points[k].x = point_cloud_ele[j++];
            cloud_scene_ptr -> points[k].y = point_cloud_ele[j++];
            cloud_scene_ptr -> points[k].z = point_cloud_ele[j++];

            std::uint32_t rgb = ( (uint32_t) r << 24 | (uint32_t) g << 16 | (uint32_t) b << 8);
                
            cloud_scene_ptr -> points[k++].rgb = *reinterpret_cast<float*>(&rgb);
        }
    }

    // Compute the transformation matrix from world to gazebo camera
    static Eigen::Affine3d w_T_gaze;    
    tf::poseMsgToEigen(rovi_gazebo::get_model_pose("camera_stereo"), w_T_gaze);

    // Get constant OPENGL transformation
    static Eigen::Matrix4f gaze_T_c  = ( Eigen::Matrix4f() << 0.f, 0.f, 1.f, 0.f, 
                                                             -1.f, 0.f, 0.f, 0.f, 
                                                              0.f,-1.f, 0.f, 0.f, 
                                                              0.f, 0.f, 0.f, 1.f ).finished();

    // Define the overall transformation
    static Eigen::Matrix4f trans = (w_T_gaze.cast<float>()).matrix() * gaze_T_c;

    // Transform the point cloud 
    pcl::transformPointCloud(*cloud_scene_ptr, *cloud_scene_ptr, trans);

    // Cropbox
    static pcl::CropBox<pcl::PointXYZRGBNormal> box_filter;
    {
        box_filter.setMin(Eigen::Vector4f(0.0, 0.85, 0.70, 1.0f));
        box_filter.setMax(Eigen::Vector4f(0.8, 1.25, 1.25, 1.0f));
        box_filter.setInputCloud(cloud_scene_ptr);
        box_filter.filter(*filtered_scene_ptr);
        *cloud_scene_ptr = *filtered_scene_ptr;
    }

    // pcl::io::savePCDFile("cloud_scene.pcd", *cloud_scene_ptr);  

    // Plane fitting
    constexpr auto dist_tsh = 0.01f;
    static pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);    
    static pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    static pcl::SACSegmentation<pcl::PointXYZRGBNormal> segm;
    {
        segm.setOptimizeCoefficients(true);
        segm.setModelType(pcl::SACMODEL_PLANE);
        segm.setMethodType(pcl::SAC_RANSAC);
        segm.setDistanceThreshold(dist_tsh);
        segm.setInputCloud(cloud_scene_ptr);
        segm.segment(*inliers, *coeff);

        ROS_INFO_STREAM("Number of inliers for plane_segmentation: " << inliers->indices.size());

        if (inliers->indices.size() != 0 )
        {
            double zmin = 0;
            for (const auto &idx : inliers->indices)
                zmin += cloud_scene_ptr->points[idx].z;
            zmin /= inliers->indices.size();

            // Create filtering object  
            pcl::PassThrough<pcl::PointXYZRGBNormal> pass;
            pass.setInputCloud(cloud_scene_ptr);
            pass.setFilterFieldName("z");
            pass.setFilterLimits(zmin + 0.015, zmin + 1);
            pass.setFilterLimitsNegative(false);
            pass.filter(*filtered_scene_ptr);
            *cloud_scene_ptr = *filtered_scene_ptr;
        }
    }

    // Voxel Filter
    static pcl::VoxelGrid<pcl::PointXYZRGBNormal> voxel_filter;
    {
        voxel_filter.setInputCloud(cloud_scene_ptr);
        voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
        voxel_filter.filter(*filtered_scene_ptr);
        *cloud_scene_ptr = *filtered_scene_ptr;
    }

    // Statistical outlier removal
    static pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBNormal> sor;
    {
        sor.setInputCloud(cloud_scene_ptr);
        sor.setMeanK(10);
        sor.setStddevMulThresh(1);
        sor.filter(*filtered_scene_ptr);
        *cloud_scene_ptr = *filtered_scene_ptr;
    }

    // Estimate normals from PoV
    static pcl::NormalEstimation<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> normal_est;
    {
        pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBNormal>());
        normal_est.setInputCloud(cloud_scene_ptr);
        normal_est.setViewPoint(trans(0,3), trans(1,3), trans(2,3));
        normal_est.setSearchMethod(tree);
        normal_est.setKSearch(25);
        normal_est.compute(*cloud_scene_ptr);
    }

    static pcl::SpinImageEstimation<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, pcl::Histogram<153>> spin;
    {
        spin.setInputCloud(cloud_scene_ptr);
        spin.setInputNormals(cloud_scene_ptr);
        spin.setRadiusSearch(0.15);
        spin.compute(*features_scene_ptr);
    }

    // ROS_INFO_STREAM("The scene_voxel" << cloud_scene_ptr->size());

    // Save the pointcloud 
    // pcl::io::savePCDFileASCII("scene_voxel.pcd", *cloud_scene_ptr);

    // exit(1);

    // Clear it, to avoid segmentation fault

    auto toc = ros::Time::now();
    auto delta = toc-tic;
    // ROS_INFO_STREAM("Time spent in preprocessing dense stereo scene: " << delta.toSec() << "ms");
}

bool
M1::read_compute_features_object(const std::string & obj)
{
    // tic
    auto tic = ros::Time::now();

    int error = pcl::io::loadPCDFile<pcl::PointXYZRGBNormal>(obj, *cloud_object_ptr);

    // ROS_INFO_STREAM("Cloud object ptr size := " << cloud_object_ptr->size());

    if(error != 0)
    {
        // ROS_INFO_STREAM("The filename is wrong, it was not possible to load the object.");
        return false;
    }

    // Compute the transformation matrix from world to gazebo camera
    static Eigen::Affine3d w_T_gaze;    
    tf::poseMsgToEigen(rovi_gazebo::get_model_pose("camera_stereo"), w_T_gaze);

    // Get constant OPENGL transformation
    static Eigen::Matrix4f gaze_T_c  = ( Eigen::Matrix4f() << 0.f, 0.f, 1.f, 0.f, 
                                                             -1.f, 0.f, 0.f, 0.f, 
                                                              0.f,-1.f, 0.f, 0.f, 
                                                              0.f, 0.f, 0.f, 1.f ).finished();
    // Define the overall transformation
    static Eigen::Matrix4f trans = (w_T_gaze.cast<float>()).matrix() * gaze_T_c;

    // Give some arbitrary colours to the object
    for (size_t i = 0; i < cloud_object_ptr->size(); i++)
    {
        std::uint8_t b = (std::uint8_t) 200;
        std::uint8_t g = (std::uint8_t) 150;
        std::uint8_t r = (std::uint8_t) 128;
        std::uint32_t rgb = ( (uint32_t) r << 24 | (uint32_t) g << 16 | (uint32_t) b << 8);
        cloud_object_ptr -> points[i].rgb = *reinterpret_cast<float*>(&rgb);
    }

    // Estimate normals from PoV
    static pcl::NormalEstimation<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> normal_est;
    {
        pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBNormal>());
        normal_est.setInputCloud(cloud_object_ptr);
        normal_est.setViewPoint(trans(0,3), trans(1,3), trans(2,3));
        normal_est.setSearchMethod(tree);
        normal_est.setKSearch(25);
        normal_est.compute(*cloud_object_ptr);
    }

    static pcl::SpinImageEstimation<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, pcl::Histogram<153>> spin;
    {
        spin.setInputCloud(cloud_object_ptr);
        spin.setInputNormals(cloud_object_ptr);
        spin.setRadiusSearch(0.15);
        spin.compute(*features_object_ptr);
    }

    // toc
    auto toc = ros::Time::now();
    // ROS_INFO_STREAM("Time spent in compute_features_object: " << (toc-tic).toSec());

    // Save the pointcloud 
    //pcl::io::savePCDFileASCII("voxel_object.pcd", *cloud_object_ptr);

    return true;
}

void 
M1::match_features()
{
    // Tic
    auto tic = ros::Time::now();

    // Define the L2_norm operator
    static auto L2_norm = [](const pcl::Histogram<153> & f1, const pcl::Histogram<153> & f2)
    {
        float sum = 0.f;
        for(auto i = 0; i < f1.descriptorSize(); i++)
            sum += std::pow((f1.histogram[i]-f2.histogram[i]), 2);
        return sum;
    };

    // Define nearest neightbour
    static auto nearest_neighbour = [](const pcl::PointCloud<pcl::Histogram<153>>::Ptr & query, const int idx_query, const pcl::PointCloud<pcl::Histogram<153>>::Ptr & scene, int & idx_scene)
    {
        float min_dist = std::numeric_limits<float>::max();

        for(auto i = 0; i < scene->size(); i++)
        {
            float dist = L2_norm(query->points[idx_query], scene->points[i]);
            if (min_dist > dist)
            {
                min_dist = dist;
                idx_scene = i;
            }
        }

        return min_dist;
    };

    // Correspondences are reset.
    corr.clear();
    corr.resize(features_object_ptr->size());

    // Start matching
    int idx_scene = 0;
    for(auto idx_query = 0; idx_query < features_object_ptr->size(); idx_query++)
    {
        corr[idx_query].index_query    = idx_query;
        corr[idx_query].distance       = nearest_neighbour(features_object_ptr, idx_query, features_scene_ptr, idx_scene);
        corr[idx_query].index_match    = idx_scene;
    }

    // Toc
    auto toc = ros::Time::now();
    // ROS_INFO_STREAM("Time spent in compute_features_object: " << (toc-tic).toSec());
}

Eigen::Matrix4f 
M1::ransac_features(const int & max_it)
{
    // Note
    ROS_INFO_STREAM("Amount of iterations: " << max_it);

    // Metrics
    int max_inliers = 0;
    Eigen::Matrix4f best_T, T;

    // Variables
    std::vector<int> query(3), scene(3);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr aligned_object_ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>());

    // Random generator
    pcl::common::UniformGenerator<int> rnd_gen(0, corr.size() - 1);
    rnd_gen.setSeed(time(0));

    // Define search tree
    pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> kd_tree_scene;
    kd_tree_scene.setInputCloud(cloud_scene_ptr);

    for(auto i = 0; i < max_it; i++)
    {   
        // How far are we?
        if(i % 500 == 0)
        {
            ROS_INFO_STREAM("Iteration: " << i);
        }

        // Pick 3 points
        for (auto j = 0; j < 3; j++)
        {
            int random = rnd_gen.run();
            query[j] = corr[random].index_query;
            scene[j] = corr[random].index_match;
        }
        // Determine the transformation between cloud and cloud_object, we use the object and the scene
        pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> svd;
        svd.estimateRigidTransformation(*cloud_object_ptr, query, *cloud_scene_ptr, scene, T);

        // Perform the transformation on the two clouds, here we usedaligned
        pcl::transformPointCloud(*cloud_object_ptr, *aligned_object_ptr, T);

        // Perform NN
        int K = 1;
        std::vector<int> point_idx_nn_search(K);
        std::vector<float> point_nn_L2(K);
        
        // Inlier
        int inliers = 0;

        for (auto j = 0; j < aligned_object_ptr->size(); j++)
        {
            if ( kd_tree_scene.nearestKSearch(aligned_object_ptr->points[j], K, point_idx_nn_search, point_nn_L2) > 0 )
            {   
                if (point_nn_L2[0] < 0.00001f)
                {
                    inliers++;
                }
            }
        }

        if (inliers > max_inliers)
        {
            ROS_INFO_STREAM("Inliers: " << inliers);
            max_inliers = inliers;
            best_T = T;
        }
    }

    pcl::transformPointCloud(*cloud_object_ptr, *cloud_object_ptr, best_T);

    // pcl::IterativeClosestPoint<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal > icp;
    // icp.setInputCloud(cloud_object_ptr);
    // icp.setInputTarget(cloud_scene_ptr);
    // icp.setMaxCorrespondenceDistance(0.01);
    // icp.setMaximumIterations(100);
    // icp.align(*cloud_object_ptr);

    // pcl::visualization::PCLVisualizer viewer("vis");
    // viewer.addPointCloud<pcl::PointXYZRGBNormal>(cloud_scene_ptr, "scene");
    // viewer.addPointCloud<pcl::PointXYZRGBNormal>(cloud_object_ptr, "obj");
    // viewer.spin();

    return best_T; //;

}

geometry_msgs::Pose
M1::estimate_pose(const int & it, const bool & draw, const double & noise)
{
    // Get stereo images
    auto cam_images = M1::get_image_data();

    cv::Mat gaussian_noise_left = cv::Mat::zeros(cam_images[0].size(), cam_images[0].type());
    cv::Mat gaussian_noise_right = gaussian_noise_left;
    
    std::vector<double> mean = {0, 0, 0};
    std::vector<double> std = {noise, noise, noise};

    cv::randn(gaussian_noise_left, mean, std);
    cv::randn(gaussian_noise_right, mean, std);

    if(draw)
    {
        cv::imwrite("left.jpg", gaussian_noise_left);
    }

    cam_images[0] += gaussian_noise_left;
    cam_images[1] += gaussian_noise_right;

    // Compute the disparity map
    cv::Mat point_cloud = M1::compute_disparitymap(cam_images[LEFT], cam_images[RIGHT]);
    
    if(draw)
    {
        cv::imwrite("bottle_left.jpg",  cam_images[0]);
        cv::imwrite("bottle_right.jpg", cam_images[1]);
        std::ofstream write_cloud("write_mat.mat", std::ios_base::out);
        write_cloud << cv::format(point_cloud, cv::Formatter::FMT_CSV) << std::endl;
        write_cloud.close();
    }
    
    // cv::imwrite("point_cloud.jpg",  point_cloud);
    // Compute the point cloud
    M1::compute_pointcloud_scene(point_cloud, cam_images[0]);

    // std::string read_file = "test.pcd";
    static auto file_bottle_pcd = ros::package::getPath("rovi_gazebo") + std::string("/models/bottle/bottle.pcd");
    
    // ROS_INFO_STREAM(file_bottle_pcd);

    M1::read_compute_features_object(file_bottle_pcd);
    M1::match_features();
    auto T = Eigen::Affine3d(M1::ransac_features(it).cast<double>());
    geometry_msgs::Pose pose;
    tf::poseEigenToMsg(T, pose);

    return pose;
}


}