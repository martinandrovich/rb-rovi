#include <iostream>
#include <vector>
#include <tuple>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int32.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/imgcodecs.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/flann.hpp>
#include <opencv4/opencv2/calib3d.hpp>
#include <opencv4/opencv2/stereo.hpp>
#include <opencv4/opencv2/ximgproc/disparity_filter.hpp>

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

#include <pcl/surface/mls.h>
#include <pcl/surface/impl/mls.hpp>
#include <pcl/surface/bilateral_upsampling.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <pcl/common/random.h>
#include <pcl/common/time.h>
#include <pcl/common/transforms.h>

#include <rovi_pose_estimator/rovi_pose_est_M1.h>
#include <rovi_utils/rovi_utils.h>

namespace rovi_pose_estimator
{

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

    if(first)
    {
        first = false;
        ros::Duration(0.5).sleep();
    }

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
M1::get_ROI(const cv::Mat & img_left, const cv::Mat & img_right, const cv::Mat & Q)
/*
*   this method finds the RoI 4 points needed from the table.
*/
{
    // time it for benchmarking
    auto tic = ros::Time::now();

    // the current method is not robust to noise
    cv::Mat temp_left, temp_right;

    // threshhold the images, done in matlab
    cv::inRange(img_left,  cv::Scalar(120, 170, 170), cv::Scalar(180, 255, 255), temp_left);
    cv::inRange(img_right, cv::Scalar(120, 170, 170), cv::Scalar(180, 255, 255), temp_right);

    // stage one convert image to grayscale
    cv::Canny(temp_left,  temp_left, 50, 200, 3);
    cv::Canny(temp_right, temp_right, 50, 200, 3);

    // contours, hulls, hier
    std::vector<std::vector<cv::Point>> contours;
    std::vector<std::vector<cv::Point>> hull;
    std::vector<cv::Point> roi_left, roi_right;
    std::vector<cv::Vec4i> hier;

    for (auto & [ img, roi ] : std::array{std::tuple(&temp_left, &roi_left), std::tuple(&temp_right, &roi_right)} )
    {
        cv::findContours(*img, contours, hier, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        hull.resize(contours.size());

        for (size_t i = 0; i < contours.size(); i++)
        {
            cv::convexHull(contours[i], hull[i]);
            cv::drawContours(*img, hull, 0, cv::Scalar(255));
        }

        int xMin = std::numeric_limits<int>::max();
        int xMax = std::numeric_limits<int>::min();
        int yMin = std::numeric_limits<int>::max();
        int yMax = std::numeric_limits<int>::min();

        for (const auto &hull_ele : hull)
        {
            for(const auto &pt : hull_ele)
            {
                xMin = xMin > pt.x ? pt.x : xMin;
                xMax = xMax < pt.x ? pt.x : xMax;
                yMin = yMin > pt.y ? pt.y : yMin;
                yMax = yMax < pt.y ? pt.y : yMax;
            }
        }

        roi->push_back(cv::Point(xMin, yMin));
        roi->push_back(cv::Point(xMin, yMax));
        roi->push_back(cv::Point(xMax, yMax));
        roi->push_back(cv::Point(xMax, yMin));

        contours.clear();
        hull.clear();
        hier.clear();
    }

    // save the roi_element
    for (const auto & roi_ele : roi_left)
    {
        cv::drawMarker(temp_left, roi_ele, cv::Scalar(255), 0, 40, 10);
    }

    for (const auto & roi_ele : roi_right)
    {
        cv::drawMarker(temp_right, roi_ele, cv::Scalar(255), 0, 40, 10);
    }

    // cv::imwrite("ROI_left.jpg", temp_left);
    // cv::imwrite("ROI_right.jpg", temp_right);

    // disp
    std::vector<double> disparity(roi_left.size());
    cv::Mat p(cv::Size(roi_left.size(), roi_left.size()), CV_64F);
    for (size_t i = 0; i < roi_left.size(); i++)
    {
        disparity[i] = roi_left[i].x - roi_right[i].x;
        p.at<double>(0, i) = (double)roi_left[i].x;
        p.at<double>(1, i) = (double)roi_left[i].y;
        p.at<double>(2, i) = (double)disparity[i];
        p.at<double>(3, i) = 1.0f;
    }

    cv::Mat roi = Q * p;

    for (size_t i = 0; i < roi_left.size(); i++)
    {
        roi.at<double>(0, i) = roi.at<double>(0, i) / roi.at<double>(3, i);
        roi.at<double>(1, i) = roi.at<double>(1, i) / roi.at<double>(3, i);
        roi.at<double>(2, i) = roi.at<double>(2, i) / roi.at<double>(3, i);
        roi.at<double>(3, i) = 1.f;
    }

    // end benchmark
    auto toc = ros::Time::now();
    auto dur = toc - tic;
    ROS_INFO_STREAM("Preprocessing took: " << dur.toSec());

    return roi;
}

cv::Mat
M1::compute_disparitymap(const cv::Mat & img_left, const cv::Mat & img_right, const cv::Mat & Q)
{   

    cv::FileStorage fs("stereosgbm_config.yaml", cv::FileStorage::READ);

    static auto min_disp          = 0;
    static auto num_disp          = 16*8;
    static auto block_size        = 3; // has to be odd
    static auto cn                = 3;
    static auto p1_smoothness     = 8*cn*block_size*block_size;
    static auto p2_smoothness     = 32*cn*block_size*block_size; // he larger the values are, the smoother the disparity is
    static auto disp_12_max       = 1;
    static auto unique_ratio      = 5;
    static auto speckle_win_size  = 0;
    static auto specke_range      = 1;
    static auto filter_gap        = 2;
    static auto written           = 0;
    static auto lambda            = 40;
    static auto sigma             = 10;

    // check if the file exists
    fs["written"] >> written;
    ROS_INFO_STREAM_ONCE("Checking if the file exists stereosgbm_config.yaml exists, the state is: " << written);
    ROS_INFO_STREAM_ONCE("If not, write to the file");

    if (fs.isOpened() && written == 1)
    {   
        //read from file
        fs["min_disp"]         >> min_disp;
        fs["num_disp"]         >> num_disp;
        fs["block_size"]       >> block_size;
        fs["cn"]               >> cn;
        fs["p1_smoothness"]    >> p1_smoothness;
        fs["p2_smoothness"]    >> p2_smoothness;
        fs["disp_12_max"]      >> disp_12_max;
        fs["unique_ratio"]     >> unique_ratio;
        fs["speckle_win_size"] >> speckle_win_size;
        fs["specke_range"]     >> specke_range;
        fs["filter_gap"]       >> filter_gap;
        fs["lambda"]           >> lambda;
        fs["sigma"]            >> sigma;
    }
    else
    {
        // write to file
        cv::FileStorage fs1("stereosgbm_config.yaml", cv::FileStorage::WRITE);
        fs1 << "written"          << 1;
        fs1 << "min_disp"         << min_disp;
        fs1 << "num_disp"         << num_disp;
        fs1 << "block_size"       << block_size;
        fs1 << "cn"               << cn;
        fs1 << "p1_smoothness"    << p1_smoothness;
        fs1 << "p2_smoothness"    << p2_smoothness;
        fs1 << "disp_12_max"      << disp_12_max;
        fs1 << "unique_ratio"     << unique_ratio;
        fs1 << "speckle_win_size" << speckle_win_size;
        fs1 << "specke_range"     << specke_range;
        fs1 << "filter_gap"       << filter_gap;
        fs1 << "sigma"            << sigma;
        fs1 << "lambda"           << lambda;
    }

    ROS_INFO_STREAM( min_disp           << ", " << 
                     num_disp           << ", " <<  
                     block_size         << ", " <<
                     cn                 << ", " << 
                     p1_smoothness      << ", " <<
                     p2_smoothness      << ", " <<
                     disp_12_max        << ", " <<
                     unique_ratio       << ", " <<
                     speckle_win_size   << ", " <<
                     specke_range       << ", " <<
                     filter_gap
                    );
    
    // setup the matchers
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

    // use the recomended solvers
    cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls = cv::ximgproc::createDisparityWLSFilter(left_matcher);
    cv::Ptr<cv::StereoMatcher> right_matcher = cv::ximgproc::createRightMatcher(left_matcher);

    // for plotting, use all 5, not optimal
    cv::Mat disp_left, disp_right, disp_filtered, img_left_temp, img_right_temp, point_cloud;

    cv::cvtColor(img_left,  img_left_temp, cv::COLOR_BGR2GRAY);
    cv::cvtColor(img_right, img_right_temp, cv::COLOR_BGR2GRAY);

    // cv::imwrite("disp_left.jpg", img_left_temp);
    // cv::imwrite("disp_right.jpg", img_right_temp);

    left_matcher->compute(img_left_temp, img_right_temp, disp_left);
    right_matcher->compute(img_right_temp, img_left_temp, disp_right);

    // cv::imshow("eleft_disp",disp_left);
    // cv::waitKey(0);
    // cv::imwrite("not_filtered.jpg", disp_left);

    wls->setLambda(lambda);
    wls->setSigmaColor(sigma);
    wls->filter(disp_left, img_left, disp_filtered, disp_right);

    // cv::imwrite("wls_filtered.jpg", disp_filtered);

    cv::ximgproc::getDisparityVis(disp_filtered, disp_filtered, 1.0);

    // cv::imwrite("disp_map.jpg", disp_filtered);

    cv::reprojectImageTo3D(disp_filtered, point_cloud, Q, true);

    return point_cloud;
};

void
M1::compute_pointcloud(const cv::Mat & point_cloud, const cv::Mat & left_img, const cv::Mat & ROI, const Eigen::Matrix4f & T )
{
    auto tic = ros::Time::now();

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    cloud_ptr->width = point_cloud.cols;
    cloud_ptr->height = point_cloud.rows;
    cloud_ptr->is_dense = false;
    cloud_ptr->resize(cloud_ptr->width * cloud_ptr->height);

    ROS_INFO_STREAM("The size of the point_cloud parsed: " << point_cloud.size());

    int k = 0;
    int m = 0;

    for (int i = 0; i < point_cloud.rows; ++i)
    {

        const float* point_cloud_ele = point_cloud.ptr<float>(i);
        const uchar* rbg_left = left_img.ptr<uchar>(i);
        m = 0;

        for(int j = 0; j < point_cloud.cols * 3; )
        {

            std::uint8_t b = (std::uint8_t) rbg_left[m++];
            std::uint8_t g = (std::uint8_t) rbg_left[m++];
            std::uint8_t r = (std::uint8_t) rbg_left[m++];

            if ( ( ( g > 225 or g < 210 ) and ( r > 225 or r < 210 ) ) and abs(point_cloud_ele[j+2]) < 500 )
            {
                cloud_ptr -> points[k].x = point_cloud_ele[j++];
                cloud_ptr -> points[k].y = point_cloud_ele[j++];
                cloud_ptr -> points[k].z = point_cloud_ele[j++];

                std::uint32_t rgb = ( (uint32_t) r << 24 | (uint32_t) g << 16 | (uint32_t) b << 8);
                
                cloud_ptr -> points[k++].rgb = *reinterpret_cast<float*>(&rgb);
            }
            else
                j += 3;
        }
    }

    // compute the transformation matrix

    pcl::transformPointCloud(*cloud_ptr, *cloud_ptr, T);

    // transform to the base frame.

    for (int i = 0; i < cloud_ptr->points.size(); i++)
    {
        cloud_ptr->points[i].x *= -1.;
    }


    // boxFilter();
    
    // statisticalFilter();

    // voxelFilter();

    // estimateNormalsScene();

    // computeFeaturesScene();

    pcl::io::savePCDFileASCII("test_pcd.pcd", *cloud_ptr);

    cloud_ptr->clear();

    auto toc = ros::Time::now();
    auto delta = toc-tic;
    ROS_INFO_STREAM("Time spent in preprocessing dense stereo scene: " << delta.toSec() << "ms");
}

}