#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "pcl_ros/point_cloud.h"

#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/convolution_3d.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/spin_image.h>
#include <pcl/common/random.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/registration/icp.h>
#include <pcl/keypoints/harris_3d.h>

#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/tracking.hpp>
#include <opencv2/core.hpp>


#include "rovi_pose_estimator/rovi_pose_estimator.h"

void
rovi_pose_estimator::test(const std::string& str)
{
	ROS_INFO_STREAM("hello: " << str);
}


namespace rovi_pose_estimator
{
	const std::string models_path = "src/rovi_gazebo/models/";

	namespace M2
	{
		void
        get_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud, bool save_to_disk)
		{
			const auto point_cloud = ros::topic::waitForMessage<pcl::PointCloud<pcl::PointXYZ>>("/rbrovi/kinect_sensor_ir/rbrovi/kinect_sensor/depth/points");

			if(save_to_disk)
			{
			pcl::PCDWriter writer;
  			writer.write<pcl::PointXYZ> ("kinect_cloud.pcd", *point_cloud, false);
			}
			
			*output_cloud = *point_cloud;
		}

		void compute_spin_image_features(const pcl::PointCloud<pcl::PointXYZ>::Ptr& scene, const pcl::PointCloud<pcl::PointXYZ>::Ptr& obj, const pcl::PointCloud<pcl::Histogram<153>>::Ptr& scene_features, const pcl::PointCloud<pcl::Histogram<153>>::Ptr& obj_features)
		{
			pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree (new pcl::search::KdTree<pcl::PointXYZ>);
			pcl::SpinImageEstimation<pcl::PointXYZ, pcl::Normal, pcl::Histogram<153> > spin_image_descriptor;

			pcl::PointCloud<pcl::Normal>::Ptr scene_normals (new pcl::PointCloud<pcl::Normal>);
			pcl::PointCloud<pcl::Normal>::Ptr obj_normals (new pcl::PointCloud<pcl::Normal>);

			ROS_INFO("Calculating normals for scene...");
			compute_normals(scene, scene_normals);
			ROS_INFO("Calculating normals for obj...");
			compute_normals(obj, obj_normals);

			spin_image_descriptor.setSearchMethod(kd_tree);
			spin_image_descriptor.setRadiusSearch(0.05);
			
			kd_tree->setInputCloud(scene);
			spin_image_descriptor.setInputCloud(scene);
			spin_image_descriptor.setInputNormals(scene_normals);
			ROS_INFO("Calculating features for scene...");
			spin_image_descriptor.compute(*scene_features);

			kd_tree->setInputCloud(obj);
			spin_image_descriptor.setInputCloud(obj);
			spin_image_descriptor.setInputNormals(obj_normals);
			ROS_INFO("Calculating features for scene...");
			spin_image_descriptor.compute(*obj_features);
		}

		void match_spin_image_features(const pcl::PointCloud<pcl::Histogram<153>>::Ptr& scene, const pcl::PointCloud<pcl::Histogram<153>>::Ptr& obj, std::vector<int>& scene_match_indices)
		{   
			float L2_distance_best = 0.0;
			float SSE = 0.0;
			int best_match_idx = 0;
			float hist_difference[153];

			for(auto& obj_descriptor: obj->points)
			{
				int scene_match_idx = 0;
				L2_distance_best = MAXFLOAT;
				for(auto& scene_descriptor: scene->points)
				{
					SSE = 0.0;
					for(int i = 0; i < 153; i++)
					{
						SSE += std::pow((obj_descriptor.histogram[i] - scene_descriptor.histogram[i]), 2);
					}
					
					if(SSE < L2_distance_best)
					{
						best_match_idx = scene_match_idx;
						L2_distance_best = SSE;
					}
					scene_match_idx++ ;
				}
				//std::cout << "Best L2_distance found was: " << L2_distance_best << ", at idx: " << best_match_idx << std::endl;
				scene_match_indices.push_back(best_match_idx);
			}
		}


		const pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4 
		RANSAC_pose_est(const pcl::PointCloud<pcl::PointXYZ>::Ptr& scene, const pcl::PointCloud<pcl::PointXYZ>::Ptr& obj, std::vector<int>& scene_match_indices, int max_iterations, double inlier_threshold)
		{
			pcl::common::UniformGenerator<int> gen(0, scene_match_indices.size()-1);
			std::vector<int> obj_match_idices_sample(3);
			std::vector<int> scene_match_idices_sample(3);
			pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ, float> svd;
			pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4 rot;
			pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4 best_pose_est;


			pcl::PointCloud<pcl::PointXYZ> obj_cp;
			pcl::copyPointCloud(*obj, obj_cp);

			pcl::search::KdTree<pcl::PointXYZ>::Ptr scene_tree (new pcl::search::KdTree<pcl::PointXYZ>);
			scene_tree->setInputCloud(scene);

			int inliers = 0;    //prepare for inlier validation
			int highest_number_of_inliers = 0;
			std::vector<int> inlier_idices;
			std::vector<float> inlier_distances;

			for(int iteration = 0; iteration < max_iterations; iteration++)
			{
				for(int sample= 0; sample < 3; sample++)
				{
					int X = gen.run();
					obj_match_idices_sample[sample] = X;
					scene_match_idices_sample[sample] = scene_match_indices[X];
				}
				svd.estimateRigidTransformation(*obj, obj_match_idices_sample, *scene, scene_match_idices_sample, rot);
				pcl::transformPointCloud(*obj, obj_cp, rot);

				// Validate the transformation, counting inliers within 5 mm in scene for each point.
				inliers = 0;
				for(auto& point: obj_cp.points)
				{
					inliers += scene_tree->radiusSearch(point, inlier_threshold, inlier_idices, inlier_distances);
				}
				if(inliers > highest_number_of_inliers)
				{
					highest_number_of_inliers = inliers;
					best_pose_est = rot;
				}
				if(iteration % 100 == 0)
				{
					std::cout << iteration << " Iterations has passed.. Best match so far: " << highest_number_of_inliers << " number of inĺiers" << std::endl;
				}
			}
			return best_pose_est;
		}

		void 
		global_pose_est(const pcl::PointCloud<pcl::PointXYZ>::Ptr& scene, const pcl::PointCloud<pcl::PointXYZ>::Ptr& obj, pcl::PointCloud<pcl::PointXYZ>::Ptr& output, pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4& transformation, float inlier_tresh, int ransac_iterations)
		{

			pcl::PointCloud<pcl::Histogram<153>>::Ptr scene_spin_images (new pcl::PointCloud<pcl::Histogram<153>>);
			pcl::PointCloud<pcl::Histogram<153>>::Ptr obj_spin_images (new pcl::PointCloud<pcl::Histogram<153>>);
			auto start_total = std::chrono::high_resolution_clock::now();
			
			std::cout << "Computing spin image features: ..." << std::endl;
			auto start_local = std::chrono::high_resolution_clock::now();
			compute_spin_image_features(scene, obj, scene_spin_images, obj_spin_images);
			auto stop_local = std::chrono::high_resolution_clock::now();
			std::cout << " done: took " << std::chrono::duration_cast<std::chrono::seconds>(stop_local - start_local).count() << " Seconds" << std::endl;

			std::vector<int> match_indices;
			std::cout << "Finding spin image feature matches: ..." << std::endl;
			start_local = std::chrono::high_resolution_clock::now();
			match_spin_image_features(scene_spin_images, obj_spin_images, match_indices);
			stop_local = std::chrono::high_resolution_clock::now();
			std::cout << " done: took " << std::chrono::duration_cast<std::chrono::seconds>(stop_local - start_local).count() << " Seconds" << std::endl;

			std::cout << "Using ransac to find pose estimate: ..." << std::endl;
			start_local = std::chrono::high_resolution_clock::now();
			transformation = RANSAC_pose_est(scene, obj, match_indices, ransac_iterations, inlier_tresh);
			stop_local = std::chrono::high_resolution_clock::now();
			std::cout << " done: took " << std::chrono::duration_cast<std::chrono::seconds>(stop_local - start_local).count() << " Seconds" << std::endl;
			std::cout << "Applying transform to global object..." << std::endl;
			pcl::transformPointCloud(*obj, *output, transformation);
			std::cout << "Whole process took: " << std::chrono::duration_cast<std::chrono::seconds>(stop_local- start_total).count() << " Seconds" << std::endl;

		}
				

		void 
		load_model(const std::string& model, const pcl::PointCloud<pcl::PointXYZ>::Ptr& dst)
		{
			const std::string full_path = models_path + model +"/" + model + ".pcd";
			std::cout << full_path << std::endl;
			if(pcl::io::loadPCDFile<pcl::PointXYZ> (full_path, *dst) == -1)
			{
				ROS_ERROR("Could not read the model file...");
			}
		}

	}

	void compute_normals(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input, const pcl::PointCloud<pcl::Normal>::Ptr& normals)
	{
		pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree (new pcl::search::KdTree<pcl::PointXYZ>);
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_est;
		
		normal_est.setSearchMethod(kd_tree);
		kd_tree->setInputCloud(input);
		normal_est.setKSearch(10);
		normal_est.setInputCloud(input);

		normal_est.compute(*normals);
	}

	void 
	voxelGrid(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud, float leaf_size)
	{
		ROS_INFO("PointCloud before voxelgrid filtering: %i ...", input_cloud->width * input_cloud->height);
		pcl::VoxelGrid<pcl::PointXYZ> filter;
		
		filter.setInputCloud(input_cloud);
		filter.setLeafSize(leaf_size, leaf_size, leaf_size);
		filter.filter(*output_cloud);
		ROS_INFO("PointCloud after voxelgrid filtering: %i ..., data points ( %s )", output_cloud->width * output_cloud->height, pcl::getFieldsList(*output_cloud).c_str());
		pcl::io::savePCDFile ("Voxel_grid.pcd", *output_cloud);
	}

	void 
	outlierRemoval(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud)
	{
		std::cerr << "PointCloud before statistical_outlier filtering: " << input_cloud->width * input_cloud->height 
		<< " data points (" << pcl::getFieldsList (*input_cloud) << ")." << std::endl;

		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statistical_filter;
		statistical_filter.setInputCloud(input_cloud);
		statistical_filter.setMeanK(100);
		statistical_filter.setStddevMulThresh(1);
		statistical_filter.filter(*output_cloud);
		pcl::io::savePCDFile ("Statistical_outlier.pcd", *output_cloud);

		std::cerr << "PointCloud after statistical_outlier filtering: " << output_cloud->width * output_cloud->height 
		<< " data points (" << pcl::getFieldsList (*output_cloud) << ")." << std::endl;
	}

	void 
	spatialFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud )
	{
		std::cerr << "PointCloud before Spatial filtering: " << input_cloud->width * input_cloud->height 
		<< " data points (" << pcl::getFieldsList (*input_cloud) << ")." << std::endl;
		pcl::PointXYZ minPt, maxPt;
		pcl::getMinMax3D (*input_cloud, minPt, maxPt);
  		std::cout << "Max x: " << maxPt.x << std::endl;
  		std::cout << "Min x: " << minPt.x << std::endl;
		std::cout << "Max y: " << maxPt.y << std::endl;
  		std::cout << "Min y: " << minPt.y << std::endl;
		std::cout << "Max z: " << maxPt.z << std::endl;
  		std::cout << "Min z: " << minPt.z << std::endl << std::endl;

		pcl::PassThrough<pcl::PointXYZ> pass_filter;
		pass_filter.setInputCloud(input_cloud);
		pass_filter.setFilterFieldName("z");
		pass_filter.setFilterLimits(0.0, 1.17);
		pass_filter.filter(*output_cloud);
		pass_filter.setInputCloud(output_cloud);

		pass_filter.setFilterFieldName("y");
		pass_filter.setFilterLimits(-0.84, -0.10);
		pass_filter.filter(*output_cloud);

		pass_filter.setFilterFieldName("x");
		pass_filter.setFilterLimits(-0.5, 0.5);
		pass_filter.filter(*output_cloud);
		pcl::io::savePCDFile ("spatial_filter.pcd", *output_cloud);

		pcl::getMinMax3D (*input_cloud, minPt, maxPt);
  		std::cout << "Max x: " << maxPt.x << std::endl;
  		std::cout << "Min x: " << minPt.x << std::endl;
		std::cout << "Max y: " << maxPt.y << std::endl;
  		std::cout << "Min y: " << minPt.y << std::endl;
		std::cout << "Max z: " << maxPt.z << std::endl;
  		std::cout << "Min z: " << minPt.z << std::endl;


		std::cerr << "PointCloud after Spatial filtering: " << output_cloud->width * output_cloud->height 
		<< " data points (" << pcl::getFieldsList (*output_cloud) << ")." << std::endl;
	}

	void smoothing(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud )
	{
		std::cerr << "PointCloud before moving least squares filtering: " << input_cloud->width * input_cloud->height 
		<< " data points (" << pcl::getFieldsList (*input_cloud) << ")." << std::endl;

		pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZINormal> mls;
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZINormal> mls_points;
		mls.setComputeNormals(false);
		mls.setInputCloud(input_cloud);
		mls.setPolynomialOrder(2);
		mls.setSearchMethod(tree);
		mls.setSearchRadius(3);
		mls.process(mls_points);
		pcl::copyPointCloud(mls_points, *output_cloud);
		pcl::io::savePCDFile ("moving_least_squares.pcd", mls_points);

		std::cerr << "PointCloud after moving least squares filtering: " << output_cloud->width * output_cloud->height 
		<< " data points (" << pcl::getFieldsList (*output_cloud) << ")." << std::endl;
	}

	// Example found at https://pointclouds.org/documentation/tutorials/planar_segmentation.html
	void
    plane_segmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, pcl::PointIndices::Ptr& inlier_idices, pcl::ModelCoefficients::Ptr& plane_coeff, float leaf_size)
	{
		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		// Optional
		seg.setOptimizeCoefficients (true);
		// Mandatory
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setDistanceThreshold(leaf_size/2.0);

		seg.setInputCloud(input_cloud);
		seg.segment(*inlier_idices, *plane_coeff);

		if (inlier_idices->indices.size () == 0)
		{
			ROS_ERROR("Could not estimate a planar model for the given dataset");
		}

	}

	// Example found at https://pointclouds.org/documentation/tutorials/extract_indices.html
    void
    extract_indices(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, pcl::PointIndices::Ptr& inlier_idices, pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud, bool inverse_extraction)
	{
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		// Extract the inliers
		extract.setInputCloud (input_cloud);
		extract.setIndices(inlier_idices);
		extract.setNegative(inverse_extraction);
		extract.filter(*output_cloud);
		if(inverse_extraction) 	ROS_INFO("PointCloud representing any points BUT the indices: %i", output_cloud->width * output_cloud->height);
		else ROS_INFO("PointCloud representing extracted indices %i", output_cloud->width * output_cloud->height);
		
	}
	// added for RGB pointCloud...
	void
    extract_indices(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud, pcl::PointIndices::Ptr& inlier_idices, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud, bool inverse_extraction)
	{
		pcl::ExtractIndices<pcl::PointXYZRGB> extract;
		// Extract the inliers
		extract.setInputCloud (input_cloud);
		extract.setIndices(inlier_idices);
		extract.setNegative(inverse_extraction);
		extract.filter(*output_cloud);
		if(inverse_extraction) 	ROS_INFO("PointCloud representing any points BUT the indices: %i", output_cloud->width * output_cloud->height);
		else ROS_INFO("PointCloud representing extracted indices %i", output_cloud->width * output_cloud->height);
	}

	void 
	CheckforNans(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud)
	{
		ROS_INFO("PointCloud before nan check: %i ..., data points ( %s )", output_cloud->width * output_cloud->height, pcl::getFieldsList(*output_cloud).c_str());
		pcl::Indices tmp;
		pcl::removeNaNFromPointCloud(*input_cloud, *output_cloud, tmp);
		ROS_INFO("PointCloud after nan check: %i ..., data points ( %s )", output_cloud->width * output_cloud->height, pcl::getFieldsList(*output_cloud).c_str());
	}

	void
    ICP(const pcl::PointCloud<pcl::PointXYZ>::Ptr& scene, const pcl::PointCloud<pcl::PointXYZ>::Ptr& model, pcl::PointCloud<pcl::PointXYZ>::Ptr& transformed_model, pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4& final_trans, float leaf_size,  int max_iterations)
	{
		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  		icp.setMaximumIterations(max_iterations);
  		icp.setInputSource(model);
  		icp.setInputTarget(scene);
		icp.setMaxCorrespondenceDistance(leaf_size);
  		icp.align(*transformed_model);
		if (icp.hasConverged())
		{
		    std::cout << "ICP has converged, score is " << icp.getFitnessScore(leaf_size) <<"" << std::endl;
			final_trans = icp.getFinalTransformation();
		}
	}

	void
    get_final_pose(const pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4& global_rot, const pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4& local_rot, const pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4& sensor_pose, pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4& final_pose)
	{
		final_pose = sensor_pose*global_rot*local_rot;
	}
	
	namespace M4
	{
		//Example taken from https://github.com/PointCloudLibrary/pcl/blob/master/examples/keypoints/example_get_keypoints_indices.cpp
		void Harris_keypoints_example(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& model, pcl::PointIndices::Ptr& inlier_idices)
		{
			pcl::HarrisKeypoint3D <pcl::PointXYZRGB, pcl::PointXYZI> detector;
			pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints (new pcl::PointCloud<pcl::PointXYZI>);
			
			detector.setNonMaxSupression(true);
			detector.setInputCloud(model);
			detector.setThreshold(1e-6);
			pcl::StopWatch watch;
			detector.compute(*keypoints);
			pcl::console::print_highlight("Detected %zd points in %lfs\n", keypoints->size (), watch.getTimeSeconds ());
			*inlier_idices  = *detector.getKeypointsIndices ();
			if (inlier_idices->indices.empty ())
			{
				pcl::console::print_warn ("Keypoints indices are empty!\n");
			}
		}

		struct Harris_settings
		{
			int block_size =3;
			int aperture_size = 3;
			int k = 4;
			int norm_tresh=200;
			cv::Mat* src;
			cv::Mat* out;
			std::vector<cv::Point2i>* corner_coordinates;
		};
		

		void 
		cornerHarris_demo( int, void* args );

		cv::Scalar_<float> 
		get_mean_and_std(const std::vector<cv::Point>& points);

		std::vector<cv::Point>
		statistical_filter_2d(const std::vector<cv::Point>& points, float sigma_tresh=1.0);


		void
		Harris_corners_2d(const cv::Mat& img, std::vector<cv::Point2d>& corner_points, float quality_level, float min_dist, bool useharris, float filter_sigma)
		{
			cv::Mat img_gray;
			cv::cvtColor(img, img_gray,cv::COLOR_BGR2GRAY);

			const std::string& src = "src window";
			cv::namedWindow(src);

			//auto roi = cv::selectROI(src, img_gray);
			//std::cout << "Roi selected.."<< roi << std::endl;
			
			/*201 x 747 from (283, 39)*/
			auto roi = cv::Rect(283, 39, 198, 747);
			cv::Mat gray_roi = img_gray(roi);
			std::cout << "Type?" << gray_roi.type() << std::endl;

			cv::Mat corners = cv::Mat::zeros(gray_roi.size(), CV_32FC1 );
			std::vector<cv::Point2i> corner_coordinates;


			//Harris_settings settings = {1, 1, 1, 180, &gray_roi, &corners, &corner_coordinates};
			//cv::namedWindow("corners_window");
//
			//cv::createTrackbar( "Aperture: ", src, &settings.aperture_size, 10, cornerHarris_demo, &settings);
			//cv::createTrackbar( "block_size: ", src, &settings.block_size, 10, cornerHarris_demo, &settings);
			//cv::createTrackbar( "k: ", src, &settings.k, 15, cornerHarris_demo, &settings);
			//cv::createTrackbar( "norm_tresh: ", src, &settings.norm_tresh, 180, cornerHarris_demo, &settings);
			//cv::imshow(src, gray_roi);
//
			////cornerHarris_demo(0, &settings);
			//cv::waitKey();
//
			//for(auto& coord:corner_coordinates)
			//{
			//	std::cout << "Corner found at: " << coord << " In image_coordinates" << std::endl;
			//}
//
			cv::Mat corners_good_features_mat;
			gray_roi.copyTo(corners_good_features_mat);


			std::vector<cv::Point> corners_good_features;
			cv::goodFeaturesToTrack(gray_roi, corners_good_features, 0, quality_level, min_dist, cv::noArray(), 3 , useharris);

			std::cout << " Done using good features to track... " << std::endl;

			for(auto& coord:corners_good_features)
			{
				std::cout << "Corner found at: " << coord << " In image_coordinates" << std::endl;
				circle(corners_good_features_mat, coord, 10,  cv::Scalar(0), 2, 8, 0 );

			}

			cv::imshow("Good features to track", corners_good_features_mat);
			cv::waitKey();

			auto mean_std = get_mean_and_std(corners_good_features);
			std::cout << "Mean_std is:" << mean_std << std::endl;

			auto filtered = statistical_filter_2d(corners_good_features, filter_sigma);

			for(const auto& point: filtered)
			{
				corner_points.emplace_back(point);
			} 

			cv::Mat corners_good_features_mat_statistical_filtered;
			gray_roi.copyTo(corners_good_features_mat_statistical_filtered);

			for(auto& coord:corner_points)
			{
				std::cout << "Corner found at: " << coord << " In image_coordinates" << std::endl;
				circle(corners_good_features_mat_statistical_filtered, coord, 10,  cv::Scalar(0), 2, 8, 0 );
			}

			cv::imshow("Filtered corners...", corners_good_features_mat_statistical_filtered);
			cv::waitKey();
		}

		

		cv::Scalar_<float> 
		get_mean_and_std(const std::vector<cv::Point>& points)
		{
	
			cv::Point2f zero(0.0f, 0.0f);
			cv::Point2f sum = std::accumulate(points.begin(), points.end(), zero, std::plus<cv::Point2f>());
			cv::Point2f mean = {sum.x / points.size(), sum.y / points.size()};

			auto calc_std = [&](auto& points, cv::Point2f& mean)
			{
				cv::Point2f std = {0,0};
				for(const auto& point:points)
				{
					std.x += std::pow((point.x - mean.x), 2);
					std.y += std::pow((point.y - mean.y), 2);
				}
				std.x = std.x/ points.size() -1;
				std.y = std.y/ points.size() -1;
				std.x = sqrt(std.x);
				std.y = sqrt(std.y);
				return std;
			};

			auto std = calc_std(points, mean);
			return {mean.x, mean.y, std.x, std.y};
		}

		std::vector<cv::Point>
		statistical_filter_2d(const std::vector<cv::Point>& points, float sigma_tresh)
		{
			auto mean_std = get_mean_and_std(points);
			cv::Point2f threshold_lower = {mean_std[0] - sigma_tresh * mean_std[2], mean_std[1] - sigma_tresh* mean_std[3]};
			cv::Point2f threshold_upper = {mean_std[0] + sigma_tresh * mean_std[2], mean_std[1] + sigma_tresh* mean_std[3]};
			
			std::vector<cv::Point> filtered;
			for(const auto& point:points)
			{
				if(point.x > threshold_lower.x && point.x < threshold_upper.x)
				{
					if(point.y > threshold_lower.y && point.y < threshold_upper.y)
					{
						filtered.push_back(point);
					}
				}				
			}
			return filtered;
		}
	

		void permute_point_matches(const pcl::PointCloud<pcl::PointXYZ>& model_corners, const std::vector<cv::Point2d>& image_corners, std::vector<cv::Point3d>& model_matches, std::vector<cv::Point2d>& image_matches)
		{
			for(const auto& point2d: image_corners)
			{
				for(const auto& point3d: model_corners)
				{
					image_matches.push_back(point2d);
					model_matches.emplace_back(point3d.x, point3d.y, point3d.z);
					//std::cout << "Found 3d point: " << point3d << " , " << point3d.data <<  std::endl; 
				}
			}
		}




















































		void 
		cornerHarris_demo( int, void* args )
		{
			std::cout << "Hello, inside corner demo..:" << std::endl;
			Harris_settings* settings = reinterpret_cast<Harris_settings*>(args);
			int blockSize = settings->block_size + 1;
			int apertureSize = settings->aperture_size *2 + 1;
			double k = settings->k/100.0;

			std::cout << "Settings are; " << blockSize << " , " << apertureSize << " , " << k << std::endl;

			cv::Mat& corners = *settings->out;
			cv::cornerHarris(*settings->src, corners, blockSize, apertureSize, k );
			cv::Mat dst_norm, dst_norm_scaled;
			cv::normalize(corners, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat() );
			convertScaleAbs( dst_norm, dst_norm_scaled );
			settings->corner_coordinates->clear();

			int number_of_found_corners = 0;
			for( int i = 0; i < dst_norm.rows ; i++ )
			{
				for( int j = 0; j < dst_norm.cols; j++ )
				{
					if( (int) dst_norm.at<float>(i,j) > settings->norm_tresh )
					{
						circle( dst_norm_scaled, cv::Point(j,i), 5,  cv::Scalar(0), 2, 8, 0 );
						settings->corner_coordinates->push_back(cv::Point(j,i));
						number_of_found_corners++;
					}
				}
			}
			std::cout << "Found " << number_of_found_corners << " Corners in the image." << std::endl;
			cv::imshow("corners_window", dst_norm_scaled);
		}

	}





}