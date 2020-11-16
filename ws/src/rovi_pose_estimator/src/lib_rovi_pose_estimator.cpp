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
		global_pose_est(const pcl::PointCloud<pcl::PointXYZ>::Ptr& scene, const pcl::PointCloud<pcl::PointXYZ>::Ptr& obj, pcl::PointCloud<pcl::PointXYZ>::Ptr& output, float inlier_tresh, int ransac_iterations)
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
			auto& pose_est = RANSAC_pose_est(scene, obj, match_indices, ransac_iterations, inlier_tresh);
			stop_local = std::chrono::high_resolution_clock::now();
			std::cout << " done: took " << std::chrono::duration_cast<std::chrono::seconds>(stop_local - start_local).count() << " Seconds" << std::endl;
			std::cout << "Applying transform to global object..." << std::endl;
			pcl::transformPointCloud(*obj, *output, pose_est);
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
    plane_segmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, pcl::PointIndices::Ptr& inlier_idices, pcl::ModelCoefficients::Ptr& plane_coeff )
	{
		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		// Optional
		seg.setOptimizeCoefficients (true);
		// Mandatory
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setDistanceThreshold (0.015);

		seg.setInputCloud(input_cloud);
		seg.segment (*inlier_idices, *plane_coeff);

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
		if(inverse_extraction) 	ROS_INFO("PointCloud representing any points BUT the plane: %i", output_cloud->width * output_cloud->height);
		else ROS_INFO("PointCloud representing the planar component: %i", output_cloud->width * output_cloud->height);
		
	}



	void 
	CheckforNans(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud)
	{
		ROS_INFO("PointCloud before nan check: %i ..., data points ( %s )", output_cloud->width * output_cloud->height, pcl::getFieldsList(*output_cloud).c_str());
		pcl::Indices tmp;
		pcl::removeNaNFromPointCloud(*input_cloud, *output_cloud, tmp);
		ROS_INFO("PointCloud after nan check: %i ..., data points ( %s )", output_cloud->width * output_cloud->height, pcl::getFieldsList(*output_cloud).c_str());
	}
	
}