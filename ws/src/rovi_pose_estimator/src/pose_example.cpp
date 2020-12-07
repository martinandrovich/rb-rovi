#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv4/opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/topic.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <pcl/io/pcd_io.h>
#include <rovi_pose_estimator/rovi_pose_estimator.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <chrono>
#include <thread>
#include <pcl/io/ply_io.h>

#include <pcl/common/common.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <rovi_gazebo/rovi_gazebo.h>
#include <rovi_pose_estimator/rovi_pose_est.h>

#include <filesystem>
#include <eigen_conversions/eigen_msg.h>
#include <opencv4/opencv2/core/eigen.hpp>

void imageCallback(const sensor_msgs::ImageConstPtr& msg, const std::string& im_window)
{
  try
  {
    cv::imshow(im_window, cv_bridge::toCvShare(msg, "bgr8")->image );
  }
  
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}
void pose_estimation_exampleM2();
void pose_estimation_exampleM4(const std::string& ply_path, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& key_points, int pointsize, const bool & draw = false);


int
main(int argc, char** argv)
{
	// define ROS node
	// https://yuzhangbit.github.io/tools/several-ways-of-writing-a-ros-node/

	if ( argc != 5)
	{	
		// 	// 0.08 10.0 0 1.15
		ROS_INFO_STREAM("use this by argv := <quality> <min_dist> <harris:true/false> <filter_std>");
		return -1;
	}

	ros::init(argc, argv, "example_node");
	ros::NodeHandle nh;

	pcl::PointCloud<pcl::PointXYZ>::Ptr model (new pcl::PointCloud<pcl::PointXYZ>);

	auto model_path = ros::package::getPath("rovi_gazebo") + std::string("/models/milk/milk_color.ply");
	
	if(pcl::io::loadPLYFile<pcl::PointXYZ>(model_path, *model) == -1) // load the file
	{
		pcl::console::print_error ("Couldn't read file %s!\n");
	}
	else
	{
		ROS_INFO_STREAM_ONCE("Cloud size : " << model->height*model->width);
	}

	const std::string window_name = "left_image";
	cv::namedWindow(window_name);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr key_points (new pcl::PointCloud<pcl::PointXYZRGB>);
	pose_estimation_exampleM4(model_path, key_points, 0.5, true);

	ROS_INFO_STREAM_ONCE("Initialized a single-thread ROS example node.");
	ROS_INFO_STREAM_ONCE("CWD is: " << std::filesystem::current_path());

	//pose_estimation_exampleM2();

	const auto msg = ros::topic::waitForMessage<sensor_msgs::Image>("/rbrovi/camera/image_raw");
	cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
	cv::imshow(window_name, img);
	cv::waitKey(0);

	//

	std::vector<cv::Point2f> corner_points;
	rovi_pose_estimator::M4::Harris_corners_2d(img, corner_points, std::stof(argv[1]), std::stof(argv[2]), std::stoi(argv[3]), std::stof(argv[4]));

	// the corner matches

	std::vector<cv::Point2f> corner2d_matches;
	std::vector<cv::Point3f> corner3d_matches;

	for(auto& point: key_points->points)
	{
		ROS_INFO_STREAM("Point is:" << point.getArray3fMap());
	}

	ROS_INFO_STREAM("Size of model: " << key_points->width*key_points->height);

	rovi_pose_estimator::M4::permute_point_matches(*key_points, corner_points, corner3d_matches, corner2d_matches);

	ROS_INFO_STREAM("Size of corner2d_matches: " << corner2d_matches.size() << " , Corner3d_matches: " << corner3d_matches.size());

	auto model_corner_points = rovi_pose_estimator::M4::PCL_pointcloud_to_OPENCV_Point3d(*key_points);

	//
	
	cv::Mat pose_est;
	rovi_pose_estimator::M4::RANSAC_pose_estimation(model_corner_points, corner_points, corner3d_matches, corner2d_matches, pose_est, 10000, 5.001f, false, &img);
	auto model_pose = rovi_gazebo::get_model_pose("milk");

	//

	cv::Mat w_T_o;
    Eigen::Affine3d affine;
    tf::poseMsgToEigen(rovi_gazebo::get_model_pose("milk"), affine);
    cv::eigen2cv(affine.matrix(), w_T_o);

	ROS_INFO_STREAM_ONCE("Model pose is: " << model_pose);
	ROS_INFO_STREAM_ONCE("Pose est is: " << w_T_o);
	
}

void pose_estimation_exampleM2()
{

	typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ColorHandlerT;

	pcl::PointCloud<pcl::PointXYZ>::Ptr scene (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr obj (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr global_transformed (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr local_transformed (new pcl::PointCloud<pcl::PointXYZ>);

	rovi_pose_estimator::M2::get_point_cloud(scene, true);
	ROS_INFO("Scene loaded..");
	rovi_pose_estimator::M2::load_model("milk", obj);
	ROS_INFO("Model loaded..");

	rovi_pose_estimator::CheckforNans(obj, obj);

	pcl::visualization::PCLVisualizer visu("Scene and model");
    visu.addPointCloud (scene, ColorHandlerT (scene, 0.0, 255.0, 0.0), "scene");
    visu.addPointCloud (obj, ColorHandlerT (obj, 0.0, 0.0, 255.0), "object");
	visu.spin();

	rovi_pose_estimator::CheckforNans(scene, scene);
	visu.updatePointCloud(scene, "scene");
	visu.spin();

	rovi_pose_estimator::spatialFilter(scene, scene);
	visu.updatePointCloud(scene, "scene");
	visu.spin();

	float leaf_size = 0.015;

	rovi_pose_estimator::voxelGrid(scene, scene, leaf_size);
	visu.updatePointCloud(scene, "scene");
	rovi_pose_estimator::voxelGrid(obj, obj, leaf_size);
	visu.updatePointCloud(obj, "object");
	visu.spin();

	pcl::ModelCoefficients::Ptr plane_coeff (new pcl::ModelCoefficients);
 	pcl::PointIndices::Ptr plane_inliers (new pcl::PointIndices);

	rovi_pose_estimator::plane_segmentation(scene, plane_inliers, plane_coeff, leaf_size);

	// pcl::PointCloud<pcl::PointXYZ>::Ptr segmented_plane (new pcl::PointCloud<pcl::PointXYZ>);
	rovi_pose_estimator::extract_indices(scene, plane_inliers, scene, true);
	// visu.addPointCloud(segmented_plane, ColorHandlerT (obj, 255.0, 0.0, 0.0), "plane");
	visu.updatePointCloud(scene, "scene");
	ROS_INFO("Points after plane removal... %i", scene->height*scene->width);
	visu.spin();

	//rovi_pose_estimator::plane_segmentation(obj, plane_inliers, plane_coeff, leaf_size);
	//rovi_pose_estimator::extract_indices(obj, plane_inliers, obj, true);

	//visu.updatePointCloud(obj, "object");
	//ROS_INFO("Points for bottle after plane removal... %i", obj->height*obj->width);
	//visu.spin();


	//rovi_pose_estimator::outlierRemoval(scene, scene);
	//visu.updatePointCloud(scene, "scene");
	//visu.spin();


	//rovi_pose_estimator::smoothing(scene, scene);
	//visu.updatePointCloud(scene, "scene");
	//visu.spin();
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4 global_pose_est;
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4 local_pose_est;

	rovi_pose_estimator::M2::global_pose_est(scene, obj, global_transformed, global_pose_est, leaf_size, 5000);
	visu.addPointCloud(global_transformed, ColorHandlerT(global_transformed, 0.0, 0.0, 255.0), "object_aligned_global");
	std::cout << "Transformation matrix Global..\n" << global_pose_est << std::endl;
	visu.spin();
	std::cout << "Performing ICP to align better.." << std::endl;
	rovi_pose_estimator::ICP(scene, global_transformed, local_transformed, local_pose_est, leaf_size, 500);
	std::cout << "Transformation matrix local..\n" << local_pose_est << std::endl;
	visu.addPointCloud(local_transformed, ColorHandlerT (local_transformed, 255.0, 0.0, 0.0), "object_aligned_local");
	visu.spin();

	visu.removePointCloud("object_aligned_global");
	visu.spin();

	pcl::PCDWriter writer;

	writer.write<pcl::PointXYZ>("transformed.pcd", *local_transformed);
	ROS_INFO("Done performing global pose est...");
	
}



void pose_estimation_exampleM4(const std::string& ply_path, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& key_points, int point_size, const bool & draw)
{
	typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> ColorHandlerT;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr model (new pcl::PointCloud<pcl::PointXYZRGB>);
	
	if(!ply_path.empty())
	{
		if(pcl::io::loadPLYFile<pcl::PointXYZRGB>(ply_path, *model) == -1) // load the file
		{
			pcl::console::print_error ("Couldn't read file %s!\n", ply_path);
		}
		else
		{
			std::cout << "Cloud size : " << model->height*model->width << std::endl;
		}
	}

	
	pcl::TextureMesh mesh_milk;
	//pcl::io::loadOBJFile("milkobj.obj", mesh_milk);
	//pcl::io::loa ("milkobj.obj", mesh_milk);


	//pcl::TextureMesh mesh1;
	//pcl::io::loadPolygonFileOBJ ("milkobj.obj", mesh1);
//
	//pcl::TextureMesh mesh2;
	//pcl::io::loadOBJFile("milkobj.obj", mesh2);
//
	//mesh1.tex_materials = mesh2.tex_materials;c
	//pcl::visualization::PCLVisualizer visu_mesh("Test");
	//visu_mesh.addTextureMesh (mesh1,"texture");
	//visu_mesh.spin();


	//std::cout << "Size of mesh.." << mesh2.cloud.width* mesh2.cloud.height << std::endl;

	//pcl::visualization::PCLVisualizer visuMesh("Milk");
	//visuMesh.addTextureMesh(mesh_milk,  "milk_mesh");
	//visuMesh.spin();

	pcl::PointIndices::Ptr feature_idices (new pcl::PointIndices);
	rovi_pose_estimator::M4::Harris_keypoints_example(model, feature_idices);
	std::cout << "Done performing Harris_keypoints_example... " << std::endl;

	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr key_points (new pcl::PointCloud<pcl::PointXYZRGB>);
	//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(model);
	//pcl::visualization::PointCloudColorHandlerRGB<pcl::PointXYZRGB> rgb(point_cloud_ptr);

	rovi_pose_estimator::extract_indices(model, feature_idices, key_points, false);

	if(draw)
	{
		pcl::visualization::PCLVisualizer::Ptr visu (new pcl::visualization::PCLVisualizer("Model and features"));
		pcl::visualization::PCLVisualizer::Ptr features (new pcl::visualization::PCLVisualizer("features"));
		visu->addPointCloud(model, "model");
		visu->addPointCloud(key_points, ColorHandlerT (key_points, 255.0, 255.0, 0.0), "features");
		visu->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "features");
		features->addPointCloud(key_points, ColorHandlerT (key_points, 255.0, 255.0, 0.0), "features");
		visu->spin();
	}
}