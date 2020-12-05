#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/topic.h>
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

#include <filesystem>





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
void pose_estimation_exampleM4(const std::string& ply_path, int pointsize);







int
main(int argc, char** argv)
{
	// define ROS node
	// https://yuzhangbit.github.io/tools/several-ways-of-writing-a-ros-node/

	ros::init(argc, argv, "example_node");
	ros::NodeHandle nh;

	pcl::PointCloud<pcl::PointXYZ>::Ptr model (new pcl::PointCloud<pcl::PointXYZ>);
	
	if(pcl::io::loadPLYFile<pcl::PointXYZ>("src/rovi_gazebo/models/milk/milk.ply", *model) == -1) // load the file
	{
		pcl::console::print_error ("Couldn't read file %s!\n");
	}
	else
	{
		std::cout << "Cloud size : " << model->height*model->width << std::endl;
	}

	const std::string window_name = "left_image";
	cv::namedWindow(window_name);


	ROS_INFO("Initialized a single-thread ROS example node.");
	std::cout << "CWD is: " << std::filesystem::current_path() << '\n';

	//pose_estimation_exampleM2();

	const auto msg = ros::topic::waitForMessage<sensor_msgs::Image>("/rbrovi/camera_stereo/left/image_raw");
	cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
	cv::imshow(window_name, img);
	cv::waitKey(0);

	std::vector<cv::Point2f> corner_points;
	rovi_pose_estimator::M4::Harris_corners_2d(img, corner_points, std::stof(argv[1]), std::stof(argv[2]), std::stoi(argv[3]), std::stof(argv[4]));

	std::vector<cv::Point2f> corner2d_matches;
	std::vector<cv::Point3f> corner3d_matches;
	for(auto& point: model->points)
	{
		std::cout << "Point is:" << point.getArray3fMap() << std::endl;
	}
	std::cout << "Size of model: " << model->width*model->height << std::endl;
	rovi_pose_estimator::M4::permute_point_matches(*model, corner_points, corner3d_matches, corner2d_matches);

	std::cout << "Size of corner2d_matches: " << corner2d_matches.size() << " , Corner3d_macthes: " << corner3d_matches.size() << std::endl;

	auto model_corner_points = rovi_pose_estimator::M4::PCL_pointcloud_to_OPENCV_Point3d(*model);
	cv::Mat pose_est;
	rovi_pose_estimator::M4::RANSAC_pose_estimation(model_corner_points, corner_points, corner3d_matches, corner2d_matches, pose_est, 50, 10.0f, &img);

	//pose_estimation_exampleM4(argv[1], std::stoi(argv[2]));


	ros::Rate loop_rate(1000);
	



	//while (ros::ok())
	//{
	//	const auto msg = ros::topic::waitForMessage<sensor_msgs::Image>("/rbrovi/camera_stereo/left/image_raw");
	//	// const auto msg = ros::topic::waitForMessage<sensor_msgs::Image>("/rbrovi/camera_stereo/left/image_raw", ros::Duration(1/30.0));
//
	//	if (msg)
	//	{
	//		ROS_INFO("Got new image!");
	//		const auto img = cv_bridge::toCvShare(msg, "bgr8")->image;
	//		cv::imshow("stereo_right", img);
	//		cv::waitKey(1);
	//	}
	//	loop_rate.sleep();
//
	//}
	
	//nh.subscribe("/rbrovi/camera_stereo/left/image_raw");
	
	// give full control over to ROS to handle callbacks etc.
	//ros::spin();
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



void pose_estimation_exampleM4(const std::string& ply_path, int point_size)
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

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr key_points (new pcl::PointCloud<pcl::PointXYZRGB>);
	//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(model);
	//pcl::visualization::PointCloudColorHandlerRGB<pcl::PointXYZRGB> rgb(point_cloud_ptr);

	rovi_pose_estimator::extract_indices(model, feature_idices, key_points, false);

	
	//pcl::visualization::PCLVisualizer visu("Model and feautures");
	pcl::visualization::PCLVisualizer::Ptr visu (new pcl::visualization::PCLVisualizer("Model and features"));
	pcl::visualization::PCLVisualizer::Ptr features (new pcl::visualization::PCLVisualizer("features"));


    visu->addPointCloud(model, "model");
    visu->addPointCloud(key_points, ColorHandlerT (key_points, 255.0, 255.0, 0.0), "features");
	visu->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "features");

	features->addPointCloud(key_points, ColorHandlerT (key_points, 255.0, 255.0, 0.0), "features");
	visu->spin();

	

}