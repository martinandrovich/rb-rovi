#include <ros/ros.h>
#include "moveit_utils/moveit_utils.h"

// moveit_msgs::CollisionObject
// moveit::make_mesh_cobj(const std::string& name, const std::array<double, 3>& pos, const std::array<double, 4>& ori)
// {
// 	constexpr std::string PATH_PACKAGE = "package://rovi_gazebo/models";
// 	ROS_WARN_STREAM_ONCE("moveit::make_mesh_cobj()only searches the ... path for models.");
	
// 	co.id = name;

// 	shapes::Mesh* m = shapes::createMeshFromResource("package://rovi_gazebo/models	/workpiece_wall.stl",b); 
// 	ROS_INFO("Workpiece Wall mesh loaded");

// 	shape_msgs::Mesh mesh;

// 	shapes::ShapeMsg mesh_msg;  
// 	shapes::constructMsgFromShape(m, mesh_msg);
// 	mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

// 	co.meshes.resize(1);
// 	co.mesh_poses.resize(1);

// 	co.meshes[0] = mesh;
// 	co.header.frame_id = "base_link";   

// 	co.mesh_poses[0].position.x = 0.560651;
// 	co.mesh_poses[0].position.y = 0.579113;
// 	co.mesh_poses[0].position.z = 0.0;
// 	co.mesh_poses[0].orientation.w= 0.0; 
// 	co.mesh_poses[0].orientation.x= 0.0; 
// 	co.mesh_poses[0].orientation.y= 0.0;
// 	co.mesh_poses[0].orientation.z= 0.0;   

// 	co.meshes.push_back(mesh);
// 	co.mesh_poses.push_back(co.mesh_poses[0]);
// 	co.operation = co.ADD;
// }