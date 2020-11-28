#pragma once

#include <vector>
#include <array>
#include <tuple>
#include <thread>

#include <Eigen/Eigen>

#include <ros/ros.h>

#include <geometry_msgs/Pose.h>

#include <sensor_msgs/JointState.h>
 
#include <kdl/trajectory_composite.hpp>

#include <pluginlib/class_loader.h>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <gazebo_msgs/ModelStates.h>
#include <sensor_msgs/JointState.h>

#include <ur5_dynamics/ur5_dynamics.h>
#include <rovi_utils/rovi_utils.h>

namespace rovi_planner
{
	// trajectory planners

	KDL::Trajectory_Composite
	traj_linear(
	    const std::vector<geometry_msgs::Pose>& waypoints,
	    double vel_max      = 1.00, // [m/s]
	    double acc_max      = 1.00, // [m/s^2]
	    double equiv_radius = 0.05  // [m]
	);
	
	std::array<KDL::Trajectory_Composite*, 6>
	traj_linear(
	    const std::vector<sensor_msgs::JointState>& joint_states,
	    double vel_max      = 1.00, // [m/s]
	    double acc_max      = 1.00, // [m/s^2]
	    double equiv_radius = 0.05  // [m]
	);

	KDL::Trajectory_Composite
	traj_parabolic(
	    const std::vector<geometry_msgs::Pose>& waypoints,
	    double vel_max       = 1.00, // [m/s]
	    double acc_max       = 1.00, // [m/s^2]
	    double corner_radius = 0.05, // [m]
	    double equiv_radius  = 0.05  // [m]
	);

	// reachability planner
	std::array<KDL::Trajectory_Composite*, 6>
	traj_parabolic(
	    const std::vector<sensor_msgs::JointState>& joint_states,
	    double vel_max       = 1.00, // [m/s]
	    double acc_max       = 1.00, // [m/s^2]
	    double corner_radius = 0.05, // [m]
	    double equiv_radius  = 0.05  // [m]
    );
	
	// reachability ananlysis

	void 
	reachability(
	    const std::vector<std::array<double, 3>>& base_pts,
	    const std::array<double, 3>& obj,
	    const std::array<double, 3>& offset,
	    const std::array<double, 3>& axis,
	    ros::Publisher& planning_scene_pub,
	    int resolution = 16,
	    const std::string& obj_name = "bottle",
	    const std::array<double, 3>& table = { 0.4, 0.6, 0.64 }
	);
	
	class moveit_planner
	{
		public:
			moveit_planner() = delete;

			static bool init(ros::NodeHandle & nh);

			static planning_interface::MotionPlanResponse
			traj_moveit(
				const geometry_msgs::Pose& pose_des,
				const std::string& planner = "RRTConnect",
				std::vector<double> q      = {}
			);

			static void 
			start_async_publisher(
				const std::string & node, 
				const double & dur
			);

		private:

			// less common moveit objects
			static inline bool is_init = false;
			static inline std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
			static inline planning_interface::PlannerManagerPtr planner_instance;

			// common moveit objects
			static inline robot_model_loader::RobotModelLoaderPtr robot_model_loader;
			static inline robot_model::RobotModelPtr robot_model;
			static inline planning_scene::PlanningScenePtr planning_scene;

			// topics
			static inline ros::Publisher planning_scene_pub;
			static inline ros::Publisher disp_traj_pub;

			// configurations for moveit
			static inline constexpr auto ARM_GROUP           	= "ur5_arm";
			static inline constexpr auto WSG_GROUP           	= "wsg";
			static inline constexpr auto ROBOT_DESCRIPTION   	= "robot_description";
			static inline constexpr auto PLANNER_PLUGIN_NAME 	= "ompl_interface/OMPLPlanner";

			static inline constexpr auto PLANNING_SCENE_TOPIC 	= "/planning_scene_gazebo";
			static inline constexpr auto TRAJECTORY_TOPIC     	= "/move_group/display_planned_path";

			static inline constexpr auto DEFAULT_PLANNER     	= "RRTConnect";
			static inline constexpr auto AVAILABLE_PLANNERS  	= { "RRT", "RRTConnect", "RRTstar" };

			static inline constexpr auto DEFAULT_PUB			= "planning_scene";
			static inline constexpr auto AVAILABLE_PUBS  		= { "planning_scene", "disp_trajectory" };
	};

} // namespace rovi_planner
