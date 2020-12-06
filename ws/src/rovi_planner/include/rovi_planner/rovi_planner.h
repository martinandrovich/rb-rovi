#pragma once

#include <string>
#include <vector>
#include <array>
#include <tuple>
#include <thread>
#include <mutex>

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <kdl/trajectory_composite.hpp>
#include <pluginlib/class_loader.h>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/ModelStates.h>
#include <sensor_msgs/JointState.h>

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
	    ros::Publisher& pub_planning_scene,
	    int resolution = 16,
	    const std::string& obj_name = "bottle",
	    const std::array<double, 3>& table = { 0.4, 0.6, 0.64 }
	);

	class
	moveit_planner
	{
		public:

			moveit_planner() = delete;

			static bool
			init(ros::NodeHandle & nh);

			static void
			destruct();

			static void
			update_planning_scene();

			static bool
			attach_object_to_ee(const std::string& obj_name);

			static void
			start_planning_scene_publisher();

			static planning_interface::MotionPlanResponse
			plan(
				const geometry_msgs::Pose& pose_des,
				const std::string& planner = "RRTConnect",
				std::vector<double> q      = {}
			);

			static std::vector<sensor_msgs::JointState>
			get_traj(
				planning_interface::MotionPlanResponse& plan,
				double tol           = 0.0001, // rad
				double dt            = 0.001,  // s
				double max_vel_scale = 1.0,    // rad/s
				double max_acc_scale = 1.0     // rad/s^2
			);

		private:

			static inline bool is_init = false;
			static inline ros::NodeHandle* nh_ptr = nullptr;

			// moveit objects
			static inline robot_model::RobotModelPtr robot_model;
			static inline robot_model_loader::RobotModelLoaderPtr robot_model_loader;
			static inline boost::shared_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
			static inline planning_interface::PlannerManagerPtr planner_instance;

			// planning scence publisher
			static inline std::vector<moveit_msgs::CollisionObject> collision_objects;
			static inline planning_scene::PlanningScenePtr planning_scene;
			static inline std::mutex mtx_planning_scene;
			static inline ros::Publisher pub_planning_scene;
			static inline moveit_msgs::PlanningScene planning_scene_msg;
			static inline std::thread* thread_planning_scene_pub = nullptr;

			// trajectory publisher
			static inline ros::Publisher pub_traj;

			// configurations for moveit
			static inline constexpr auto ARM_GROUP            = "ur5_arm";
			static inline constexpr auto WSG_GROUP            = "wsg";
			static inline constexpr auto ROBOT_DESCRIPTION    = "robot_description";
			static inline constexpr auto PLANNER_PLUGIN_NAME  = "ompl_interface/OMPLPlanner";

			static inline constexpr auto PLANNING_SCENE_TOPIC = "/planning_scene_gazebo";
			static inline constexpr auto TRAJECTORY_TOPIC     = "/move_group/display_planned_path";

			static inline constexpr auto DEFAULT_PLANNER      = "RRTConnect";
			static inline constexpr auto AVAILABLE_PLANNERS   = { "RRT", "RRTConnect", "RRTstar" };
	};

} // namespace rovi_planner
