#pragma once

#include <string>
#include <vector>

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/ros.h>
#include <ur5_dynamics/ur5_dynamics.h>
#include <urdf/model.h>

#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Core>
#include <Eigen/Geometry>


namespace ur5_controllers
{
	class JointPositionController final: public controller_interface::Controller<hardware_interface::EffortJointInterface>
	{

	public:
		static inline constexpr auto CONTROLLER_NAME = "JointPositionController";
		static inline constexpr auto SATURATE_ROTATUM = true;
		static inline constexpr auto TAU_DOT_MAX = 1000.;
		static inline const std::vector<double> Q_D_INIT = { 1.57, -1.57, 1.57, 1.57, 1.57, 0.0 };

		std::vector<std::string> vec_joint_names;
		size_t num_joints;

		std::vector<hardware_interface::JointHandle> vec_joints;
		realtime_tools::RealtimeBuffer<sensor_msgs::JointState> commands_buffer;

		//Default Constructor
		JointPositionController() {}
		~JointPositionController() { sub_command.shutdown(); }

		bool
		init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh) override;

		void
		starting(const ros::Time& time) override;

		void
		update(const ros::Time& /*time*/, const ros::Duration& /*period*/) override;

	private:
		ros::Subscriber sub_command;

		Eigen::Vector6d q_d;
		Eigen::Vector6d q_dot_d;
		//Eigen::Vector6d q_ddot_d;
		double kp = 800.0;
		double kd = 400.0;

		bool
		init_KDL();

		Eigen::Vector6d
		get_position();

		Eigen::Vector6d
		get_velocity();

		Eigen::Vector6d
		get_gravity(const Eigen::Vector6d& q_eigen);

		Eigen::Vector6d
		get_mass(const Eigen::Vector6d& q_eigen);

		Eigen::Vector6d
		saturate_rotatum(const Eigen::Vector6d& tau_des, const double period = 0.001 /* [s] */);

		void
		callback_command(const sensor_msgs::JointStateConstPtr& msg);
	};
}