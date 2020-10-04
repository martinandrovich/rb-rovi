#pragma once

#include <string>
#include <vector>

#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_buffer.h>
#include <urdf/model.h>

#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>

#include <Eigen/Core>

namespace Eigen
{
	using Matrix6d = Eigen::Matrix<double, 6, 6>;
	using Vector6d = Eigen::Matrix<double, 6, 1>;
}

namespace ur5_controllers
{
class JointPositionPDGravityController final
	: public controller_interface::Controller<hardware_interface::EffortJointInterface>
{

public:

	static inline constexpr auto CONTROLLER_NAME  = "JointPositionPDGravityController";
	static inline constexpr auto SATURATE_ROTATUM = true;
	static inline constexpr auto TAU_DOT_MAX      = 1000.;

	std::vector<std::string> vec_joint_names;
	size_t num_joints;

	std::vector<hardware_interface::JointHandle> vec_joints;
	realtime_tools::RealtimeBuffer<std::vector<double>> commands_buffer;

	JointPositionPDGravityController() {}
	~JointPositionPDGravityController() { sub_command.shutdown(); }

	bool
	init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh) override;

	void
	starting(const ros::Time& time) override;

	void 
	update(const ros::Time& /*time*/, const ros::Duration& /*period*/) override;

private:

	ros::Subscriber sub_command;
	ros::Subscriber sub_joint_state;

	double kp = 200.0;
	double kd = 100.0;

	// constants
	static inline const std::string            ROBOT_DESCRIPTION = "/robot_description";
	static inline constexpr auto               NUM_JOINTS        = 6;
	static inline constexpr auto               GRAVITY           = -9.80665;
	static inline const std::vector<double>    Q_D_INIT          = { 0, -M_PI_2, 0, 0, 0, 0 };

	static inline urdf::Model                  robot_model;

	static inline KDL::Chain                   kdl_chain;
	static inline KDL::ChainDynParam*          kdl_dyn_solver;
	static inline KDL::ChainJntToJacSolver*    kdl_jac_solver;
	static inline KDL::ChainJntToJacDotSolver* kdl_jac_dot_solver;

	Eigen::Vector6d q_d;

	bool
	init_KDL();

	Eigen::Vector6d
	get_position();

	Eigen::Vector6d
	get_velocity();

	Eigen::Vector6d
	get_gravity(const Eigen::Vector6d& q_eigen);

	Eigen::Vector6d
	saturate_rotatum(const Eigen::Vector6d& tau_des, const double period = 0.001 /* [s] */);

	void
	callback_command(const std_msgs::Float64MultiArrayConstPtr& msg);

};
}