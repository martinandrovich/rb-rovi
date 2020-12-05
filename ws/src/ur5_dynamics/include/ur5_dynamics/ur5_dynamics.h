#pragma once

#include <string>
#include <mutex>
#include <optional>

#include <ros/ros.h>
#include <urdf/model.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>

#include <geometry_msgs/Pose.h>

#include <Eigen/Eigen>

namespace Eigen
{
	using Matrix6d = Eigen::Matrix<double, 6, 6>;
	using Vector6d = Eigen::Matrix<double, 6, 1>;
}

class ur5_dynamics
{

public:

	static bool
	init();

	static Eigen::Vector6d
	gravity(const Eigen::Vector6d& q);

	static Eigen::Matrix6d
	mass(const Eigen::Vector6d& q);

	static Eigen::Vector6d
	coriolis(const Eigen::Vector6d& q, const Eigen::Vector6d& qdot);

	template<typename T = Eigen::Matrix4d>
	static T
	fwd_kin(const Eigen::Vector6d& q);

	template<typename T = Eigen::Matrix4d>
	static Eigen::Vector6d
	inv_kin(const T& frame, const Eigen::Vector6d& q);

	template<typename T = Eigen::Matrix4d>
	static Eigen::MatrixXd 
	inv_kin(const T& frame);

	template<typename T = Eigen::Vector6d>
	static Eigen::Matrix6d
	pinv_jac(const T& arg, const double eps = 1.0e-5);

	template<typename T = Eigen::Vector6d>
	static Eigen::Matrix6d
	mani(const T& arg);

	static Eigen::Matrix6d
	jac(const Eigen::Vector6d& q);

	static Eigen::Matrix6d
	jac_dot(const Eigen::Vector6d& q, const Eigen::Vector6d& qdot);

	static inline const std::string  ROBOT_NAME        = "ur5";
	static inline const std::string  ROBOT_DESCRIPTION = "/robot_description";
	static inline constexpr auto     NUM_JOINTS        = 6;
	static inline constexpr auto     GRAVITY           = -9.80665;

	// without end-effector
	static inline const std::string  BASE_LINK         = ROBOT_NAME + "_link0";
	static inline const std::string  LAST_LINK         = ROBOT_NAME + "_ee";

	// transforms (defined in ur5_arm.xacro)
	static inline const auto         l6_T_ee           = Eigen::Translation3d(0, 0.0823, 0) * Eigen::Isometry3d::Identity();
	static inline const auto         ee_T_tcp          = Eigen::Translation3d(0, 0.1507, 0) * Eigen::Isometry3d::Identity();

private:

	static void
	check_init();

	static inline bool               is_init = false;
	static inline urdf::Model        robot_model;
	
	static inline KDL::Chain                       kdl_chain;
	static inline KDL::ChainDynParam*              kdl_dyn_solver;
	static inline KDL::ChainJntToJacSolver*        kdl_jac_solver;
	static inline KDL::ChainJntToJacDotSolver*     kdl_jac_dot_solver;
	static inline KDL::ChainFkSolverPos_recursive* kdl_fk_solver;
};
