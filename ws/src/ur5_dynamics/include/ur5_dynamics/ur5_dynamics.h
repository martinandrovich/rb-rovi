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


#include <Eigen/Core>

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

	static Eigen::Vector6d
	fwd_kin(const Eigen::Vector6d& q);
	
	static Eigen::Vector6d
	inv_kin(const Eigen::Vector6d& T);

	static inline const std::string   ROBOT_NAME        = "ur5";
	static inline const std::string   ROBOT_DESCRIPTION = "/robot_description";
	static inline constexpr auto      NUM_JOINTS        = 6;
	static inline constexpr auto      GRAVITY           = -9.80665;
	static inline const std::string   BASE_LINK         = ROBOT_NAME + "_link0";
	static inline const std::string   LAST_LINK         = ROBOT_NAME + "_link6";

private:

	static void
	check_init();

	static inline bool                is_init = false;

	static inline urdf::Model         robot_model;
	static inline KDL::Chain          kdl_chain;
	static inline KDL::ChainDynParam* kdl_dyn_solver;
	// static inline KDL::ChainJntToJacSolver*    kdl_jac_solver;
	// static inline KDL::ChainJntToJacDotSolver* kdl_jac_dot_solver;
};
