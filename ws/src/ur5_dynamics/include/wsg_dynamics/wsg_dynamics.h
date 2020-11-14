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

#include <wsg_dynamics/wsg_dynamics.h>
#include <geometry_msgs/Pose.h>


#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Geometry>

class wsg_dynamics
{
public:

	static bool
	init();

	static Eigen::Vector2d
	gravity(const Eigen::Vector2d& q, const Eigen::Quaterniond& pose);

	static Eigen::Matrix2d
	mass(const Eigen::Vector2d& q);

	static Eigen::Vector2d
	coriolis(const Eigen::Vector2d& q, const Eigen::Vector2d& qdot);

	static inline const std::string  ROBOT_NAME        = "wsg";
	static inline const std::string  ROBOT_DESCRIPTION = "/robot_description";
	static inline constexpr auto     NUM_JOINTS        = 1;
	static inline constexpr auto     GRAVITY           = -9.80665;
	static inline const std::string  BASE              = ROBOT_NAME + "_base";
	static inline const std::string  LEFT_FINGER       = ROBOT_NAME + "_fingertip_right";
    static inline const std::string  RIGHT_FINGER      = ROBOT_NAME + "_fingertip_left";
	static inline const int			 FINGERS		   = 2;
	static inline const int 	     LEFT			   = 0;
	static inline const int 		 RIGHT		   	   = 1;	

private:

	static void
	check_init();

	static inline bool               is_init 		   = false;
	static inline urdf::Model        robot_model;
	
	static inline KDL::Chain                       kdl_chain[FINGERS];
	static inline KDL::ChainDynParam*              kdl_dyn_solver[FINGERS];
};
