#include "ur5_inv.cpp"
#include <ur5_dynamics/ur5_dynamics.h>

bool
ur5_dynamics::init()
{
	if (is_init)
	{
		ROS_WARN("KDL is already initialized.");
		return true;
	}

	if (not robot_model.initParam(ROBOT_DESCRIPTION))
	{
		ROS_ERROR("Could not load URDF robot model from '%s'.", ROBOT_DESCRIPTION.c_str());
		return false;
	}

	// compose KDL tree
	KDL::Tree kdl_tree;
	if (not kdl_parser::treeFromUrdfModel(robot_model, kdl_tree))
	{
		ROS_ERROR("Could not construct KDL tree from robot model.");
		return false;
	};

	// load KDL chain
	kdl_tree.getChain(BASE_LINK, LAST_LINK, kdl_chain);

	// initialize KDL solver(s)
	kdl_dyn_solver = new KDL::ChainDynParam(kdl_chain, KDL::Vector(0, 0, GRAVITY));
	kdl_jac_solver = new KDL::ChainJntToJacSolver(kdl_chain);
	kdl_jac_dot_solver = new KDL::ChainJntToJacDotSolver(kdl_chain);
	kdl_fk_solver = new KDL::ChainFkSolverPos_recursive(kdl_chain);

	// done
	ROS_INFO("Initialized KDL for ur5_dynamics.");
	is_init = true;

	return true;
}

void
ur5_dynamics::check_init()
{
	if (not is_init)
	{
		ROS_WARN("KDL has yet not been initalized; initiliaizing now...");
		ur5_dynamics::init();
	}
}

Eigen::Vector6d
ur5_dynamics::gravity(const Eigen::Vector6d& q)
{
	ur5_dynamics::check_init();

	static auto q_kdl = KDL::JntArray(NUM_JOINTS);
	static auto g_kdl = KDL::JntArray(NUM_JOINTS);

	// load values of q from Eigen to JntArray
	for (size_t i = 0; i < NUM_JOINTS; ++i)
		q_kdl(i) = q[i];

	// compute gravity
	kdl_dyn_solver->JntToGravity(q_kdl, g_kdl);

	// return as Eigen vector
	return g_kdl.data;
}

Eigen::Matrix6d
ur5_dynamics::mass(const Eigen::Vector6d& q)
{
	ur5_dynamics::check_init();

	static auto q_kdl = KDL::JntArray(NUM_JOINTS);
	static auto M_kdl = KDL::JntSpaceInertiaMatrix(NUM_JOINTS);

	// load values of q from Eigen to JntArray
	for (size_t i = 0; i < NUM_JOINTS; ++i)
		q_kdl(i) = q[i];

	// compute gravity
	kdl_dyn_solver->JntToMass(q_kdl, M_kdl);

	// return as Eigen vector
	return M_kdl.data;
}

Eigen::Vector6d
ur5_dynamics::coriolis(const Eigen::Vector6d& q, const Eigen::Vector6d& qdot)
{
	ur5_dynamics::check_init();

	static auto q_kdl = KDL::JntArray(NUM_JOINTS);
	static auto qdot_kdl = KDL::JntArray(NUM_JOINTS);
	static auto C_kdl = KDL::JntArray(NUM_JOINTS);

	// load values of q and qdot from Eigen to JntArray
	for (size_t i = 0; i < NUM_JOINTS; ++i)
	{
		q_kdl(i) = q[i];
		qdot_kdl(i) = qdot[i];
	}

	kdl_dyn_solver->JntToCoriolis(q_kdl, qdot_kdl, C_kdl);

	return C_kdl.data;
}

template<typename T>
T ur5_dynamics::fwd_kin(const Eigen::Vector6d& q)
{
	static_assert(std::is_same<T, Eigen::Matrix4d>::value || std::is_same<T, geometry_msgs::Pose>::value,
	              "Wrong type use Matrix4d or Pose.");

	ur5_dynamics::check_init();

	static auto ee_frame = KDL::Frame();
	static auto q_kdl = KDL::JntArray(NUM_JOINTS);

	// load values of q from Eigen to JntArray
	for (size_t i = 0; i < NUM_JOINTS; ++i)
		q_kdl(i) = q[i];

	kdl_fk_solver->JntToCart(q_kdl, ee_frame, -1);

	static Eigen::Matrix4d frame = Eigen::Matrix4d::Zero();

	for (size_t i = 0; i < 4; ++i)
		for (size_t j = 0; j < 4; ++j)
			frame(i, j) = ee_frame(i, j);

	if constexpr (std::is_same<T, Eigen::Matrix4d>::value)
	{
		return frame;
	}

	if constexpr (std::is_same<T, geometry_msgs::Pose>::value)
	{
		geometry_msgs::Pose pose;

		// Position
		pose.position.x = frame(0, 3);
		pose.position.y = frame(1, 3);
		pose.position.z = frame(2, 3);

		// Rotation Matrix to Quat
		Eigen::Quaterniond quat(frame.topLeftCorner<3, 3>());
		pose.orientation.x = quat.x();
		pose.orientation.y = quat.y();
		pose.orientation.z = quat.z();
		pose.orientation.w = quat.w();

		return pose;
	}
}

template<typename T>
Eigen::MatrixXd ur5_dynamics::inv_kin(const T& pose)
{

	static_assert(std::is_same<T, Eigen::Matrix4d>::value || std::is_same<T, geometry_msgs::Pose>::value,
	              "Wrong type use Matrix4d or Pose.");

	Eigen::Matrix4d frame = ( Eigen::Matrix4d() <<  1, 0, 0, 0, 
													0, 1, 0, 0, 
													0, 0, 1, 0, 
													0, 0, 0, 1 ).finished();

	if constexpr (std::is_same<T, geometry_msgs::Pose>::value)
	{
		// Position
		frame.topRightCorner<3, 1>() << pose.position.x, pose.position.y, pose.position.z;

		// Orientation
		Eigen::Quaterniond quat(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
		frame.topLeftCorner<3, 3>() = quat.toRotationMatrix();
	}

	if constexpr (std::is_same<T, Eigen::Matrix4d>::value)
	{
		frame = pose;
	}

	static auto T_z = (Eigen::Matrix4d() << 0, -1, 0, 0, 
											1,  0, 0, 0, 
											0,  0, 1, 0, 
											0,  0, 0, 1).finished();

	Eigen::MatrixXd q_sol = inverse(frame * T_z);

	// if rows == 0, then no solution is found
	if (q_sol.rows() == 0)
	{
		ROS_ERROR_STREAM("There is no inverse kinematics!");
	}

	// check if joints are within -180:180 degrees
	for (size_t i = 0; i < q_sol.rows(); i++)
	{
		for (size_t j = 0; j < q_sol.cols(); j++)
		{
			if (q_sol(i,j) > M_PI)
				q_sol(i,j) -= 2*M_PI;
			else if( q_sol(i,j) > M_PI)
				q_sol(i,j) += 2*M_PI;
		}
	}

	return q_sol;
}

template<typename T>
Eigen::Vector6d ur5_dynamics::inv_kin(const T& pose, const Eigen::Vector6d& q)
{
	static_assert(std::is_same<T, Eigen::Matrix4d>::value || std::is_same<T, geometry_msgs::Pose>::value,
	              "Wrong type use Matrix4d or Pose.");

	Eigen::Matrix4d frame = ( Eigen::Matrix4d() <<  1, 0, 0, 0, 
													0, 1, 0, 0, 
													0, 0, 1, 0, 
													0, 0, 0, 1 ).finished();

	if constexpr (std::is_same<T, geometry_msgs::Pose>::value)
	{
		// Position
		frame.topRightCorner<3, 1>() << pose.position.x, pose.position.y, pose.position.z;

		// Orientation
		Eigen::Quaterniond quat(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
		frame.topLeftCorner<3, 3>() = quat.toRotationMatrix();
	}

	if constexpr (std::is_same<T, Eigen::Matrix4d>::value)
	{
		frame = pose;
	}

	static auto T_z = (Eigen::Matrix4d() << 0, -1, 0, 0, 
											1,  0, 0, 0, 
											0,  0, 1, 0, 
											0,  0, 0, 1).finished();

	Eigen::MatrixXd q_sol = inverse(frame * T_z);

	// if rows == 0, then no solution is found
	if (q_sol.rows() == 0)
	{
		ROS_ERROR_STREAM("There is no inverse kinematics!");
		return q;
	}
	
	// check if joints are within -180:180 degrees
	for (size_t i = 0; i < q_sol.rows(); i++)
	{
		for (size_t j = 0; j < q_sol.cols(); j++)
		{
			if (q_sol(i,j) > M_PI)
				q_sol(i,j) -= 2*M_PI;
			else if( q_sol(i,j) > M_PI)
				q_sol(i,j) += 2*M_PI;
		}
	}

	// least euclidean distance is determined
	int opt_idx = 0;
	double least_euclidean = std::numeric_limits<double>::max();

	for (size_t i = 0; i < q_sol.rows(); i++)
	{
		double sum = 0.f;

		for (size_t j = 0; j < NUM_JOINTS; j++)
			sum += std::pow(q_sol(i, j) - q(j), 2);

		if (sum < least_euclidean)
		{
			opt_idx = i;
			least_euclidean = sum;
		}
	}

	return q_sol.row(opt_idx);
}

Eigen::Matrix6d
ur5_dynamics::jac(const Eigen::Vector6d& q)
{
	ur5_dynamics::check_init();

	static auto q_kdl = KDL::JntArray(NUM_JOINTS);
	static auto geo_jac = KDL::Jacobian(NUM_JOINTS);

	for (size_t i = 0; i < NUM_JOINTS; ++i)
		q_kdl(i) = q[i];

	kdl_jac_solver->JntToJac(q_kdl, geo_jac);

	return geo_jac.data;
}

Eigen::Matrix6d
ur5_dynamics::jac_dot(const Eigen::Vector6d& q, const Eigen::Vector6d& qdot)
{
	ur5_dynamics::check_init();

	static auto q_kdl = KDL::JntArray(NUM_JOINTS);
	static auto qdot_kdl = KDL::JntArray(NUM_JOINTS);
	static auto geo_jac_dot = KDL::Jacobian(NUM_JOINTS);

	// load values of q and qdot from Eigen to JntArray
	for (size_t i = 0; i < NUM_JOINTS; ++i)
	{
		q_kdl(i) = q[i];
		qdot_kdl(i) = qdot[i];
	}

	// fill joint values and velocities into a velocity array
	static auto velo_arr = KDL::JntArrayVel(q_kdl, qdot_kdl);

	// determine jacobian dot, the geometric one
	kdl_jac_dot_solver->JntToJacDot(velo_arr, geo_jac_dot);

	return geo_jac_dot.data;
}

template<typename T>
Eigen::Matrix6d ur5_dynamics::pinv_jac(const T& arg, double eps)
// this is not the dynamical consistent pseudo inverse, but 
// it should for later usage be implemented, for now, this is fine.
{
	static_assert(std::is_same<T, Eigen::Matrix6d>::value || std::is_same<T, Eigen::Vector6d>::value,
	              "Wrong type use Matrix6d or Vector6d.");

	Eigen::Matrix6d jac;

	// Determine jacobian
	if constexpr (std::is_same<T, Eigen::Vector6d>::value)
	{
		jac = ur5_dynamics::jac(arg);
	}
	
	// Jacobian was an argument
	if constexpr (std::is_same<T, Eigen::Matrix6d>::value)
	{
		jac = arg;
	}
	
	// Transpose
	Eigen::Matrix6d jac_T = jac.transpose();
	Eigen::Matrix6d jac_sym = jac_T * jac;

	// Define SVD object
	Eigen::JacobiSVD<Eigen::Matrix6d> svd(jac_sym, Eigen::ComputeFullU | Eigen::ComputeFullV);

	// Pseudo Inverse
	Eigen::Matrix6d singular_inv = Eigen::Matrix6d::Zero();

	for (size_t i = 0; i < jac.rows(); i++)
	{
		// singular_inv with dampening
		singular_inv(i,i) = 1/(svd.singularValues()(i) + eps*eps);	
	}
	
	// Calculate pinv
	return svd.matrixV() * singular_inv * svd.matrixU().transpose() * jac_T;
}

template<typename T>
Eigen::Matrix6d ur5_dynamics::mani(const T& arg)
{

	static_assert(std::is_same<T, Eigen::Matrix6d>::value || std::is_same<T, Eigen::Vector6d>::value,
	              "Wrong type use Matrix6d or Vector6d.");

	Eigen::Matrix6d jac;

	// Determine jacobian
	if constexpr (std::is_same<T, Eigen::Vector6d>::value)
	{
		jac = ur5_dynamics::jac(arg);
	}
	
	// Jacobian was an argument
	if constexpr (std::is_same<T, Eigen::Matrix6d>::value)
	{
		jac = arg;
	}

	Eigen::Matrix6d man = jac*jac.transpose();

	// Define SVD object
	Eigen::JacobiSVD<Eigen::Matrix6d> svd(man, Eigen::ComputeFullU | Eigen::ComputeFullV);

	// Get the EigenVectors
	man.block<3, 3>(0, 0) << svd.matrixV().block<3, 3>(0, 0);

	// Get the EigenValues
	Eigen::Matrix3d singular = Eigen::Matrix3d::Zero();
	
	for (size_t i = 0; i < singular.rows(); i++)
	{
		singular(i, i) = svd.singularValues()(i);
	}

	// Put inside manipulability matrix
	man.block<3, 3>(3, 3) << singular;

	return man;
}

template Eigen::Matrix4d ur5_dynamics::fwd_kin<Eigen::Matrix4d>(const Eigen::Vector6d& q);

template geometry_msgs::Pose ur5_dynamics::fwd_kin<geometry_msgs::Pose>(const Eigen::Vector6d& q);

template Eigen::Vector6d ur5_dynamics::inv_kin<Eigen::Matrix4d>(const Eigen::Matrix4d& pose, const Eigen::Vector6d& q);

template Eigen::Vector6d ur5_dynamics::inv_kin<geometry_msgs::Pose>(const geometry_msgs::Pose& pose, const Eigen::Vector6d& q);

template Eigen::MatrixXd ur5_dynamics::inv_kin<Eigen::Matrix4d>(const Eigen::Matrix4d& pose);

template Eigen::MatrixXd ur5_dynamics::inv_kin<geometry_msgs::Pose>(const geometry_msgs::Pose& pose);

template Eigen::Matrix6d ur5_dynamics::pinv_jac<Eigen::Matrix6d>(const Eigen::Matrix6d& jac, const double eps);

template Eigen::Matrix6d ur5_dynamics::pinv_jac<Eigen::Vector6d>(const Eigen::Vector6d& q, const double eps);

template Eigen::Matrix6d ur5_dynamics::mani<Eigen::Matrix6d>(const Eigen::Matrix6d& jac);

template Eigen::Matrix6d ur5_dynamics::mani<Eigen::Vector6d>(const Eigen::Vector6d& q);