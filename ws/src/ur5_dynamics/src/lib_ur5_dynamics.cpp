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
	// kdl_jac_dot_solver = new KDL::ChainJntToJacDotSolver(kdl_chain);
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

	static auto q_kdl    = KDL::JntArray(NUM_JOINTS);
	static auto qdot_kdl = KDL::JntArray(NUM_JOINTS);
	static auto C_kdl    = KDL::JntArray(NUM_JOINTS);

	// load values of q and qdot from Eigen to JntArray
	for (size_t i = 0; i < NUM_JOINTS; ++i)
	{
		q_kdl(i) = q[i];
		qdot_kdl(i) = qdot[i];
	}

	kdl_dyn_solver->JntToCoriolis(q_kdl, qdot_kdl, C_kdl);

	return C_kdl.data;
}

Eigen::Matrix4d
ur5_dynamics::fwd_kin(const Eigen::Vector6d& q)
{
	ur5_dynamics::check_init();

	static auto ee_frame = KDL::Frame();
	static auto q_kdl    = KDL::JntArray(NUM_JOINTS);

	// load values of q from Eigen to JntArray
	for (size_t i = 0; i < NUM_JOINTS; ++i)
		q_kdl(i) = q[i];

	kdl_fk_solver->JntToCart(q_kdl, ee_frame);

	static Eigen::Matrix4d T = Eigen::Matrix4d::Zero();
	for (size_t i = 0; i < 4; ++i)
		for (size_t j = 0; j < 4; ++j)
			T(i, j) = ee_frame(i, j);

	return T;
}

Eigen::Matrix6d
jacobian(const Eigen::Vector6d& q)
{
	return Eigen::Matrix6d::Zero();
}