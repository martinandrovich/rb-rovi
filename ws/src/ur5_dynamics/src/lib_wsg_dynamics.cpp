#include <wsg_dynamics/wsg_dynamics.h>

bool
wsg_dynamics::init()
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
	KDL::Tree kdl_tree[FINGERS];

    for(int i = 0; i < FINGERS; i++)
    {
        if (not kdl_parser::treeFromUrdfModel(robot_model, kdl_tree[i]))
        {
            ROS_ERROR("Could not construct KDL tree for the WSG model.");
            return false;
        };
    }

	// load KDL chain
	kdl_tree[LEFT].getChain(BASE, LEFT_FINGER, kdl_chain[LEFT]);
    kdl_tree[RIGHT].getChain(BASE, RIGHT_FINGER, kdl_chain[RIGHT]);

	// initialize KDL solver(s)
    kdl_dyn_solver[LEFT] = new KDL::ChainDynParam(kdl_chain[LEFT], KDL::Vector(0, 0, GRAVITY));
	kdl_dyn_solver[RIGHT] = new KDL::ChainDynParam(kdl_chain[RIGHT], KDL::Vector(0, 0, GRAVITY));

	// done
	ROS_INFO("Initialized KDL for WSG.");
	is_init = true;
	return true;
}

// TODO 
// * add entire chain for ur5 for feedback lineraization of the controller, however it 
// * might be adaquete to just make a pid

void
wsg_dynamics::check_init()
{
    if (is_init) return;
	else wsg_dynamics::init();
}

Eigen::Vector2d
wsg_dynamics::gravity(const Eigen::Vector2d& q)
{
	wsg_dynamics::check_init();

	for (size_t i = 0; i < FINGERS; i++)
	{
		/* code */
	}

}

Eigen::Matrix2d
wsg_dynamics::mass(const Eigen::Vector2d& q)
{
	wsg_dynamics::check_init();

	for (size_t i = 0; i < FINGERS; i++)
	{
		/* code */
	}
}

Eigen::Vector2d
wsg_dynamics::coriolis(const Eigen::Vector2d& q, const Eigen::Vector2d& qdot)
{
	wsg_dynamics::check_init();

	for (size_t i = 0; i < FINGERS; i++)
	{
		/* code */
	}
}