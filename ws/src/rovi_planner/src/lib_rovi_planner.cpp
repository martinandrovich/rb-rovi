#include "rovi_planner/rovi_planner.h"

#include <tf_conversions/tf_kdl.h>

#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/trajectory_processing/trajectory_tools.h>

#include <kdl/frames.hpp>
#include <kdl/path_point.hpp>
#include <kdl/path_line.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/trajectory_stationary.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/velocityprofile_spline.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/utilities/error.h>

KDL::Trajectory_Composite
rovi_planner::traj_linear(const std::vector<geometry_msgs::Pose>& waypoints, double vel_max, double acc_max, double equiv_radius)
{
	// equivalent radius: serves to compare rotations and translations; the "amount of motion" (pos,vel,acc)
	// of the rotation is taken to be the amount motion of a point at distance eqradius from the rotation axis.

	if (waypoints.size() < 2)
		throw std::runtime_error("There must be at least two waypoints.");

	auto traj                = new KDL::Trajectory_Composite();
	static auto interpolator = new KDL::RotationalInterpolation_SingleAxis();

	// convert vector<Pose> to vector<KDL::Frame>
	std::vector<KDL::Frame> frames;
	for (const auto& pt : waypoints)
	{
		static KDL::Frame frame;
		tf::poseMsgToKDL(pt, frame);
		frames.push_back(frame);
	}

	// try creating the trajectory; catch any errors
	try
	{
		// create a trajectory segment for each waypoint defined as a path line betwen the current and next point with some velocity profile
		// http://docs.ros.org/en/melodic/api/orocos_kdl/html/classKDL_1_1Path__Line.html#a1ea3f21f577aee2a4252c5a802b6a7f2
		for (size_t i = 0; i < frames.size() - 1; ++i)
		{

			// create path
			const auto path = new KDL::Path_Line(frames[i], frames[i + 1], interpolator, equiv_radius);

			// define velocity profile for path
			auto vel_profile = new KDL::VelocityProfile_Trap(vel_max, acc_max);
			vel_profile->SetProfile(0, path->PathLength());

			// construct segment from path and velocity profile
			const auto traj_seg = new KDL::Trajectory_Segment(path, vel_profile);
			traj->Add(traj_seg);
		}
	}
	catch (const KDL::Error& e)
	{
		ROS_ERROR("Could not plan trajectory.");

		ROS_INFO("Planning was attempted with following waypoints:\n\n");
		for (auto const& point : frames)
			std::cout << point << "\n\n";

		std::cerr << e.Description() << std::endl;
		std::cerr << e.GetType() << std::endl;

		exit(-1);
	}

	return *traj;
}

std::array<KDL::Trajectory_Composite*, 6>
rovi_planner::traj_linear(const std::vector<sensor_msgs::JointState>& joint_states, double vel_max, double acc_max, double equiv_radius)
{
	// linear joint trajectory interpolation

	if (joint_states.size() < 2)
		throw std::runtime_error("There must be at least two joint states.");

	auto trajs               = std::array<KDL::Trajectory_Composite* , 6>();
	static auto interpolator = new KDL::RotationalInterpolation_SingleAxis();

	// populate array with new trajectory composites
	for (int i = 0; i < 6; ++i)
		trajs[i] = new KDL::Trajectory_Composite();
	
	// lambda to convert angle to KDL frame
	auto angle_to_frame = [](auto& angle) { return KDL::Frame(KDL::Rotation::RPY(0, 0, 0), KDL::Vector(angle, 0, 0)); };

	// try creating the trajectory; catch any errors
	try
	{
		for (size_t i = 0; i < joint_states.size() - 1; ++i)
		{
			const auto NUM_JOINTS = joint_states[i].position.size();

			for (size_t j = 0; j < NUM_JOINTS; ++j)
			{
				// define frames
				const auto [frame_from, frame_to] = std::tuple { angle_to_frame(joint_states[i].position[j]), angle_to_frame(joint_states[i + 1].position[j]) };

				// create path
				const auto path = new KDL::Path_Line(frame_from, frame_to, interpolator, equiv_radius);

				// define velocity profile for path
				auto vel_profile = new KDL::VelocityProfile_Trap(vel_max, acc_max);
				vel_profile->SetProfile(0, path->PathLength());

				// construct segment from path and velocity profile
				const auto traj_seg = new KDL::Trajectory_Segment(path, vel_profile);
				trajs[j]->Add(traj_seg);
			}
		}
	}
	catch (const KDL::Error& e)
	{
		ROS_ERROR("Could not plan trajectory.");

		std::cerr << e.Description() << std::endl;
		std::cerr << e.GetType() << std::endl;

		exit(-1);
	}

	// return array of KDL trajectory composites
	return trajs;
}

std::array<KDL::Trajectory_Composite*, 6>
rovi_planner::traj_parabolic(const std::vector<sensor_msgs::JointState>& joint_states, double vel_max, double acc_max, double corner_radius, double equiv_radius)
{
	// parabolic joint trajectory interpolation

	if (joint_states.empty())
		throw std::runtime_error("There must be at least one joint state.");

	auto trajs                = std::array<KDL::Trajectory_Composite* , 6>();
	auto paths                = std::array<KDL::Path_RoundedComposite* , 6>();
	constexpr auto NUM_JOINTS = trajs.size();
	static auto interpolator  = new KDL::RotationalInterpolation_SingleAxis();

	// populate array with new trajectory composites
	for (size_t i = 0; i < NUM_JOINTS; ++i)
	{
		trajs[i] = new KDL::Trajectory_Composite();
		paths[i] = new KDL::Path_RoundedComposite(corner_radius, equiv_radius, interpolator);
	}
	
	// lambda to convert angle to KDL frame
	auto angle_to_frame = [](auto& angle) { return KDL::Frame(KDL::Rotation::RPY(0, 0, 0), KDL::Vector(angle, 0, 0)); };

	// try creating the trajectory; catch any errors
	try
	{
		// add all waypoints (frames) to path
		for (size_t n = 0; n < NUM_JOINTS; ++n)
		{
			for (size_t i = 0; i < joint_states.size(); ++i)
			{
				const auto angle = joint_states[i].position[n];
				paths[n]->Add(angle_to_frame(angle));
			}
			
			// finish creating path for joint n
			paths[n]->Finish();

			// define velocity profile based on path start and end
			auto vel_profile  = new KDL::VelocityProfile_Trap(vel_max, acc_max);
			vel_profile->SetProfile(0, paths[n]->PathLength());

			// add trajectory segment from path and velocity profile to final trajectory
			auto traj_seg = new KDL::Trajectory_Segment(paths[n], vel_profile);
			trajs[n]->Add(traj_seg);
		}
	}
	catch (const KDL::Error& e)
	{
		ROS_ERROR("Could not plan trajectory.");

		std::cerr << e.Description() << std::endl;
		std::cerr << e.GetType() << std::endl;

		exit(-1);
	}

	// return array of KDL trajectory composites
	return trajs;
}

KDL::Trajectory_Composite
rovi_planner::traj_parabolic(const std::vector<geometry_msgs::Pose>& waypoints, double vel_max, double acc_max, double corner_radius, double equiv_radius)
{

	// https://github.com/DonSiMP/trajectory_generators

	// Trajectory_Composite implements a trajectory that is composed of underlying
	// trajectoria (Trajectory_Segment objects). A single trajectory segment is generated
	// by defining a path (Path_RoundedComposite), composed of waypoints (KDL::Frame(s))
	// with rounded corners and a trapezoidal velocity profile.

	if (waypoints.empty())
		throw std::runtime_error("There must be at least one waypoint.");

	auto interpolator = new KDL::RotationalInterpolation_SingleAxis();
	auto traj         = new KDL::Trajectory_Composite();
	auto path         = new KDL::Path_RoundedComposite(corner_radius, equiv_radius, interpolator);

	// convert vector<Pose> to vector<KDL::Frame>
	std::vector<KDL::Frame> frames;

	for (const auto& pt : waypoints)
	{
		static KDL::Frame frame;
		tf::poseMsgToKDL(pt, frame);
		frames.push_back(frame);
	}

	// try creating the trajectory; catch any errors
	try
	{

	// there are multiple waypoints
	if (waypoints.size() > 1)
	{
		// add all waypoints (frames) to path
		for (auto pt : frames)
			path->Add(pt);

		// finish creating the path
		path->Finish();

		// define velocity profile based on path start and end
		auto vel_profile  = new KDL::VelocityProfile_Trap(vel_max, acc_max);
		vel_profile->SetProfile(0, path->PathLength());

		// add trajectory segment from path and velocity profile to final trajectory
		auto traj_seg = new KDL::Trajectory_Segment(path, vel_profile);
		traj->Add(traj_seg);
	}
	// there is a single waypoint (endpoint)
	else
	{
		auto traj_seg = new KDL::Trajectory_Segment(new KDL::Path_Point(frames[0]), new KDL::VelocityProfile_Trap(vel_max, acc_max));
		traj->Add(traj_seg);
	}

	// wait 0.5 seconds at the end of trajectory
	// traj->Add(new KDL::Trajectory_Stationary(0.5, frames.back()));

	}
	catch (const KDL::Error& e)
	{
		ROS_ERROR("Could not plan trajectory.");

		ROS_INFO("Planning was attempted with following waypoints:\n\n");
		for (auto const& point : frames)
			std::cout << point << "\n\n";

		std::cerr << e.Description() << std::endl;
		std::cerr << e.GetType() << std::endl;

		exit(-1);
	}

	return *traj;
}

bool
rovi_planner::moveit_planner::init(ros::NodeHandle & nh)
{
	// create the model of the robot
	robot_model_loader = std::make_shared<robot_model_loader::RobotModelLoader>(ROBOT_DESCRIPTION);
	robot_model = robot_model_loader->getModel();

	// create planning scene
	planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model);

	// create planner objects
	planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
	planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(PLANNER_PLUGIN_NAME));

	// create publishers
	ros::Publisher planning_scene_pub 	= nh.advertise<moveit_msgs::PlanningScene>(		PLANNING_SCENE_TOPIC, 1);
	ros::Publisher traj_pub 			= nh.advertise<moveit_msgs::DisplayTrajectory>(	TRAJECTORY_TOPIC, 1);

	// check for success
	if (!planner_instance->initialize(robot_model, ros::this_node::getNamespace()))
	{
		// do some error handling here
		ROS_FATAL_STREAM("Could not initialize planner instance... exiting...");
		is_init = false;
	}
	else
	{
		is_init = true;
	}

	return is_init;
}

void
rovi_planner::moveit_planner::destruct()
{
	planner_instance->terminate();
}

planning_interface::MotionPlanResponse
rovi_planner::moveit_planner::traj_moveit(const geometry_msgs::Pose& pose_des, const std::string& planner, std::vector<double> q)
{
	// this sets the current planner
	auto check_planner = [&]()
	{
		for (const auto& plan : AVAILABLE_PLANNERS)
			if (plan == planner) return planner;

		ROS_WARN_STREAM("The specified planner '" << planner << "' is not available; resorting to '" << DEFAULT_PLANNER << "'.");
		return std::string(DEFAULT_PLANNER);
	};

	// set the planner
	ros::param::set(std::string(ARM_GROUP) + "/default_planner_config", check_planner());

	// set robot_state, arm_group and wsg_group ptrs
	auto& robot_state    = planning_scene->getCurrentStateNonConst();

	const auto arm_group = robot_state.getJointModelGroup(ARM_GROUP);
	const auto wsg_group = robot_state.getJointModelGroup(WSG_GROUP);

	// set the robot_state to the current one from Gazebo
	if (q.size() != 6 )
	{
		ROS_INFO_STREAM("Waiting for sensor_msgs::JointState...");
		
		auto q_gazebo = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");
		q             = std::vector<double>(q_gazebo->position.begin(), q_gazebo->position.end() - 2);
		Eigen::Matrix<double, 6, 1> mat(q.data());

		ROS_INFO_STREAM("Got joint states:\n\n" << mat << "\n");
	}

	// set the robot state
	const std::vector<double> HOME_WSG{0.05, 0.05};
	robot_state.setJointGroupPositions(ARM_GROUP, q);
	robot_state.setJointGroupPositions(WSG_GROUP, HOME_WSG);

	// move the base
	ROS_INFO_STREAM("Waiting for gazebo_msgs::ModelStates...");

	auto model_states = ros::topic::waitForMessage<gazebo_msgs::ModelStates>("/gazebo/model_states");
	for(size_t i = 0; model_states->name.size() > i ; ++i)
	{
		if(model_states->name[i] == "ur5")
		{
			rovi_utils::move_base(robot_state,
			{
				model_states->pose[i].position.x,
				model_states->pose[i].position.y,
				model_states->pose[i].position.z
			});
		}
	}

	// add the collisions to the scene
	std::vector<moveit_msgs::CollisionObject> collision_objects = rovi_utils::get_gazebo_obj(planning_scene->getPlanningFrame());

	// add the collision objects
	moveit_msgs::PlanningScene planning_scene_msg;
	planning_scene->getPlanningSceneMsg(planning_scene_msg);
	planning_scene_msg.world.collision_objects = collision_objects;
	planning_scene->setPlanningSceneDiffMsg(planning_scene_msg);
	
	// make a motion plan request / response msg
	planning_interface::MotionPlanRequest req;
	planning_interface::MotionPlanResponse res;

	// desired pose with timestamp
	geometry_msgs::PoseStamped pose;
	pose.header.frame_id = "ur5_link0";
	pose.pose = pose_des;

	// specify the tolerances for the pose
	std::vector<double> tol_pos(3, 0.005);
	std::vector<double> tol_ori(3, 0.005);
	// set the kinematic_constraint
	moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints("ee_tcp", pose, tol_pos, tol_ori);

	// specify plan group and pushback the goal_constraints?
	req.group_name = ARM_GROUP;

	// planning time 
	req.allowed_planning_time = 1;
	req.num_planning_attempts = 10;

	// scale factor velocity / acceleration
	req.max_acceleration_scaling_factor = 1.0;
	req.max_velocity_scaling_factor = 1.0;

	// pushback the requirements
	req.goal_constraints.push_back(pose_goal);

	// get the planningcontext
	planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
	context->solve(res);

	if (res.error_code_.val != res.error_code_.SUCCESS)
	{
		// do some error handling here
		ROS_FATAL_STREAM("It was not possible to generate a trajectory... exiting...");
	}

	return res;
}

void 
rovi_planner::moveit_planner::start_async_publisher(const std::string & node, const double & seconds)
{
	auto find_pub = [&]()
	{
		for (auto & [ name, pub ]: { std::tuple(AVAILABLE_PUBS.begin()[0], planning_scene_pub), std::tuple(AVAILABLE_PUBS.begin()[1], disp_traj_pub) } )
		{
			if ( node == name )
			{
				return pub;
			}
		}
		return planning_scene_pub;
	};
	
	auto make_thread = std::thread([&]()
	{
		auto tic = ros::Time::now();
		while (ros::ok())
		{
			auto toc = ros::Time::now();
			auto dur = toc - tic;
			if ( dur.toSec() > seconds )
				return; 
		}
	}
	);

	make_thread.detach();
}

void 
rovi_planner::moveit_planner::execution(planning_interface::MotionPlanResponse & req, ros::Publisher & pub, const double & tol, const double & period)
{
	ROS_INFO_STREAM("The trajectory is about to be executed");

	trajectory_processing::TimeOptimalTrajectoryGeneration totg( tol, period);

	totg.computeTimeStamps(*req.trajectory_, 0.2, 0.1);

	// create joint trajectory using linear interpolation
	auto joint_states = rovi_utils::joint_states_from_traj(*req.trajectory_);

	ros::Rate lp(1000);

	for(auto joint_msg : joint_states)
	{

		pub.publish(joint_msg);

		lp.sleep();
	}

	ROS_INFO_STREAM("The joint states are sent.");
}

void
rovi_planner::reachability(const std::vector<std::array<double, 3>>& base_pts, const std::array<double, 3>& obj, const std::array<double, 3>& offset, const std::array<double, 3>& axis,
						   ros::Publisher& planning_scene_pub, int resolution, const std::string& obj_name, const std::array<double, 3>& table)
{
	// arm, ee and robot defines
	constexpr auto ARM_GROUP         = "ur5_arm";
	constexpr auto WSG_GROUP         = "wsg";
	constexpr auto ROBOT_DESCRIPTION = "robot_description";

	// load robot model and kinematic model, and use it to setup the planning scene
	robot_model_loader::RobotModelLoaderPtr robot_model_loader(new robot_model_loader::RobotModelLoader(ROBOT_DESCRIPTION));
	robot_model::RobotModelPtr robot_model(robot_model_loader->getModel());

	// set up the planning scene for collision detection
	planning_scene::PlanningScene planning_scene(robot_model);

	// get robot state, this is a raw reference and pointers for arm_group and wsg_group
	auto& robot_state    = planning_scene.getCurrentStateNonConst();
	const auto arm_group = robot_state.getJointModelGroup(ARM_GROUP);
	const auto wsg_group = robot_state.getJointModelGroup(WSG_GROUP);

	// set gripper default
	std::vector gripper_state{0.05, 0.05};
	robot_state.setJointGroupPositions(wsg_group, gripper_state);

	// generate transformations to grasp from, so this should be the input
	auto generate_trans = [](const std::array<double, 3>& pos, const double theta, const std::array<double, 3>& axis)
	{
		// constant transformation
		Eigen::Affine3d trans = Eigen::Translation3d(pos[0], pos[1], pos[2]) *
		                        Eigen::AngleAxisd(theta, Eigen::Vector3d{axis[0], axis[1], axis[2]});

		return trans.matrix();
	};

	Eigen::Matrix4d w_T_obj      = generate_trans( obj,          0, {0, 0, 1} );
	Eigen::Matrix4d obj_T_offset = generate_trans( offset,       0, {0, 0, 1} );
	Eigen::Matrix4d l6_T_ee      = generate_trans( {0, 0.15, 0}, 0, {0, 0, 1} );

	// move the base around
	for (const auto& base_pt : base_pts)
	{
		// make a scene message to update scene
		moveit_msgs::PlanningScene planning_scene_msg;

		// move the base of the robot
		rovi_utils::move_base(robot_state, base_pt);

		// insert table and other constant objects into the planning scene
		std::vector<moveit_msgs::CollisionObject> collision_objects
		{
			rovi_utils::make_mesh_cobj("table",  planning_scene.getPlanningFrame(), table),
			rovi_utils::make_mesh_cobj(obj_name, planning_scene.getPlanningFrame(), obj)
		};

		// how to change color
		// std_msgs::ColorRGBA color;
		// color.a = 1.0;
		// color.b = 0.0;
		// color.g = 0.0;
		// color.r = 1.0;

		// generate the matrices used to calculate the desired pose
		Eigen::Matrix4d w_T_base = generate_trans(base_pt, 0, { 0, 0, 1 });
		{
			// this is only to visualie in RViz
			ros::Rate lp(5);

			for (int i = 0; i < resolution+1; i++)
			{
				// update orientation
				double ori = 2.0 * M_PI / (double)resolution * double(i);

				Eigen::Matrix4d T_rotate = generate_trans({0, 0, 0}, ori, axis);

				// the inverse does always exist in this case
				Eigen::Matrix4d b_T_offset = w_T_base.inverse().matrix() * w_T_obj * obj_T_offset * T_rotate * l6_T_ee.inverse().matrix();

				// calculate the inverse_kinematics to the pose
				auto q_solutions = ur5_dynamics::inv_kin(b_T_offset);

				for (int j = 0; j < q_solutions.rows(); j++)
				{
					// update the robot state
					robot_state.setJointGroupPositions(arm_group, q_solutions.row(j).transpose());

					// get the planning msg
					planning_scene.getPlanningSceneMsg(planning_scene_msg);
					planning_scene_msg.world.collision_objects = collision_objects;
					planning_scene.setPlanningSceneDiffMsg(planning_scene_msg);
					// planning_scene.setObjectColor("table", color);

					// publish
					planning_scene_pub.publish(planning_scene_msg);
					lp.sleep();
				}
			}
		}
	}
}