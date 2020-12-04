#include "rovi_planner/rovi_planner.h"

#include <tf_conversions/tf_kdl.h>

#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/trajectory_processing/trajectory_tools.h>

#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/ModelStates.h>
#include <moveit_msgs/DisplayTrajectory.h>

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

#include <rovi_utils/rovi_utils.h>
#include <rovi_gazebo/rovi_gazebo.h>
#include <ur5_dynamics/ur5_dynamics.h>

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
rovi_planner::moveit_planner::init(ros::NodeHandle& nh)
{
	// setup async spinners for moveit
	ros::AsyncSpinner spin(0);
	spin.start();

	// create the model of the robot
	robot_model_loader = std::make_shared<robot_model_loader::RobotModelLoader>(ROBOT_DESCRIPTION);
	robot_model = robot_model_loader->getModel();

	// create planning scene
	planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model);

	// create planner objects
	planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
	planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(PLANNER_PLUGIN_NAME));

	// create publishers
	pub_planning_scene = nh.advertise<moveit_msgs::PlanningScene>(PLANNING_SCENE_TOPIC, 1);
	pub_traj           = nh.advertise<moveit_msgs::DisplayTrajectory>(TRAJECTORY_TOPIC, 1);

	// check for success
	if (!planner_instance->initialize(robot_model, ros::this_node::getNamespace()))
	{
		// do some error handling here
		ROS_FATAL_STREAM("Could not initialize planner instance...");
		is_init = false;
	}
	else
		is_init = true;

	return is_init;
}

void
rovi_planner::moveit_planner::destruct()
{
	planner_instance->terminate();
}

void
rovi_planner::moveit_planner::update_planning_scene()
{
	// lock planning scene mutex for this scope	
	// std::lock_guard lock(mtx_planning_scene);
	mtx_planning_scene.lock();

	// get robot state and joint groups
	auto& robot_state    = planning_scene->getCurrentStateNonConst();
	const auto arm_group = robot_state.getJointModelGroup(ARM_GROUP);
	const auto wsg_group = robot_state.getJointModelGroup(WSG_GROUP);

	// set the robot arm state to the current one from Gazebo
	const auto q_ur5 = rovi_gazebo::get_current_robot_state().position;
	const auto q_wsg = rovi_gazebo::get_current_gripper_state().position;
	robot_state.setJointGroupPositions(ARM_GROUP, q_ur5);
	// robot_state.setJointGroupPositions(WSG_GROUP, q_wsg);
	robot_state.setJointGroupPositions(WSG_GROUP, std::vector{ 0.05, 0.05 });
	
	// set robot base position to the current one from Gazebo
	const auto base_pose = rovi_gazebo::get_current_base_pose();
	rovi_utils::move_base(robot_state, base_pose);
	
	// add the collisions to the scene and remove any attached objects
	collision_objects = rovi_gazebo::get_collision_objects(planning_scene->getPlanningFrame());
	
	planning_scene->getPlanningSceneMsg(planning_scene_msg);
	planning_scene_msg.robot_state.attached_collision_objects = {};
	planning_scene_msg.world.collision_objects = collision_objects;
	planning_scene->setPlanningSceneMsg(planning_scene_msg);
	
	mtx_planning_scene.unlock();
	ros::Duration(1).sleep();
}

bool
rovi_planner::moveit_planner::attach_object_to_ee(const std::string& obj_name)
{
	// lock planning scene mutex for this scope
	mtx_planning_scene.lock();

	// check if collision objects exists (find index)
	const auto cobj = std::find_if(collision_objects.begin(), collision_objects.end(), [&](auto& obj) {
		return obj.id == obj_name;
	});

	if (cobj == collision_objects.end())
	{
		ROS_WARN_STREAM("Collision object '" << obj_name << "' does not exist.");
		return false;
	}

	// get current planning scence (msg)
	planning_scene->getPlanningSceneMsg(planning_scene_msg);

	// attach the object to the end-effector
	// offset object a little bit to abvoid collisions
	constexpr auto offset_z = 0.005;
	moveit_msgs::AttachedCollisionObject acobj;
	acobj.link_name = "ee_tcp";
	acobj.object = moveit_msgs::CollisionObject(*cobj);
	// acobj.object.pose.position.z += offset_z;
	acobj.object.mesh_poses[0].position.z += offset_z;
	acobj.object.operation = moveit_msgs::CollisionObject::ADD;
	
	planning_scene_msg.robot_state.attached_collision_objects = { acobj };

	// remove object from scene
	cobj->operation = moveit_msgs::CollisionObject::REMOVE;
	planning_scene_msg.world.collision_objects = collision_objects;
	
	// update robot position
	auto& robot_state = planning_scene->getCurrentStateNonConst();
	const auto q_ur5 = rovi_gazebo::get_current_robot_state().position;
	robot_state.setJointGroupPositions(ARM_GROUP, q_ur5);

	// update planning scene
	planning_scene->setPlanningSceneMsg(planning_scene_msg);
	
	mtx_planning_scene.unlock();
	ros::Duration(1).sleep();

	return true;
}

planning_interface::MotionPlanResponse
rovi_planner::moveit_planner::plan(const geometry_msgs::Pose& pose_des, const std::string& planner, std::vector<double> q)
{
	// lock planning scene mutex for this scope
	std::lock_guard lock(mtx_planning_scene);

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
	req.allowed_planning_time = 5;
	req.num_planning_attempts = 10000;

	// scale factor velocity / acceleration
	req.max_acceleration_scaling_factor = 1.0;
	req.max_velocity_scaling_factor = 1.0;

	// pushback the requirements
	req.goal_constraints.push_back(pose_goal);

	// get the planningcontext
	auto context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
	context->solve(res);

	if (res.error_code_.val != res.error_code_.SUCCESS)
	{
		// do some error handling here
		ROS_FATAL_STREAM("It was not possible to generate a trajectory... exiting...");
		exit(-1);
	}

	// return the plan
	return res;
}

std::vector<sensor_msgs::JointState>
rovi_planner::moveit_planner::get_traj(planning_interface::MotionPlanResponse& plan, double tol, double dt, double max_vel_scale, double max_acc_scale)
{
	trajectory_processing::TimeOptimalTrajectoryGeneration totg(tol, dt);
	totg.computeTimeStamps(*plan.trajectory_, max_vel_scale, max_acc_scale);

	return rovi_utils::joint_states_from_traj(*plan.trajectory_);
}

void
rovi_planner::moveit_planner::start_planning_scene_publisher()
{
	if (not thread_planning_scene_pub)
	{
		ROS_WARN_STREAM("Updating planning scene and initializing planning scene publisher thread (once)...");

		// update_planning_scene();

		thread_planning_scene_pub = new std::thread([&]()
		{
			ros::Rate lp(100); // Hz
			while(ros::ok())
			{
				mtx_planning_scene.lock();
				// planning_scene->setPlanningSceneMsg(planning_scene_msg);
				planning_scene->getPlanningSceneMsg(planning_scene_msg);
				pub_planning_scene.publish(planning_scene_msg);
				mtx_planning_scene.unlock();
				lp.sleep();
			}
		});
		
		thread_planning_scene_pub->detach();
	}

	else
		ROS_ERROR_STREAM("Planning scene publisher thread is already active.");
}

rovi_planner::moveit_planner::ReachabilityData
rovi_planner::moveit_planner::reachability(const std::array<double, 3>& base_pos, const std::string& obj_name, const std::array<double, 3>& obj_pos, const std::array<double, 3>& offset, const std::array<double, 3>& axis, size_t resolution, bool visualize)
{

	// check
	if (visualize)
		ROS_WARN_ONCE("To visualize the planning scene, remember to call start_planning_scene_publisher() beforehand!");
	
	// get robot state, this is a raw reference and pointers for arm_group and wsg_group
	mtx_planning_scene.lock();
	auto& robot_state    = planning_scene->getCurrentStateNonConst();
	const auto arm_group = robot_state.getJointModelGroup(ARM_GROUP);
	const auto wsg_group = robot_state.getJointModelGroup(WSG_GROUP);
	
	// set gripper default
	robot_state.setJointGroupPositions(wsg_group, DEFAULT_WSG_STATE);
	
	// generate transformations to grasp from, so this should be the input
	auto make_tf = [](const std::array<double, 3>& pos, const double theta, const std::array<double, 3>& axis)
	{
		// constant transformation
		Eigen::Affine3d trans = Eigen::Translation3d(pos[0], pos[1], pos[2]) *
		                        Eigen::AngleAxisd(theta, Eigen::Vector3d{axis[0], axis[1], axis[2]});

		return trans.matrix();
	};
	
	// collision data
	ReachabilityData data
		= { base_pos, resolution, 2.0 * M_PI / (double)resolution, 0U, 0U, 0U, 0.0 };
	
	// generate transformations
	Eigen::Matrix4d w_T_obj      = make_tf(obj_pos,        0, { 0, 0, 1 });
	Eigen::Matrix4d obj_T_offset = make_tf(offset,         0, { 0, 0, 1 });
	Eigen::Matrix4d l6_T_ee      = make_tf({ 0, 0.15, 0 }, 0, { 0, 0, 1 });
	Eigen::Matrix4d w_T_base     = make_tf(base_pos,       0, { 0, 0, 1 });
	
	// move the base of the robot
	rovi_utils::move_base(robot_state, base_pos);
	
	// insert table and other constant objects into the planning scene
	std::vector<moveit_msgs::CollisionObject> collision_objects
	{
		rovi_utils::make_mesh_cobj("table",  planning_scene->getPlanningFrame(), rovi_gazebo::TABLE.POS),
		rovi_utils::make_mesh_cobj(obj_name, planning_scene->getPlanningFrame(), obj_pos),
	};
	
	// update planning scene
	planning_scene->getPlanningSceneMsg(planning_scene_msg);
	planning_scene_msg.world.collision_objects = collision_objects;
	planning_scene->setPlanningSceneMsg(planning_scene_msg);
	
	mtx_planning_scene.unlock();
	
	// iterate all possible orientations (given the resolution) and possible solutions
	for (size_t i = 0; i < resolution + 1 and ros::ok(); ++i)
	{
		// update orientation
		const double theta = 2.0 * M_PI / (double)resolution * double(i);

		// compute transformations
		Eigen::Matrix4d T_rotate = make_tf({ 0, 0, 0 }, theta, axis);
		Eigen::Matrix4d b_T_offset = w_T_base.inverse().matrix() * w_T_obj * obj_T_offset * T_rotate * l6_T_ee.inverse().matrix();

		// calculate the inverse_kinematics to the pose
		const auto q_solutions = ur5_dynamics::inv_kin(b_T_offset);
		
		// iterate all possible solutions
		for (size_t j = 0; j < q_solutions.rows(); ++j)
		{
			// update the robot state
			mtx_planning_scene.lock();
			robot_state.setJointGroupPositions(arm_group, q_solutions.row(j).transpose());
			
			// do collision test
			collision_detection::CollisionRequest col_req;
			collision_detection::CollisionResult col_res;
			planning_scene->checkCollision(col_req, col_res);
			
			data.iterations++;
			data.collisions += col_res.collision ? 1 : 0;
			
			mtx_planning_scene.unlock();
			
			if (visualize)
			{
				static ros::Rate lp(5); // Hz
				ROS_INFO_STREAM("Collision test:  " << i << "/" << (resolution + 1) << ", angle: " << theta);
				ROS_INFO_STREAM("Current state is " << (col_res.collision ? "in" : "not in") << " collision");
				lp.sleep();
			}
		}
	}

	// compute return reachability data
	data.ratio = data.collisions / (double)data.iterations;
	data.plausible_states = data.iterations - data.collisions;
	
	return data;
}