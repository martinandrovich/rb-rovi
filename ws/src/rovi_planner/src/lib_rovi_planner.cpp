#include "rovi_planner/rovi_planner.h"

#include <tf_conversions/tf_kdl.h>

#include <kdl/frames.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/path_point.hpp>
#include <kdl/path_line.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/trajectory_stationary.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/velocityprofile_spline.hpp>
#include <kdl/utilities/error.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <pluginlib/class_loader.h>
#include <boost/scoped_ptr.hpp>

#include <gazebo_msgs/ModelStates.h>
#include <sensor_msgs/JointState.h>
#include <ur5_dynamics/ur5_dynamics.h>
#include <rovi_utils/rovi_utils.h>

KDL::Trajectory_Composite
rovi_planner::traj_linear(const std::vector<geometry_msgs::Pose>& waypoints, double vel_max, double acc_max, double equiv_radius)
{
	// equivalent radius: serves to compare rotations and translations; the "amount of motion" (pos,vel,acc)
	// of the rotation is taken to be the amount motion of a point at distance eqradius from the rotation axis.
	
	if (waypoints.size() < 2)
		throw std::runtime_error("There must be at least two waypoints.");

	auto traj         = new KDL::Trajectory_Composite();
	auto interpolator = new KDL::RotationalInterpolation_SingleAxis();
	
	// convert vector<Pose> to vector<KDL::Frame>
	std::vector<KDL::Frame> frames;

	// try creating the trajectory; catch any errors
	try
	{
		for (const auto& pt : waypoints)
		{
			static KDL::Frame frame;
			tf::poseMsgToKDL(pt, frame);
			frames.push_back(frame);
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

				ROS_DEBUG_STREAM("path->PathLength(): " << path->PathLength());
				ROS_DEBUG_STREAM("traj_seg->Duration(): " << traj_seg->Duration());
				ROS_DEBUG_STREAM("traj->Duration(): " << traj->Duration());
			}
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

KDL::Trajectory_Composite
rovi_planner::traj_moveit(const geometry_msgs::Pose & pose_des, std::string planner, std::vector<double> q, double vel_max, double acc_max, double corner_radius)
{
	// arm and robot_description
	constexpr auto ARM_GROUP    	 	= "ur5_arm";
	constexpr auto WSG_GROUP 	    	= "wsg";
	constexpr auto ROBOT_DESCRIPTION 	= "robot_description";
	constexpr auto PLANNER_PLUGIN_NAME 	= "ompl_interface/OMPLPlanner";

	// this sets the current planner
	auto check_planner = [planner]()
	{
		for (auto plan : {"RRT", "RRTstar"})
			if ( plan == planner ) return planner;
		
		return std::string("RRTConnect");
	};

	// set the planner
	ros::param::set("/ur5_arm/default_planner_config", check_planner());

	// create the model of the robot
	robot_model_loader::RobotModelLoaderPtr robot_model_loader(new robot_model_loader::RobotModelLoader(ROBOT_DESCRIPTION));
    robot_model::RobotModelPtr 				robot_model(robot_model_loader->getModel());

	// create the planning-scene
    planning_scene::PlanningScenePtr planning_scene( new planning_scene::PlanningScene(robot_model) );

	// get robot state, this is a raw reference and pointers for arm_group and wsg_group
    auto& robot_state    = planning_scene->getCurrentStateNonConst();
    const auto arm_group = robot_state.getJointModelGroup(ARM_GROUP);
    const auto wsg_group = robot_state.getJointModelGroup(WSG_GROUP);

	// set the robot_state to the current one
	if (q.size() != 6 )
	{
		auto q_gazebo 	= ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");
		q 				= std::vector<double>(q_gazebo->position.begin(), 
											  q_gazebo->position.end()-2);
		Eigen::Matrix<double, 6, 1> mat(q.data());
		ROS_INFO_STREAM_ONCE("q: " << mat);
	}

	// set the robot state
	robot_state.setJointGroupPositions(ARM_GROUP, q);
	robot_state.setJointGroupPositions(WSG_GROUP, std::vector({0.05, 0.05}));

	// move the base 
	auto model_states = ros::topic::waitForMessage<gazebo_msgs::ModelStates>("/gazebo/model_states");
	for(int i = 0; model_states->name.size() > i ; ++i)
	{
		if(model_states->name[i] == "ur5")
		{ 
			rovi_utils::move_base(robot_state, 
									{
										model_states->pose[i].position.x, 
										model_states->pose[i].position.y, 
										model_states->pose[i].position.z
									}
								  );
		}
	}

	// add the collisions to the scene
	std::vector<moveit_msgs::CollisionObject> collision_objects = 
		rovi_utils::get_gazebo_obj(planning_scene->getPlanningFrame());
	
	// add the collision objects
	moveit_msgs::PlanningScene planning_scene_msg;
	planning_scene->getPlanningSceneMsg(planning_scene_msg);
	planning_scene_msg.world.collision_objects = collision_objects;
	planning_scene->setPlanningSceneDiffMsg(planning_scene_msg);

	// setup the runtime plugin
	boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
	planning_interface::PlannerManagerPtr planner_instance;
	planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
	planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(PLANNER_PLUGIN_NAME));

	// check for success
	if (!planner_instance->initialize(robot_model, ros::this_node::getNamespace()))
	{	
    	ROS_FATAL_STREAM("Could not initialize planner instance");
		// do some error handling here
	}

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
	req.goal_constraints.push_back(pose_goal);

	// get the planningcontext
	planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
	context->solve(res);

	if (res.error_code_.val != res.error_code_.SUCCESS)
	{
		// do some error handling here
	}

	// export waypoints to file
	rovi_utils::export_traj(*res.trajectory_, "rrt_way.csv");

	// create trajectory using linear interpolation + export to file
	auto waypoints = rovi_utils::waypoints_from_traj(*res.trajectory_);
	auto traj_lin = rovi_planner::traj_parabolic(waypoints, 0.1, 0.1, 0.001, 0.001);
	rovi_utils::export_traj(traj_lin, "rrt_linear.csv");

	return traj_lin;
}

void 
rovi_planner::reachability( const std::vector<std::array<double, 3>>& base_pts, const std::array<double, 3>& obj, const std::array<double, 3>& offset, const std::array<double, 3>& axis,
							ros::Publisher& planning_scene_pub, int resolution, const std::string& obj_name, const std::array<double, 3>& table)
{
    // arm, ee and robot defines
	constexpr auto ARM_GROUP    	 	= "ur5_arm";
	constexpr auto WSG_GROUP 	    	= "wsg";
	constexpr auto ROBOT_DESCRIPTION 	= "robot_description";

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
                                Eigen::AngleAxisd(theta, Eigen::Vector3d{axis[0],axis[1],axis[2]});
        
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
            rovi_utils::make_mesh_cobj("table",  planning_scene.getPlanningFrame() , table),
            rovi_utils::make_mesh_cobj(obj_name, planning_scene.getPlanningFrame() , obj)
        };

        // how to change color
        // std_msgs::ColorRGBA color;
        // color.a = 1.0;
        // color.b = 0.0;
        // color.g = 0.0;
        // color.r = 1.0;

        // generate the matrices used to calculate the desired pose
        Eigen::Matrix4d w_T_base = generate_trans( base_pt,      0, {0, 0, 1} );
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