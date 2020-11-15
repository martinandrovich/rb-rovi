#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ur5_dynamics/ur5_dynamics.h>
#include <rovi_utils/rovi_utils.h>

#include <Eigen/Dense>

const std::string PLANNING_SCENE_TOPIC      = "planning_scene_2";


void reachability(
                       const std::vector<std::array<double, 3>>& base_pts, 
                       const std::array<double, 3>& obj,
                       const std::array<double, 3>& offset,
                       const std::array<double, 3>& axis,
                       ros::Publisher& planning_scene_pub,
                       int resolution = 16, 
                       const std::string& obj_name = "bottle",
                       const std::array<double, 3>& table = {0.4, 0.6, 0.64}
                       );

int
main(int argc, char** argv)
{	
	// init node
	ros::init(argc, argv, "reachability");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(1);
	spinner.start();

    // publish the planning_scene for visualization in rviz
    ros::Publisher planning_scene_pub = nh.advertise<moveit_msgs::PlanningScene>(PLANNING_SCENE_TOPIC, 1);

    // loop_rate
    {
        ros::Rate lp(0.5);
        while( planning_scene_pub.getNumSubscribers() < 1)
        {
            ROS_WARN_STREAM_ONCE("Please connect to the planning_scene: " << PLANNING_SCENE_TOPIC << " in RViz.");
            ROS_WARN_STREAM_ONCE("The Reachability test node will start afterwards.");
            lp.sleep();
        }
    }

    // perform the reachability test
    reachability(  

                // base_pts
                {
                    {0.1, 0.4, 0.75},
                    {0.1, 0.5, 0.75},
                    {0.1, 0.6, 0.75},
                },

                // obj_pts
                {0.5, 1.0, 0.75},

                // offset
                {0.0, 0.0, 0.10},

                // objects axis
                {0.0, 0.0, 1.0},

                // Nodehandle for visualisation
                planning_scene_pub,

                // resolution divides 360 into resolution 16, i.e. 360/16
                180

                );

   return 0;
}

void reachability(
                       const std::vector<std::array<double, 3>>& base_pts, 
                       const std::array<double, 3>& obj,
                       const std::array<double, 3>& offset,
                       const std::array<double, 3>& axis,
                       ros::Publisher& planning_scene_pub,
                       int resolution,
                       const std::string& obj_name,
                       const std::array<double, 3>& table
                       )
{
    // arm, ee and robot defines
    const std::string ARM_GROUP                 = "ur5_arm";
    const std::string WSG_GROUP                 = "wsg";
    const std::string ROBOT_DESCRIPTION         = "robot_description";

    // setting up visulisation tool for RViz
    moveit_visual_tools::MoveItVisualTools visual_tools("ur5_link0");
    visual_tools.deleteAllMarkers();

    // load robot model and kinematic model, and use it to setup the planning scene
    robot_model_loader::RobotModelLoaderPtr robot_model_loader(new robot_model_loader::RobotModelLoader(ROBOT_DESCRIPTION));
    robot_model::RobotModelPtr robot_kinematic_model(robot_model_loader->getModel());

    // set up the planning scene for collision detection
    planning_scene::PlanningScene planning_scene(robot_kinematic_model);

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
        std_msgs::ColorRGBA color;
        color.a = 1.0;
        color.b = 0.0;
        color.g = 0.0;
        color.r = 1.0;

        // generate the matrices used to calculate the desired pose
        Eigen::Matrix4d w_T_base = generate_trans( base_pt,      0, {0, 0, 1} );
        {
            // this is only to visualie in RViz
            ros::Rate lp(2); 

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
                    planning_scene.setObjectColor("table", color);
                    
                    // publish
                    planning_scene_pub.publish(planning_scene_msg);
                    lp.sleep();
                }

            }

        }

    }
    
}