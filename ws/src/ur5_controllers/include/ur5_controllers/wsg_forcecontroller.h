#pragma once

#include <string>
#include <vector>

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/ros.h>
#include <urdf/model.h>

#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <ur5_controllers/PoseTwist.h>

#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <visualization_msgs/Marker.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ur5_controllers
{

class WSGForceController final
: public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
    public:

    WSGForceController(){}
    ~WSGForceController(){}
    
    private:
    


}

}
