#include <ros/ros.h>
#include <rovi_pose_estimator/rovi_pose_est.h>
#include <rovi_gazebo/rovi_gazebo.h>
#include <rovi_utils/rovi_utils.h>
#include <Eigen/Eigen>
#include <cv_bridge/cv_bridge.h>

#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/stereo/matching.hpp>
#include <opencv4/opencv2/ml.hpp>
#include <opencv4/opencv2/core/eigen.hpp>

#include <eigen_conversions/eigen_msg.h>
#include <iostream>
#include <fstream>

const static std::vector<std::array<double, 3>> OBJECT_POS{
                                                                {0.1, 1, 0.79},
                                                                {0.2, 1.03, 0.79},
                                                                {0.3, 1.05, 0.79},
                                                                {0.4, 1.03, 0.79},
                                                                {0.5, 0.98, 0.79},
                                                                {0.6, 1.01, 0.79}
                                                            };


const static std::vector<std::array<double, 3>> OBJECT_ORI{
                                                                {0.0, 0., 0.5},
                                                                {0.0, 0., 0.4},
                                                                {0.0, 0., 0.9},
                                                                {0.0, 0., 0.2},
                                                                {0.0, 0., 0.3},
                                                                {0.0, 0., 0.6}
                                                            };


int main(int argc, char** argv)
{
    using namespace rovi_pose_estimator;

    std::fstream file_handle("rovi_pose.csv", std::ios_base::out);

    if(argc != 2 ) 
        return -1;

    float noise = std::stof(argv[1]);

    if( not file_handle.is_open() )
    {
        ROS_INFO_STREAM("File is not open, closing.");
        return -1;
    }

    if (OBJECT_POS.size() != OBJECT_ORI.size())
    {
        ROS_INFO_STREAM("Object pos and ori is snot the same");
        return -1;
    }

    // init the node
	ros::init(argc, argv, "pose_M1");
	ros::NodeHandle nh;
	ros::AsyncSpinner spin(2);
	spin.start();  

    // init
    rovi_gazebo::set_projector(false);
    rovi_gazebo::set_simulation(true);
    rovi_gazebo::spawn_model("cube", "cube1", OBJECT_POS[0], OBJECT_ORI[0]);

    ros::Duration(1).sleep();

    file_handle << "actual_pose vs detected" << std::endl;
    
    for (int i = 0; i < OBJECT_ORI.size(); ++i )
    {
        auto file_name = std::string("pose_M3_") + std::to_string(i) + std::string(".jpg");
        rovi_gazebo::move_model("cube1", OBJECT_POS[i], OBJECT_ORI[i]);
        ros::Duration(2).sleep();
        geometry_msgs::Pose guess = M3::estimate_pose(1, file_name, (double)noise);
        geometry_msgs::Pose actual = rovi_utils::make_pose(OBJECT_POS[i], OBJECT_ORI[i]);

        file_handle << actual.position.x << ", " << actual.position.y << ", " << actual.position.z << ", " 
                    << actual.orientation.x << ", "<< actual.orientation.y << ", "<< actual.orientation.z << ", " << actual.orientation.w << ", ";
        file_handle << guess.position.x << ", " << guess.position.y << ", " << guess.position.z << ", " 
                    << guess.orientation.x << ", "<< guess.orientation.y << ", "<< guess.orientation.z << ", " << guess.orientation.w << std::endl;
    }
    
    file_handle.close();
    
    return 0;
}