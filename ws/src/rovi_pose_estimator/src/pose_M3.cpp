#include <ros/ros.h>

#include <iostream>
#include <fstream>

#include <Eigen/Eigen>
#include <cv_bridge/cv_bridge.h>

#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/stereo/matching.hpp>
#include <opencv4/opencv2/ml.hpp>
#include <opencv4/opencv2/core/eigen.hpp>

#include <eigen_conversions/eigen_msg.h>

#include <rovi_utils/rovi_utils.h>
#include <rovi_gazebo/rovi_gazebo.h>
#include <rovi_pose_estimator/rovi_pose_estimator.h>

const static std::vector<std::array<double, 3>> OBJECT_POS{
                                                                {0.22, 1.02, 0.79},
                                                                {0.27, 1.03, 0.79},
                                                                {0.33, 1.05, 0.79},
                                                                {0.44, 1.03, 0.79},
                                                                {0.57, 0.98, 0.79},
                                                                {0.63, 1.01, 0.79}
                                                            };


const static std::vector<std::array<double, 3>> OBJECT_ORI{
                                                                {0.0, 0., 0.15},
                                                                {0.0, 0., 0.22},
                                                                {0.0, 0., 0.35},
                                                                {0.0, 0., 0.44},
                                                                {0.0, 0., 0.53},
                                                                {0.0, 0., 0.62}
                                                            };

const static std::vector<double> OBJECT_NOISE               {
                                                                3*3
                                                            };

int main(int argc, char** argv)
{
    using namespace rovi_pose_estimator;

    std::fstream file_handle("rovi_pose.csv", std::ios_base::out);

    if( not file_handle.is_open() )
    {
        ROS_INFO_STREAM("File is not open, closing.");
        return -1;
    }

    float noise = std::stof(argv[1]);

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
    
    for (int i = 0; i < OBJECT_POS.size(); ++i )
    {

        for (int j = 0; j < OBJECT_ORI.size(); ++j )
        {

            rovi_gazebo::move_model("cube1", OBJECT_POS[i], OBJECT_ORI[j]);

            ros::Duration(1).sleep();

            //for (int k = 0; k < OBJECT_NOISE.size(); ++k)
            //{

            for(int z = 0; z < 2; ++z)
            {
                auto file_name = std::string("pose_M3_") + std::to_string(i) + std::to_string(j) + std::string(".jpg");

                //rovi_gazebo::move_model("cube1", OBJECT_POS[i], OBJECT_ORI[i]);

                ros::Duration(1).sleep();

                geometry_msgs::Pose guess = M3::estimate_pose(1, file_name, 0);

                geometry_msgs::Pose actual = rovi_utils::make_pose(OBJECT_POS[i], OBJECT_ORI[j]);

                file_handle << 0 << ", " << actual.position.x << ", " << actual.position.y << ", " << actual.position.z << ", " 
                            << actual.orientation.x << ", "<< actual.orientation.y << ", "<< actual.orientation.z << ", " << actual.orientation.w << ", ";

                file_handle << guess.position.x << ", " << guess.position.y << ", " << guess.position.z << ", " 
                            << guess.orientation.x << ", "<< guess.orientation.y << ", "<< guess.orientation.z << ", " << guess.orientation.w << std::endl;
                
            }

        }
    }
    
    file_handle.close();
    
    return 0;
}