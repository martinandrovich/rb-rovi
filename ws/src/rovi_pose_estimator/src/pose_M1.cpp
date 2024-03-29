#include <ros/ros.h>
#include <ros/package.h>
#include <Eigen/Eigen>
#include <cv_bridge/cv_bridge.h>
#include <rovi_gazebo/rovi_gazebo.h>
#include <rovi_utils/rovi_utils.h>
#include <eigen_conversions/eigen_msg.h>

#include <rovi_pose_estimator/rovi_pose_estimator.h>

const static std::vector<std::array<double, 3>> OBJECT_POS{
                                                                {0.55, 0.95, 0.75},
                                                                {0.55, 1.00, 0.75},
                                                                {0.55, 1.05, 0.75},
                                                                {0.60, 0.95, 0.75},
                                                                {0.60, 1.00, 0.75},
                                                                {0.60, 1.05, 0.75},
                                                            };


const static std::vector<std::array<double, 3>> OBJECT_ORI{
                                                                {0.0, 0.0, 0.0},
                                                                {0.0, 0.0, 0.0},
                                                                {0.0, 0.0, 0.0},
                                                                {0.0, 0.0, 0.0},
                                                                {0.0, 0.0, 0.0},
                                                                {0.0, 0.0, 0.0}
                                                            };

const static std::vector<double> OBJECT_NOISE               {
                                                                0,
                                                                10,
                                                                20,
                                                                30,
                                                                40,
                                                                50
                                                            };

int main(int argc, char** argv)
{
    using namespace rovi_pose_estimator;

    if ( argc != 2 )
    {
        ROS_INFO_STREAM("How to use, insert number of it : <it>");
        return -1;
    }

    std::fstream file_handle("rovi_pose_M1.csv", std::ios_base::out);

    if( not file_handle.is_open() )
    {
        ROS_INFO_STREAM("File is not open, closing.");
        return -1;
    }

    // set
    int iterations = std::stoi(argv[1]);

    // init the node
	ros::init(argc, argv, "pose_M1");
	ros::NodeHandle nh;
	ros::AsyncSpinner spin(4);
	spin.start();  

    // init
    rovi_gazebo::set_projector(true);
    rovi_gazebo::set_simulation(true);
    rovi_gazebo::spawn_model("bottle", "bottle1", OBJECT_POS[0], OBJECT_ORI[0]);

    geometry_msgs::Pose guess;
    
    for (auto i = 0; i < OBJECT_ORI.size(); ++i )
    {

        rovi_gazebo::move_model("bottle1", OBJECT_POS[i], OBJECT_ORI[i]);

        ros::Duration(1).sleep();

        for (auto k = 0; k < OBJECT_NOISE.size(); k++)
        {

            for (auto j = 0; j < 20; j++)
            {
                auto tic = ros::Time::now();

                guess = M1::estimate_pose(iterations, false, OBJECT_NOISE[k]);

                geometry_msgs::Pose actual = rovi_utils::make_pose(OBJECT_POS[i], OBJECT_ORI[i]);

                auto toc = ros::Time::now();

                file_handle << (toc-tic).toSec() << ", " 
                            << actual.position.x << ", " 
                            << actual.position.y << ", " 
                            << actual.position.z << ", "
                            << guess.position.x << ", " 
                            << guess.position.y << ", " 
                            << guess.position.z << ", " 
                            << OBJECT_NOISE[k] << std::endl;
            }

        }

    }

    file_handle.close();

    return 0;
}