#include <interpolator/ur5_interpolator.h>

void ur5_interp::rounded_composite()
{
    //KDL::RotationalInterpolation

    KDL::Path_Composite x;

    KDL::RotationalInterpolation_SingleAxis rotint;

    std::cout << "Created object rotint" << std::endl;

    KDL::Rotation r_start(  
                            KDL::Vector(1, 0, 0),
                            KDL::Vector(0, 1, 0),
                            KDL::Vector(0, 0, 1) 
                        );
    
    KDL::Rotation r_end(    
                            KDL::Vector(0, -1, 0),
                            KDL::Vector(1, 0, 0),
                            KDL::Vector(0, 0, 1)
                        );

    KDL::Vector t_start(1, 0, 0);
    KDL::Vector t_end(2, 0, 0);

    rotint.SetStartEnd(r_start, r_end);

    KDL::Frame frame_start(r_start, t_start);
    KDL::Frame frame_end(r_end, t_end);

    KDL::Path_Line line(frame_start, frame_end, &rotint, 0.1f, false);

    std::cout << line.Acc(1, 1, 15) << std::endl;
    std::cout << line.LengthToS(1) << std::endl;

    /*

    KDL::Path_RoundedComposite rounted(1, 0.3f, &rotint);

    std::cout << "Added rounted" << std::endl;

    x.Add(&rounted);
    
    x.Finish();

    */


    //KDL::Path_RoundedComposite 
}