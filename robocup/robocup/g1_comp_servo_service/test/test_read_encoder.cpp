#include "dxl.h"

int main(int argc, char** argv)
{
    param::helper(argc, argv);
    auto joints = dxl::Motors<2>(1000000);

    for(int id=0;id<2;id++)
    {
        joints.enable(id);
        joints.set_position_p_gain(id,0);
        joints.set_position_d_gain(id,0);
    }

    while(true)
    {   
        std::cout<<std::endl;
        joints.sync_get_position();
        std::cout<<"servo 0 position : "<<joints.present_position[0]<<std::endl;
        std::cout<<"servo 1 position : "<<joints.present_position[1]<<std::endl;
        sleep(0.2);
    }

    return 0;
}