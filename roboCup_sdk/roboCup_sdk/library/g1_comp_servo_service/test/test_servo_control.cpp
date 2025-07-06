#include "dxl.h"
#include <unistd.h> // For sleep()
#include <cmath> // For sin and M_PI

int main(int argc, char** argv)
{
    param::helper(argc, argv);
    auto joints = dxl::Motors<1>(57600);

    int id = 0;
    joints.enable(id);
    joints.set_position_p_gain(id,500);
    joints.set_position_d_gain(id,300);

    int32_t position = 0;
    joints.get_position(id, (uint32_t*)&position);
    std::cout<<"debug position: "<<position<<std::endl;

    int counter = 0;
    const int amplitude = 100; // Amplitude of the sine wave
    const int frequency = 4; // Frequency of the sine wave

    while(true)
    {
        counter++;
        int desired_position =  amplitude * sin(2 * M_PI * frequency * counter / 500.0) + position;
        std::cout<<"debug desired_position: "<<desired_position<<std::endl;
        joints.set_position(id, desired_position);
        usleep(10000); 
    }
    return 0;
}