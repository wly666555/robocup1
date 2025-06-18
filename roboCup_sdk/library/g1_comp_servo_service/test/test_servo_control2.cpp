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

    int desired_position = -88;
    joints.set_position(id, desired_position);
    return 0;
}