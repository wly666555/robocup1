#include <iostream>
#include <stdio.h>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include "yaml_parser.h"
#include "dxl.h"
#include "utilities.h"

int main(int argc, char** argv)
{
    param::helper(argc, argv);
    auto joints = dxl::Motors<2>(57600);

    YamlParser yamlParam_;
    std::string env_cfg_path = "/home/unitree/g1_comp_servo_service/config/config2.yaml";
    yamlParam_.setup(env_cfg_path.c_str());

    float servo0_calibration = yamlParam_.ReadFloatFromYaml("servo0_calibration");
    float servo1_calibration = yamlParam_.ReadFloatFromYaml("servo1_calibration");

    Eigen::VectorXf joint0_limitation(2);
    joint0_limitation = yamlParam_.ReadVectorFromYaml("joint0",2);

    Eigen::VectorXf joint1_limitation(2);
    joint1_limitation = yamlParam_.ReadVectorFromYaml("joint1",2);

    float servo0_limit_enoder = yamlParam_.ReadFloatFromYaml("servo0_calibration");
    float servo1_limit_enoder = yamlParam_.ReadFloatFromYaml("servo1_calibration");

    Eigen::VectorXf direction(2);
    direction = yamlParam_.ReadVectorFromYaml("direction",2);

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
        float joint0_servo_angle = utilities::encoder2angle(joints.present_position[0],servo0_limit_enoder,joint0_limitation,direction(0));
        float joint1_servo_angle = utilities::encoder2angle(joints.present_position[1],servo1_limit_enoder,joint1_limitation,direction(1));
        std::cout<<"joint0_servo_angle : "<<joint0_servo_angle<<std::endl;
        std::cout<<"joint1_servo_angle : "<<joint1_servo_angle<<std::endl;
        sleep(0.2);
    }

    return 0;
}