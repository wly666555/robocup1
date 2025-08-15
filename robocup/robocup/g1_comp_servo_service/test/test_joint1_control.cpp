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
    auto joints = dxl::Motors<1>(1000000);

    YamlParser yamlParam_;
    std::string env_cfg_path = "/home/unitree/g1_comp_servo_service/config/config.yaml";
    yamlParam_.setup(env_cfg_path.c_str());

    float has_calibrate = yamlParam_.ReadFloatFromYaml("has_calibrate");
    if(!has_calibrate) 
    {
        std::cout<<std::endl;
        std::cout<<" please run test_calibration first! "<<std::endl;
        exit(-1);
    }

    float servo1_calibration = yamlParam_.ReadFloatFromYaml("servo1_calibration");

    Eigen::VectorXf joint1_limitation(2);
    joint1_limitation = yamlParam_.ReadVectorFromYaml("joint1",2);

    Eigen::VectorXf direction(2);
    direction = yamlParam_.ReadVectorFromYaml("direction",2);

    float desired_angle = 0;
    printf("Enter the desired angle: ");
    scanf("%f", &desired_angle);

    int id = 1;
    joints.enable(id);
    joints.set_position_p_gain(id,500);
    joints.set_position_d_gain(id,300);
    float joint1_servo_positon = utilities::angle2encoder(desired_angle,servo1_calibration,joint1_limitation,direction(1));
    joints.set_position(id, joint1_servo_positon);
}

