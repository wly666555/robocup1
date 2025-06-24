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
    auto joints = dxl::Motors<1>(57600);

    int id = 0;


    YamlParser yamlParam_;
    std::string env_cfg_path = "/home/unitree/g1_comp_servo_service/config/config2.yaml";
    yamlParam_.setup(env_cfg_path.c_str());

    float servo_limit_enoder = yamlParam_.ReadFloatFromYaml("servo0_calibration");

    Eigen::VectorXf joint0_limitation(2);
    joint0_limitation = yamlParam_.ReadVectorFromYaml("joint0",2);

    Eigen::VectorXf direction(2);
    direction = yamlParam_.ReadVectorFromYaml("direction",2);

    float desired_angle = 0;
    printf("Enter the desired angle: ");
    scanf("%f", &desired_angle);

    joints.enable(id);
    joints.set_position_p_gain(id,500);
    joints.set_position_d_gain(id,300);
    float joint0_servo_positon = utilities::angle2encoder(desired_angle,servo_limit_enoder,joint0_limitation,direction(0));
    joints.set_position(id, joint0_servo_positon);

}

