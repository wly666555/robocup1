#include <iostream>
#include "yaml_parser.h"

int main()
{
    YamlParser yamlParam_;
    std::string env_cfg_path = "/home/unitree/g1_comp_servo_service/config/config.yaml";
    yamlParam_.setup(env_cfg_path.c_str());

    float  dt = yamlParam_.ReadFloatFromYaml("dt");
    std::cout<<"dt: "<<dt<<std::endl;
}