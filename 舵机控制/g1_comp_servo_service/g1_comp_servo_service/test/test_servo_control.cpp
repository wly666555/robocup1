#include <iostream>
#include <stdio.h>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include "yaml_parser.h"
#include "dxl.h"
#include "utilities.h"

#include <eigen3/Eigen/Dense>

// 必须要先设置好 kp kd ， 如果是为0，则会抖

int main(int argc, char** argv)
{
    param::helper(argc, argv);
    auto joints = dxl::Motors<2>(1000000);

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
        joints.set_position_p_gain(id,500);
        joints.set_position_d_gain(id,300);
    }

    joints.sync_get_position();
    float joint0_servo_angle_init = utilities::encoder2angle(joints.present_position[0],servo0_limit_enoder,joint0_limitation,direction(0));
    float joint1_servo_angle_init = utilities::encoder2angle(joints.present_position[1],servo1_limit_enoder,joint1_limitation,direction(1));
    std::cout<<"debug joint0_servo_angle_init: "<<joint0_servo_angle_init<<std::endl;
    std::cout<<"debug joint1_servo_angle_init: "<<joint1_servo_angle_init<<std::endl;

    float _duration_1 = 200;  
    float _percent_1 = 0;   

    Eigen::VectorXf target_angle(2);
    target_angle<<0,30;

    Eigen::VectorXf target_angle_command(2);


    Eigen::VectorXf angle_init(2);
    angle_init<<joint0_servo_angle_init,joint1_servo_angle_init;

    int counter = 0;
    const int amplitude = 30; // Amplitude of the sine wave
    const int frequency = 4; // Frequency of the sine wave

    while(1)
    {
        _percent_1 += (float)1 / _duration_1;
        _percent_1 = _percent_1 > 1 ? 1 : _percent_1;

        if (_percent_1 < 1.0)
        {
            for (int id = 0; id < 2; id++)
            {
                target_angle_command(id) = (1 - _percent_1) * angle_init(id) + _percent_1 * target_angle(id);
                float joint1_servo_positon;
                if(id == 0 )
                    joint1_servo_positon = utilities::angle2encoder(target_angle_command(id),servo0_limit_enoder,joint0_limitation,direction(0));
                if(id == 1 )
                    joint1_servo_positon = utilities::angle2encoder(target_angle_command(id),servo1_limit_enoder,joint1_limitation,direction(1));
                joints.set_position(id, joint1_servo_positon);
            }
        }
        else
        {
            counter++;
            int desired0_position =  amplitude * sin(2 * M_PI * frequency * counter / 500.0);
            float joint0_servo_positon;
            joint0_servo_positon = utilities::angle2encoder(desired0_position,servo0_limit_enoder,joint0_limitation,direction(0));
            joints.set_position(0, joint0_servo_positon);

            int desired1_position =  20 * sin(2 * M_PI * frequency * counter / 500.0)+target_angle(1);
            float joint1_servo_positon;
            joint1_servo_positon = utilities::angle2encoder(desired1_position,servo1_limit_enoder,joint1_limitation,direction(1));
            joints.set_position(1, joint1_servo_positon);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2)); 
    }
    return 0;
}