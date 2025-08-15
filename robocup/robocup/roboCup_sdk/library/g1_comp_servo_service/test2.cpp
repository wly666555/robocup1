#include "dxl.h"
#include "dds/Publisher.h"
#include "dds/Subscription.h"
#include <unitree/idl/go2/MotorCmds_.hpp>
#include <unitree/idl/go2/MotorStates_.hpp>
#include <unitree/common/thread/thread.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include "utilities.h"
#include "yaml_parser.h"


class IOUSB
{
public:
    IOUSB()
    {
        auto joints = dxl::Motors<2>(57600);
        for (int i(0); i<2; i++)
        {
            joints.set_position_p_gain(i, 0);
            joints.set_position_d_gain(i, 0);
        }
        spdlog::info("Set gain success.");

        motorstate = std::make_unique<unitree::robot::RealTimePublisher<unitree_go::msg::dds_::MotorStates_>>(
            "rt/g1_comp_servo/state");
        motorstate->msg_.states().resize(2);
        motorcmd = std::make_shared<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorCmds_>>(
            "rt/g1_comp_servo/cmd");
        motorcmd->msg_.cmds().resize(2);

        std::string env_cfg_path = "/home/unitree/g1_comp_servo_service/config/config2.yaml";
        yamlParam_.setup(env_cfg_path.c_str());

        servo0_calibration = yamlParam_.ReadFloatFromYaml("servo0_calibration");
        servo1_calibration = yamlParam_.ReadFloatFromYaml("servo1_calibration");

        joint0_limitation.setZero(2);
        joint1_limitation.setZero(2);
        joint0_limitation = yamlParam_.ReadVectorFromYaml("joint0",2);
        joint1_limitation = yamlParam_.ReadVectorFromYaml("joint1",2);

        servo0_limit_enoder = yamlParam_.ReadFloatFromYaml("servo0_calibration");
        servo1_limit_enoder = yamlParam_.ReadFloatFromYaml("servo1_calibration");

        direction.setZero(2);
        direction = yamlParam_.ReadVectorFromYaml("direction",2);

        servo_angle.setZero(2);
    }

    void Start()
    {
        thread_ = std::make_shared<unitree::common::RecurrentThread>(
           0.015 * 1000, std::bind(&IOUSB::run, this));
    }

    // std::shared_ptr<dxl::g1Head> joints;
    dxl::Motors<2> joints;
private:
    void run()
    {
        check_motor_enable();

        
        // for(int i(0); i<2; i++)
        // {   
        
        //     joints.cmd.q[i] = std::clamp(mapValue(motorcmd->msg_.cmds()[i].q(),i), lower_position_limit[i], upper_position_limit[i]);
        //     // std::cout<<"joints.cmd.q[i]:  "<<joints.cmd.q[i]<<std::endl;
        // }
      
        // joints.Send();

        // /* --- Recv --- */
        // joints.Recv();

        // // std::cout<<"debug recv: "<<std::endl;
        // for(int i(0); i<2; i++)
        // {
        //     motorstate->msg_.states()[i].q(reverseMapValue(joints.state.q[i],i));
        //     // std::cout<<" motorstate->msg_.states()[i].q(): "<< motorstate->msg_.states()[i].q()<<std::endl;
        // }
        // // std::cout<<std::endl;
        // motorstate->msg_.states()[0].mode(joint_enable ? 1 : 0);

        joints.sync_get_position();
        servo_angle(0) = utilities::encoder2angle(joints.present_position[0],servo0_limit_enoder,joint0_limitation,direction(0));
        servo_angle(1) = utilities::encoder2angle(joints.present_position[1],servo1_limit_enoder,joint1_limitation,direction(1));
        for(int i(0); i<2; i++)
        {
            motorstate->msg_.states()[i].q(servo_angle(i));
        }
        motorstate->unlockAndPublish();
    }

    void check_motor_enable()
    {
        if(motorcmd->msg_.cmds()[0].mode() == 1)
        {
            if(!joint_enable)
            {
                for(int i(0); i<MOTOR_NUM; i++)
                {
                    joints.enable(i);
                }
                joint_enable = true;
                spdlog::info("Enable arm.");
            }
        }
        else
        {
            if(joint_enable)
            {
                for(int i(0); i<MOTOR_NUM; i++)
                {
                    joints.enable(i, 0);
                }
                joint_enable = false;
                spdlog::info("Release arm.");
            }
        }
    }

    unitree::common::RecurrentThreadPtr thread_;
    bool joint_enable = false;
    std::array<float, 2> upper_position_limit = { 274, 60}; // 臂用角度表示，较为直观；夹爪采用归一化
    std::array<float, 2> lower_position_limit = { 174,-50};

    /* DDS */
    std::unique_ptr<unitree::robot::RealTimePublisher<unitree_go::msg::dds_::MotorStates_>> motorstate; // motor state
    std::shared_ptr<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorCmds_>> motorcmd; // motor cmd
    YamlParser yamlParam_;
    float servo0_calibration;
    float servo1_calibration;
    Eigen::VectorXf joint0_limitation;
    Eigen::VectorXf joint1_limitation;
    float servo0_limit_enoder;
    float servo1_limit_enoder;
    Eigen::VectorXf direction;
    Eigen::VectorXf servo_angle;
};

int main(int argc, char** argv)
{
    auto vm = param::helper(argc, argv);
    param::parse_config();

    unitree::robot::ChannelFactory::Instance()->Init(1, argv[1]);
    // unitree::robot::ChannelFactory::Instance()->Init(0);
    auto iousb = IOUSB();
    iousb.Start();
    while(true)
    {
        sleep(1);
    }
    return 0;
}