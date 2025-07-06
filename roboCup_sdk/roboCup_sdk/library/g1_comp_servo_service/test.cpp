#include "dxl.h"
#include "dds/Publisher.h"
#include "dds/Subscription.h"
#include <unitree/idl/go2/MotorCmds_.hpp>
#include <unitree/idl/go2/MotorStates_.hpp>
#include <unitree/common/thread/thread.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

class IOUSB
{
public:
    IOUSB()
    {
        // joints
        joints = std::make_shared<dxl::g1Head>();
        // for(int i(0); i<2; i++) 
        // {
        //     upper_position_limit[i] = upper_position_limit[i] / 180. * M_PI;
        //     lower_position_limit[i] = lower_position_limit[i] / 180. * M_PI;
        // }

        for (int i(0); i<param::cfg.kp.size(); i++)
        {
            joints->set_position_p_gain(i, param::cfg.kp[i]);
            joints->set_position_d_gain(i, param::cfg.kd[i]);
        }
        spdlog::info("Set gain success.");

        // dds
        motorstate = std::make_unique<unitree::robot::RealTimePublisher<unitree_go::msg::dds_::MotorStates_>>(
            "rt/" + param::ns + "/state");
        motorstate->msg_.states().resize(2);
        motorcmd = std::make_shared<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorCmds_>>(
            "rt/" + param::ns + "/cmd");
        motorcmd->msg_.cmds().resize(2);
    }

    void Start()
    {
        thread_ = std::make_shared<unitree::common::RecurrentThread>(
            param::cfg.dt * 1000, std::bind(&IOUSB::run, this));
    }

    std::shared_ptr<dxl::g1Head> joints;
private:
    void run()
    {
        // Check whether enable motor
        check_motor_enable();
        // std::cout<<"debug "<<std::endl;
        for(int i(0); i<2; i++)
        {   
            // std::cout<<"motorcmd->msg_.cmds()[i].q():  "<<motorcmd->msg_.cmds()[i].q()<<std::endl;
            // std::cout<<"lower_position_limit[i]:  "<<lower_position_limit[i]<<std::endl;
            // std::cout<<"upper_position_limit[i]:  "<<upper_position_limit[i]<<std::endl
            joints->cmd.q[i] = std::clamp(mapValue(motorcmd->msg_.cmds()[i].q(),i), lower_position_limit[i], upper_position_limit[i]);
            // std::cout<<"joints->cmd.q[i]:  "<<joints->cmd.q[i]<<std::endl;
        }
        // std::cout<<std::endl;

        /* --- Send --- */
        // if(joint_enable)
        // {
        //     joints->Send();
        // }
        joints->Send();

        /* --- Recv --- */
        joints->Recv();

        // std::cout<<"debug recv: "<<std::endl;
        for(int i(0); i<2; i++)
        {
            motorstate->msg_.states()[i].q(reverseMapValue(joints->state.q[i],i));
            // std::cout<<" motorstate->msg_.states()[i].q(): "<< motorstate->msg_.states()[i].q()<<std::endl;
        }
        // std::cout<<std::endl;
        motorstate->msg_.states()[0].mode(joint_enable ? 1 : 0);
        motorstate->unlockAndPublish();
    }

    float mapValue(float x, int index) {
    // 检查索引是否有效
    if (index < 0 || index >= upper_position_limit.size()) {
        std::cerr << "索引超出范围" << std::endl;
        return -1;  // 错误代码
    }
    
    // 设置输入范围
    float inputMin, inputMax;
    if (index == 0) {
        inputMin = -50;
        inputMax = 50;
    } else if (index == 1) {
        inputMin = -20;
        inputMax = 90;
    } else {
        std::cerr << "未定义的索引" << std::endl;
        return -1;  // 错误代码
    }
    
    // 计算斜率和截距
    float slope = (lower_position_limit[index] - upper_position_limit[index]) / (inputMax - inputMin);
    float intercept = upper_position_limit[index] - slope * inputMin;
    
    // 计算映射值
    float mappedValue = slope * x + intercept;

    // 检查是否超出上下限
    if (mappedValue < std::min(upper_position_limit[index], lower_position_limit[index]) ||
        mappedValue > std::max(upper_position_limit[index], lower_position_limit[index])) {
        std::cerr << "映射值超出范围" << std::endl;
        return -1;  // 错误代码
    }

    return mappedValue;
}

float reverseMapValue(float y, int index) {
    // 检查索引是否有效
    if (index < 0 || index >= upper_position_limit.size()) {
        std::cerr << "索引超出范围" << std::endl;
        return -1;  // 错误代码
    }
    
    // 设置输入范围
    float inputMin, inputMax;
    if (index == 0) {
        inputMin = -50;
        inputMax = 50;
    } else if (index == 1) {
        inputMin = -20;
        inputMax = 90;
    } else {
        std::cerr << "未定义的索引" << std::endl;
        return -1;  // 错误代码
    }
    
    // 计算斜率和截距
    float slope = (lower_position_limit[index] - upper_position_limit[index]) / (inputMax - inputMin);
    float intercept = upper_position_limit[index] - slope * inputMin;

    // 计算反映射值
    float mappedValue = (y - intercept) / slope;

    // 检查是否超出原始输入范围
    if (mappedValue < std::min(inputMin, inputMax) ||
        mappedValue > std::max(inputMin, inputMax)) {
        std::cerr << "反映射值超出原始范围" << std::endl;
        return -1;  // 错误代码
    }

    return mappedValue;
}

    /**
     * @brief 在DDS mode 变化时进行电机使能
     * 
     * 目前仅允许同时使能或卸力, 通过Joint[0].mode 判断
     */
    void check_motor_enable()
    {
        if(motorcmd->msg_.cmds()[0].mode() == 1)
        {
            if(!joint_enable)
            {
                for(int i(0); i<MOTOR_NUM; i++)
                {
                    joints->enable(i);
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
                    joints->enable(i, 0);
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
};

int main(int argc, char** argv)
{
    auto vm = param::helper(argc, argv);
    param::parse_config();

    unitree::robot::ChannelFactory::Instance()->Init(0, param::cfg.dds.network);
    auto iousb = IOUSB();
    iousb.Start();
    while(true)
    {
        sleep(1);
    }
    return 0;
}