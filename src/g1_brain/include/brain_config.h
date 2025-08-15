#pragma once

#include <string>
#include <ostream>

#include "locate/types.h"
#include "locate/math_utils.h"


using namespace std;

/**
 * 存储 Brain 需要的一些配置值，这些值应该是初始化就确认好了，在机器人决策过程中只读不改的
 * 需要在决策过程中变化的值，应该放到 BrainData 中
 * 注意：
 * 1、配置文件会从 config/config.yaml 中读取
 * 2、如果存在 config/config_local.yaml，将会覆盖 config/config.yaml 的值
 * 
 */

 
using namespace std;

class BrainConfig
{
public:

    string fieldType; // 球场类型
    string playerStartPos;
    string playerRole; 

    FieldDimensions fieldDimensions; // 球场尺寸
    
    // 机器人相关参数
    double pitch_compensation;
    double yaw_compensation;
    double height;
    double scale_factor;
    double servo_height;

    // Head servo soft limits (degrees)
    double yaw_limit_min;
    double yaw_limit_max;
    double pitch_limit_min;
    double pitch_limit_max;

    std::string treeFilePath;

    // 速度上限
    double vxLimit = 2.0;
    double vyLimit = 2.0;
    double vthetaLimit = 2.0;


    //camera param
    double camPixX = 640;
    double camPixY = 480;
    double camAngleX = deg2rad(86);
    double camAngleY = deg2rad(57);
    
    // 记忆相关参数
    double memoryLength= 3.0;  // 球位置记忆时间（秒）

    // BrainNode 填充完参数后，调用 handle() 进行一些参数的处理（校正、计算等）,成功返回 true
    void handle() ;
    
};
