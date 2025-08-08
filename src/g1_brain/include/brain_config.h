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


class BrainConfig
{
public:

    string fieldType; // 球场尺寸
    string location_mode;
    string playerStartPos;
    
    FieldDimensions fieldDimensions; // 球场尺寸
    // 机器人相关参数
    double pitch_compensation;
    double yaw_compensation;
    double height;
    double scale_factor;
    
    // 记忆相关参数
    double memoryLength;  // 球位置记忆时间（秒）
    
    void handle() ;
}
