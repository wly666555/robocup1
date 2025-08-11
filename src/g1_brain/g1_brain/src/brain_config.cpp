#include "brain_config.h"



void BrainConfig::handle()
{
    treeFilePath = "/home/unitree/wly666/g1_brain/behavior_trees/Sub_CamFindAndTrackBall.xml";
    // playerStartPos[left, right]
    if (playerStartPos != "left" && playerStartPos != "right")
    {
        throw invalid_argument("palyer_start_pos must be one of [left, right]. Got: " + playerStartPos);
    }

    // fieldType [adult_size, kid_size]
    if (fieldType == "adult_size")
    {
        fieldDimensions = FD_ADULTSIZE;
    }
    else if (fieldType == "kid_size")
    {
        fieldDimensions = FD_KIDSIZE;
    }
    else
    {
        throw invalid_argument("[Error] fieldType must be one of [adult_size, kid_size]. Got: " + fieldType);
    }
}


