#ifndef G1_BRAIN_CONFIG_HPP
#define G1_BRAIN_CONFIG_HPP

#include <string>
#include <map>
#include <sstream>

struct BrainConfig {
    int teamId = 0;
    int playerId = 29;
    std::string fieldType = "";
    std::string playerRole = "striker";
    std::string playerStartPos = "";

    double robotHeight = 1.0;
    double odomFactor = 1.0;
    double vxFactor = 0.95;
    double yawOffset = 0.1;

    std::map<std::string, double> fieldDimensions;

    double memoryLength = 5.0;

    void handle() {}

    void print(std::ostringstream& oss) const {
        oss << "BrainConfig: teamId=" << teamId
            << ", playerId=" << playerId
            << ", playerRole=" << playerRole;
    }
};

#endif // G1_BRAIN_CONFIG_HPP


