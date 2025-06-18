#pragma once
#include <sensor_msgs/msg/joy.hpp>
#include <string>

using namespace std;

class JoyMsg {
public:
    enum JoyType { LOGITECH, BEITONG };
    inline static JoyType type;

    bool BTN_A = false;
    bool BTN_B = false;
    bool BTN_X = false;
    bool BTN_Y = false;
    bool BTN_LT = false;
    bool BTN_LB = false;
    bool BTN_RT = false;
    bool BTN_RB = false;
    double AX_LX = 0.0;
    double AX_LY = 0.0;
    double AX_RX = 0.0;
    double AX_RY = 0.0;
    double AX_DX = 0.0;
    double AX_DY = 0.0;

    JoyMsg(const sensor_msgs::msg::Joy &msg);
    void print(ostream &os) const;

private:
    void fromLogicall(const sensor_msgs::msg::Joy &msg);
    void fromBeiTong(const sensor_msgs::msg::Joy &msg);
};