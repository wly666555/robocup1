#include "joy_msg.h"

JoyMsg::JoyMsg(const sensor_msgs::msg::Joy &msg)
{
    if (JoyMsg::type == JoyMsg::LOGITECH)
    {
        fromLogicall(msg);
    }
    else if (JoyMsg::type == JoyMsg::BEITONG)
    {
        fromBeiTong(msg);
    }
}

void JoyMsg::print(ostream &os) const
{
    os << "BTN_A: " << BTN_A << endl;
    os << "BTN_B: " << BTN_B << endl;
    os << "BTN_X: " << BTN_X << endl;
    os << "BTN_Y: " << BTN_Y << endl;
    os << "BTN_LT: " << BTN_LT << endl;
    os << "BTN_LB: " << BTN_LB << endl;
    os << "BTN_RT: " << BTN_RT << endl;
    os << "BTN_RB: " << BTN_RB << endl;
    os << "AX_LX: " << AX_LX << endl;
    os << "AX_LY: " << AX_LY << endl;
    os << "AX_RX: " << AX_RX << endl;
    os << "AX_RY: " << AX_RY << endl;
    os << "AX_DX: " << AX_DX << endl;
    os << "AX_DY: " << AX_DY << endl;
}

void JoyMsg::fromLogicall(const sensor_msgs::msg::Joy &msg)
{
    BTN_A = msg.buttons[1] == 1;
    BTN_B = msg.buttons[2] == 1;
    BTN_X = msg.buttons[0] == 1;
    BTN_Y = msg.buttons[3] == 1;
    BTN_LT = msg.buttons[6] == 1;
    BTN_LB = msg.buttons[4] == 1;
    BTN_RT = msg.buttons[7] == 1;
    BTN_RB = msg.buttons[5] == 1;
    AX_LX = msg.axes[0];
    AX_LY = msg.axes[1];
    AX_RX = msg.axes[2];
    AX_RY = msg.axes[3];
    AX_DX = msg.axes[4];
    AX_DY = msg.axes[5];
}

void JoyMsg::fromBeiTong(const sensor_msgs::msg::Joy &msg)
{
    BTN_A = msg.buttons[0] == 1;
    BTN_B = msg.buttons[1] == 1;
    BTN_X = msg.buttons[2] == 1;
    BTN_Y = msg.buttons[3] == 1;
    BTN_LT = msg.axes[2] < 0.0;
    BTN_LB = msg.buttons[4] == 1;
    BTN_RT = msg.axes[5] < 0.0;
    BTN_RB = msg.buttons[5] == 1;
    AX_LX = msg.axes[0];
    AX_LY = msg.axes[1];
    AX_RX = msg.axes[3];
    AX_RY = msg.axes[4];
    AX_DX = msg.axes[6];
    AX_DY = msg.axes[7];
}