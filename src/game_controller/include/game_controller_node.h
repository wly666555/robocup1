#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>

#include "RoboCupGameControlData.h"

#include "game_controller_interface/msg/game_control_data.hpp"

using namespace std;

class GameControllerNode : public rclcpp::Node
{
public:
    GameControllerNode(string name);
    ~GameControllerNode();

    // init UDP Socket
    void init();

    // Enter the loop, receive UDP broadcast messages, process them and then publish them to the Ros2 Topic.
    void spin();

private:
    // Check whether the received packet is from the whitelisted machine.
    bool check_ip_white_list(string ip);

    // Process the data packet (copy field by field).
    void handle_packet(RoboCupGameControlData &data, game_controller_interface::msg::GameControlData &msg);

    // The listening port, read from the configuration file.
    int _port;
    // Whether to enable the IP whitelist.
    bool _enable_ip_white_list;
    // The list of IPs in the whitelist.
    vector<string> _ip_white_list;

    // UDP Socket
    int _socket;
    // thread

    thread _thread;

    // Ros2 publisher
    rclcpp::Publisher<game_controller_interface::msg::GameControlData>::SharedPtr _publisher;
};