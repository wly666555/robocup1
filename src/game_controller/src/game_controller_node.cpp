#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>

#include "game_controller_node.h"

GameControllerNode::GameControllerNode(string name) : rclcpp::Node(name)
{
    _socket = -1;

    // Declare Ros2 parameters. Note that newly added parameters in the configuration file need to be explicitly declared here.
    declare_parameter<int>("port", 3838);
    declare_parameter<bool>("enable_ip_white_list", false);
    declare_parameter<vector<string>>("ip_white_list", vector<string>{});

    // Read parameters from the configuration. Note that the read parameters should be printed in the log for easy problem investigation.
    get_parameter("port", _port);
    RCLCPP_INFO(get_logger(), "[get_parameter] port: %d", _port);
    get_parameter("enable_ip_white_list", _enable_ip_white_list);
    RCLCPP_INFO(get_logger(), "[get_parameter] enable_ip_white_list: %d", _enable_ip_white_list);
    get_parameter("ip_white_list", _ip_white_list);
    RCLCPP_INFO(get_logger(), "[get_parameter] ip_white_list(len=%ld)", _ip_white_list.size());
    for (size_t i = 0; i < _ip_white_list.size(); i++)
    {
        RCLCPP_INFO(get_logger(), "[get_parameter]     --[%ld]: %s", i, _ip_white_list[i].c_str());
    }

    // Create a publisher and publish to /robocup/game_controller
    _publisher = create_publisher<game_controller_interface::msg::GameControlData>("/robocup/game_controller", 10);
}

GameControllerNode::~GameControllerNode()
{
    if (_socket >= 0)
    {
        close(_socket);
    }

    if (_thread.joinable())
    {
        _thread.join();
    }
}

/**
 * Create a Socket and bind it to the specified port.
 */
void GameControllerNode::init()
{
    _socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (_socket < 0)
    {
        RCLCPP_ERROR(get_logger(), "socket failed: %s", strerror(errno));
        throw runtime_error(strerror(errno));
    }

    sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(_port);

    if (bind(_socket, (sockaddr *)&addr, sizeof(addr)) < 0)
    {
        RCLCPP_ERROR(get_logger(), "bind failed: %s (port=%d)", strerror(errno), _port);
        throw runtime_error(strerror(errno));
    }

    RCLCPP_INFO(get_logger(), "Listening for UDP broadcast on 0.0.0.0:%d", _port);

    // Start a new thread to receive data. The main thread enters the Node's own spin to handle some services of the Node itself.
    _thread = thread(&GameControllerNode::spin, this);
}

void GameControllerNode::spin()
{
    // Used to obtain the remote address.
    sockaddr_in remote_addr;
    socklen_t remote_addr_len = sizeof(remote_addr);

    // The 'data' and'msg' are reused within the loop. Pay attention to this point when updating the code in the future.
    RoboCupGameControlData data;
    game_controller_interface::msg::GameControlData msg;

    while (rclcpp::ok())
    {
        // Receive data packets from the socket. Expect to receive complete data packets.
        ssize_t ret = recvfrom(_socket, &data, sizeof(data), 0, (sockaddr *)&remote_addr, &remote_addr_len);
        if (ret < 0)
        {
            RCLCPP_ERROR(get_logger(), "receiving UDP message failed: %s", strerror(errno));
            continue;
        }

        // Obtain the remote address
        string remote_ip = inet_ntoa(remote_addr.sin_addr);

        // Ignore incomplete packets
        if (ret != sizeof(data))
        {
            RCLCPP_INFO(get_logger(), "packet from %s invalid length=%ld", remote_ip.c_str(), ret);
            continue;
        }

        if (data.version != GAMECONTROLLER_STRUCT_VERSION)
        {
            RCLCPP_INFO(get_logger(), "packet from %s invalid version: %d", remote_ip.c_str(), data.version);
            continue;
        }

        // filter
        if (!check_ip_white_list(remote_ip))
        {
            RCLCPP_INFO(get_logger(), "received packet from %s, but not in ip white list, ignore it", remote_ip.c_str());
            continue;
        }

        // handle packet
        handle_packet(data, msg);

        // publish
        _publisher->publish(msg);

        RCLCPP_INFO(get_logger(), "handle packet successfully ip=%s, packet_number=%d", remote_ip.c_str(), data.packetNumber);
    }
}

/**
 * Check whether the IP is in the whitelist. Return true if the whitelist is not enabled or the IP is in the whitelist, and return false in other cases.
 */
bool GameControllerNode::check_ip_white_list(string ip)
{
    // Return true if it is not enabled or is in the whitelist.
    if (!_enable_ip_white_list)
    {
        return true;
    }
    for (size_t i = 0; i < _ip_white_list.size(); i++)
    {
        if (ip == _ip_white_list[i])
        {
            return true;
        }
    }
    return false;
}

/**
 * Convert the UDP data format to the custom Ros2 message format (copy field by field).
 * If any changes are needed, be sure to carefully check each field.
 */
void GameControllerNode::handle_packet(RoboCupGameControlData &data, game_controller_interface::msg::GameControlData &msg)
{

    // The length of the header is fixed at 4.
    for (int i = 0; i < 4; i++)
    {
        msg.header[i] = data.header[i];
    }
    msg.version = data.version;
    msg.packet_number = data.packetNumber;
    msg.players_per_team = data.playersPerTeam;
    msg.game_type = data.gameType;
    msg.state = data.state;
    msg.first_half = data.firstHalf;
    msg.kick_off_team = data.kickOffTeam;
    msg.secondary_state = data.secondaryState;
    // The length of secondary_state_info is fixed at 4.
    for (int i = 0; i < 4; i++)
    {
        msg.secondary_state_info[i] = data.secondaryStateInfo[i];
    }
    msg.drop_in_team = data.dropInTeam;
    msg.drop_in_time = data.dropInTime;
    msg.secs_remaining = data.secsRemaining;
    msg.secondary_time = data.secondaryTime;
    /// The length of teams is fixed at 2.
    for (int i = 0; i < 2; i++)
    {
        msg.teams[i].team_number = data.teams[i].teamNumber;
        msg.teams[i].team_colour = data.teams[i].teamColour;
        msg.teams[i].score = data.teams[i].score;
        msg.teams[i].penalty_shot = data.teams[i].penaltyShot;
        msg.teams[i].single_shots = data.teams[i].singleShots;
        msg.teams[i].coach_sequence = data.teams[i].coachSequence;

        // msg.teams[i].players is defined as an array with variable length. Note that it should be distinguished from arrays with fixed length.
        int coach_message_len = sizeof(data.teams[i].coachMessage) / sizeof(data.teams[i].coachMessage[0]);
        msg.teams[i].coach_message.clear(); // Since the'msg' is reused, remember to call clear() here.
        for (int j = 0; j < coach_message_len; j++)
        {
            msg.teams[i].coach_message.push_back(data.teams[i].coachMessage[j]);
        }

        // msg.teams[i].cocah
        msg.teams[i].coach.penalty = data.teams[i].coach.penalty;
        msg.teams[i].coach.secs_till_unpenalised = data.teams[i].coach.secsTillUnpenalised;
        msg.teams[i].coach.number_of_warnings = data.teams[i].coach.numberOfWarnings;
        msg.teams[i].coach.yellow_card_count = data.teams[i].coach.yellowCardCount;
        msg.teams[i].coach.red_card_count = data.teams[i].coach.redCardCount;
        msg.teams[i].coach.goal_keeper = data.teams[i].coach.goalKeeper;

        // msg.teams[i].coach_message is defined as an array with variable length. Pay attention to the distinction from fixed-length arrays.
        int players_len = sizeof(data.teams[i].players) / sizeof(data.teams[i].players[0]);
        msg.teams[i].players.clear(); // // Since the'msg' is reused, remember to call clear() here.
        for (int j = 0; j < players_len; j++)
        {
            game_controller_interface::msg::RobotInfo rf;
            rf.penalty = data.teams[i].players[j].penalty;
            rf.secs_till_unpenalised = data.teams[i].players[j].secsTillUnpenalised;
            rf.number_of_warnings = data.teams[i].players[j].numberOfWarnings;
            rf.yellow_card_count = data.teams[i].players[j].yellowCardCount;
            rf.red_card_count = data.teams[i].players[j].redCardCount;
            rf.goal_keeper = data.teams[i].players[j].goalKeeper;
            msg.teams[i].players.push_back(rf);
        }
    }
}