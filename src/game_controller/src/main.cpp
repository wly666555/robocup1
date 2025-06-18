#include "game_controller_node.h"

using namespace std;


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto node = make_shared<GameControllerNode>("game_controller_node");

    node->init();
    
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}