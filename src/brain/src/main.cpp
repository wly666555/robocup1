#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <thread>

#include "brain.h"

#define HZ 100

using namespace std;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<Brain> brain = std::make_shared<Brain>();

    brain->init();

    thread t([&brain]()
             {
        while (rclcpp::ok()) {
            brain->tick();
            this_thread::sleep_for(chrono::milliseconds(static_cast<int>(1000 / HZ)));
        } });

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(brain);
    executor.spin();

    t.join();
    rclcpp::shutdown();
    return 0;
}
