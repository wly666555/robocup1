#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <thread>

#include "brain.h"

#define HZ 100

using namespace std;


int main(int argc, char** argv) {

    // 初始化 ros2
    rclcpp::init(argc, argv);
    
    // Brain 对象
    std::shared_ptr<Brain> brain = std::make_shared<Brain>();

    // 执行初始化操作：读取参数，构建 BehaviorTree 等
    brain->init();
    
    // 单独开一个线程执行 brain.tick
    thread t([&brain]() {
        while (rclcpp::ok()) {
            brain->tick();
            this_thread::sleep_for(chrono::milliseconds(static_cast<int>(1000 / HZ)));
        }
    });

    // 使用单线程执行器
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(brain);
    executor.spin();

    t.join();
    rclcpp::shutdown();
    return 0;
} 