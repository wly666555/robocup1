#include <iostream>
#include <dds/dds.hpp>
#include "DetectionModule.hpp"
#include <thread>
int main(int argc, char *argv[])
{
    // 创建DomainParticipant（域ID为0）
    dds::domain::DomainParticipant participant(0);

    // 创建Topic QoS配置
    dds::topic::qos::TopicQos topic_qos = dds::topic::qos::TopicQos();
    
    // 创建检测结果主题
    dds::topic::Topic<DetectionModule::DetectionResults> topic(
        participant, 
        "detectionresults",
        topic_qos);

    // 创建Subscriber
    dds::sub::Subscriber subscriber(participant);

    // 创建DataReader
    dds::sub::DataReader<DetectionModule::DetectionResults> reader(subscriber, topic);

    std::cout << "===== [Subscriber] Waiting for data..." << std::endl;

    while (true) {
        // 读取所有有效样本
        auto samples = reader.take();
        for (const auto& sample : samples) {
            if (sample.info().valid()) {
                const auto& data = sample.data();
                
                // 打印检测结果数量
                std::cout << "\n===== Received " << data.results().size() 
                        << " detection results =====\n";

                // 遍历每个检测结果
                for (const auto& result : data.results()) {
                    std::cout << "Class ID: " << result.class_id() << "\n"
                            << "Class Name: " << result.class_name() << "\n"
                            << "Score: " << result.score() << "\n";

                    // 打印边界框坐标
                    std::cout << "Box: [";
                    for (const auto& coord : result.box()) {
                        std::cout << coord << " ";
                    }
                    std::cout << "\b]\n";  // 退格消除最后一个空格

                    // 打印三维坐标
                    std::cout << "XYZ: [";
                    for (const auto& val : result.xyz()) {
                        std::cout << val << " ";
                    }
                    std::cout << "\b]\n";

                    // 打印偏移量
                    std::cout << "Offset: [";
                    for (const auto& val : result.offset()) {
                        std::cout << val << " ";
                    }
                    std::cout << "\b]\n";

                    // 打印FOV偏移量
                    std::cout << "FOV Offset: [";
                    for (const auto& val : result.offset_fov()) {
                        std::cout << val << " ";
                    }
                    std::cout << "\b]\n" << std::endl;
                }
            }
        }
        
        // 添加短暂休眠以减少CPU占用
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return 0;
}