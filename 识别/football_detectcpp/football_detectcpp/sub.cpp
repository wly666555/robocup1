#include <iostream>// 标准输入输出流，用于打印信息
#include <dds/dds.hpp>// 引入DDS相关的头文件,用于数据分发服务
#include "DetectionModule.hpp"// 引入检测模块的数据类型定义     
#include <thread>// 引入线程库，用于控制循环中的休眠
int main(int argc, char *argv[])
{
    // 创建DomainParticipant（域ID为0）【创建一个DDS域参与者，域ID为0】
    dds::domain::DomainParticipant participant(0);

    // 创建Topic QoS配置【创建一个主题的质量服务配置对象，使用默认设置】
    dds::topic::qos::TopicQos topic_qos = dds::topic::qos::TopicQos();
    
    // 创建检测结果主题【创建一个名为"detectionresults"的主题，数据类型为DetectionModule::DetectionResults】
    dds::topic::Topic<DetectionModule::DetectionResults> topic(
        participant, 
        "detectionresults",
        topic_qos);

    // 创建Subscriber【创建订阅者，与前面的参与者相关联】
    dds::sub::Subscriber subscriber(participant);

    // 创建DataReader【创建数据读取器，用于接受指定主题的数据】
    dds::sub::DataReader<DetectionModule::DetectionResults> reader(subscriber, topic);

    //打印提示信息
    std::cout << "===== [Subscriber] Waiting for data..." << std::endl;

    while (true) {
        auto samples = reader.take();//从DataReader中获取数据样本
        //遍历每个样本，如果样本有效，则提取数据部分
        for (const auto& sample : samples) {
            if (sample.info().valid()) {
                const auto& data = sample.data();
                
                // 打印检测结果数量
                std::cout << "\n===== Received " << data.results().size() 
                        << " detection results =====\n";

                // 遍历每个检测结果，打印类别ID、名称、置信度分数等信息
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
        
        // 添加短暂休眠以减少CPU占用（休息50ms）
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return 0;
}