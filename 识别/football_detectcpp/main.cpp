#include <sys/stat.h> //文件状态检查和文件操作
#include <unistd.h> //进程控制、文件操作、系统调用
#include <iostream> //C++标准输入/输出流库
#include <string> //C++标准字符串库
#include "YOLO.h" //YOLO目标检测类头文件

#include <thread> // 标准线程库
#include <atomic> //用于 统计检测帧数 或 控制线程同步（如 std::atomic<bool> 控制线程退出）。
#include <queue> //标准库队列，通常用于 任务缓冲（生产者-消费者模型）
#include <mutex> //互斥锁，保护任务队列（如多线程推入/取出检测任务）
#include "dds/DetectionModule.hpp" //自定义 DDS 检测模块
#include <dds/dds.hpp> //DDS核心库，发布检测结果
#include"common.h" //项目公共头文件
/**
 * @brief Setting up Tensorrt logger //TensorRT日志记录器设置
*/
class Logger : public nvinfer1::ILogger {//继承TensorRT的日志接口
    void log(Severity severity, const char* msg) noexcept override {
        // 只输出严重级别高于警告的日志
        if (severity <= Severity::kWARNING)
            std::cout << msg << std::endl;// 将日志消息（msg）输出，并在末尾添加换行符（std::endl）
    }
}logger;

// 图像参数定义
float image_width = 640;//图像横向分辨率
float image_height = 480; //图像横向分辨率
float init_fps = 30;// 初始帧率(FPS)
float horizontal_fov = 86;// 水平视场角度
float vertical_fov = 57;  // 垂直视场角度


// Camera reconnection logic相机控制器类
class CameraController {
    rs2::pipeline pipe;// RealSense管道，用于管理数据流
    rs2::config cfg;          // 管道配置参数
    std::string serial;       // 相机序列号
  // 图像参数常量  
    const int width = image_width, height = image_height, fps = init_fps;
    
public:
    rs2_intrinsics intrinsics; // 外部访问相机内参
 // 构造函数，配置彩色和深度流
    CameraController() {
        cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, fps);
        cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);
    }
  // 初始化相机
    bool initialize() {
        try {
            auto profile = pipe.start(cfg); // 启动管道
            auto stream = profile.get_stream(RS2_STREAM_COLOR)
                          .as<rs2::video_stream_profile>(); // 获取彩色流配置
            intrinsics = stream.get_intrinsics(); // 获取相机内参
            serial = profile.get_device().get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);// 获取序列号
            return true;// 初始化成功
        } catch (const rs2::error& e) {
            std::cerr << "Camera init failed: " << e.what() << std::endl;
            return false;// 初始化失败
        }
    }
// 关闭相机
    void shutdown() {
        try { pipe.stop(); }// 停止管道
        catch (...) {}        // 忽略任何异常
    }
 // 轮询获取帧数据
    rs2::frameset poll_frames(int max_retries = 100) {
        for (int i = 0; i < max_retries; ++i) {
            rs2::frameset frames;
            if (pipe.poll_for_frames(&frames)) {  // 非阻塞获取帧
                return frames;  // 成功获取返回帧
            }
            std::this_thread::sleep_for(10ms); // 短暂等待
        }
        return {}; // Return empty frame to indicate failure// 失败返回空帧
    }
 // 获取相机序列号
    std::string get_serial() const { return serial; }
};

// Global control variables全局控制变量
std::atomic<bool> camera_connected{false};// 相机连接状态(原子布尔)
std::atomic<int> reconnect_attempts{0};     // 重连尝试次数(原子整数)
constexpr int MAX_RECONNECT = 500;          // 最大重连次数

// 计算从像素偏移量到角度的转换
std::pair<float, float> calculate_angles_from_offsets(
    float dx,// x方向偏移(像素)
    float dy,       // y方向偏移(像素)
    int image_width, // 图像宽度
    int image_height, // 图像高度
    float horizontal_fov, // 水平视场角
    float vertical_fov)   // 垂直视场角
{
    //  归一化像素偏移量(相对于图像中心)
    const float normalized_dx = dx / static_cast<float>(image_width / 2.0f);
    const float normalized_dy = dy / static_cast<float>(image_height / 2.0f);

    // Calculate angle offsets based on FOV基于视场角计算角度偏移
    const float yaw = normalized_dx * (horizontal_fov / 2.0f);// 偏航角
    const float pitch = -normalized_dy * (vertical_fov / 2.0f); //  俯仰角Invert Y-axis

    return {yaw, pitch};// 返回角度对
}
// 安全的角度计算函数(带参数验证)
std::pair<float, float> safe_calculate_angles(
    float dx, float dy,
    int image_width, int image_height,
    float horizontal_fov, float vertical_fov)
{
    // Parameter validation 参数验证
    if (image_width <= 0 || image_height <= 0)
        throw std::invalid_argument("Invalid image dimensions");
    
    if (horizontal_fov <= 0 || vertical_fov <= 0)
        throw std::invalid_argument("FOV values must be positive");

    // Calculate normalized offsets (add clamp to ensure safe range)// 限制偏移量在合理范围内
    const float clamped_dx = std::clamp(dx, -image_width/2.0f, image_width/2.0f);
    const float clamped_dy = std::clamp(dy, -image_height/2.0f, image_height/2.0f);
    
    // Reuse basic calculation logic调用基本计算函数
    auto [yaw, pitch] = calculate_angles_from_offsets(
        clamped_dx, clamped_dy, 
        image_width, image_height,
        horizontal_fov, vertical_fov);

    // Constrain angle ranges限制角度范围
    yaw = std::clamp(yaw, -horizontal_fov/2, horizontal_fov/2);
    pitch = std::clamp(pitch, -vertical_fov/2, vertical_fov/2);

    return {yaw, pitch}; //返回安全的角度值
}
// 发布检测结果到DDS，检测对象列表
void publish_detection_results(const vector<Detection> objects,double image_width,double image_height,dds::pub::DataWriter<DetectionModule::DetectionResults> & writer) {
    DetectionModule::DetectionResults results;// 创建DDS结果对象
    // 遍历所有检测对象
    for (const auto& obj : objects) {
        DetectionModule::DetectionResult result;// 单个结果对象
     // 设置类别信息
        result.class_id(std::to_string(obj.class_id));
        result.class_name(CLASS_NAMES[obj.class_id]);
        result.score(obj.conf);// 置信度
       // 设置边界框坐标(左上和右下点)
        // Set detection box
        std::array<float, 4> box = {
            static_cast<float>(obj.bbox.x),
            static_cast<float>(obj.bbox.y),
            static_cast<float>(obj.bbox.x + obj.bbox.width),
            static_cast<float>(obj.bbox.y + obj.bbox.height)
        };
        result.box(box);
        // Calculate center point coordinates 计算中心点坐标
        const float u = obj.bbox.x + obj.bbox.width / 2.0f;
        const float v = obj.bbox.y + obj.bbox.height / 2.0f;
        float offset_x = u - image_width / 2;// x方向偏移
        float offset_y = v - image_height / 2; // y方向偏移

        // Set 3D coordinates设置3D坐标
        std::array<float, 3> xyz = {obj.XYZ.x, obj.XYZ.y, obj.XYZ.z};
        result.xyz(xyz);
        // Set offsets 设置像素偏移量
        std::array<float, 2> offset = {offset_x, offset_y};
        result.offset(offset);
            // 计算并设置FOV偏移角度
        auto yaw_pitch = safe_calculate_angles(offset_x, offset_y, image_width, image_height, 86.0f, 57.0f);
        std::array<float, 2> offset_fov = {yaw_pitch.first, yaw_pitch.second};
        result.offset_fov(offset_fov);
         // 添加到结果列表
        results.results().push_back(result);
    }

    writer.write(results);// 发布结果
    std::cout << "Published detection results" << std::endl;
}

// Image processing thread  图像处理线程函数  // 相机控制器引用// YOLO模型引用// DDS写入器// 是否显示图像的标志
void processing_loop(CameraController& camera,YOLO& model,dds::pub::DataWriter<DetectionModule::DetectionResults> & writer, string show_image_flag) {
    int specific_class_id = 0; //// 特定类别ID(0表示特殊处理)
    float conf_flag = 0.4;      // 置信度阈值
    // 创建对齐对象(将深度图对齐到彩色图)
    rs2::align align(RS2_STREAM_COLOR);
    // DetectionResults results;
    while(true) {
         // 检查相机连接状态
        if(!camera_connected) {
            std::this_thread::sleep_for(100ms);
             // 发布无效结果(表示无连接)
            DetectionModule::DetectionResults results;
            DetectionModule::DetectionResult result;
            result.class_id(std::to_string(-1));// 无效ID
            result.class_name("");                // 空名称
            result.score(-1.0f);                  // 无效分数
    
        // 设置无效的边界框
            std::array<float, 4> box = {
                static_cast<float>(-1),
                static_cast<float>(-1),
                static_cast<float>(-1),
                static_cast<float>(-1)
            };
            result.box(box);
            // 设置无效的3D坐标
            std::array<float, 3> xyz = {-1, -1,-1};
            result.xyz(xyz);
            // 设置无效的偏移量
            std::array<float, 2> offset = {-1,-1};
            result.offset(offset);
               // 设置无效的FOV偏移
            std::array<float, 2> offset_fov = {-1,-1};
            result.offset_fov(offset_fov);

            results.results().push_back(result);
            writer.write(results);// 发布无效结果
            continue;
        }
// 获取帧数据
        auto frameset = camera.poll_frames();
        if(!frameset) {
            camera_connected = false;// 标记相机断开
            continue;
        }

        // 对齐深度和彩色帧
        auto aligned = align.process(frameset);
        auto color_frame = aligned.get_color_frame();// 获取彩色帧
        auto depth_frame = aligned.get_depth_frame();// 获取深度帧

   // 转换为OpenCV格式
        cv::Mat image(cv::Size(640,480), CV_8UC3, 
                         (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat depth_image(cv::Size(640,480), CV_16UC1,(void*)depth_frame.get_data());
        if (image.empty()) break;// 检查空图像
        vector<Detection> objects;// 检测结果容器
        model.preprocess(image);  // 预处理图像
    // 执行推理并计时
        auto start = std::chrono::system_clock::now();
        model.infer();
        auto end = std::chrono::system_clock::now();
  // 后处理(过滤低置信度结果)
        model.postprocess(objects,depth_image,camera.intrinsics,conf_flag,specific_class_id);
        // 在图像上绘制检测结果  
        model.draw(image, objects);
            // 发布检测结果
        publish_detection_results(objects,image.cols,image.rows,writer);
       // 计算并打印推理时间
        auto tc = (double)std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.;
        printf("cost %2.4lf ms\n", tc);
        // 根据标志决定是否显示图像
        if (show_image_flag=="1")
        {
            imshow("prediction", image);
            if(cv::waitKey(1) == 27) break;
        }
    }
}
// 相机重连监控线程
void reconnect_monitor(CameraController& camera) {
    while(true) {
        // 如果已连接或达到最大重连次数，则等待
        if(camera_connected || reconnect_attempts >= MAX_RECONNECT) {
            std::this_thread::sleep_for(1s);
            continue;
        }
// 打印重连信息
        std::cout << "Attempting to reconnect (" << ++reconnect_attempts << "/" 
                  << MAX_RECONNECT << ")...\n";
// 先关闭相机
        camera.shutdown();
        // 尝试重新初始化
        if(camera.initialize()) {
            reconnect_attempts = 0;// 重置计数器
            camera_connected = true; // 标记为已连接
            std::cout << "Camera reconnected successfully! Serial number: " 
                      << camera.get_serial() << std::endl;
        } else {
            std::this_thread::sleep_for(2s);// 失败后等待
        }
    }
}
// 主函数
int main(int argc, char** argv)
{
     // 检查参数数量
    assert(argc >=2);
    // 获取模型文件路径
    const string engine_file_path{ argv[1] };
    string show_image_flag;
    // 处理可选参数(是否显示图像)
    if (argc==3)
    {
        show_image_flag= argv[2] ;
    }
    else
    {
       show_image_flag="0";// 默认不显示
    }
    // 检查是否是ONNX文件(需要转换)
    if (engine_file_path.find(".onnx") == std::string::npos) // 加载TensorRT引擎
    {
        YOLO model(engine_file_path, logger);
       // 初始化DDS
        dds::domain::DomainParticipant participant{0};// 创建域参与者
        // Register type and create Topic创建主题和质量策略
        dds::topic::qos::TopicQos topic_qos = dds::topic::qos::TopicQos();
        dds::topic::Topic<DetectionModule::DetectionResults> topic(participant, "detectionresults", topic_qos);
        // 创建发布者和数据写入器
        dds::pub::Publisher publisher{participant};
        dds::pub::DataWriter<DetectionModule::DetectionResults> writer{publisher, topic};
        // 初始化相机
        CameraController camera;
        if(!camera.initialize()) {
            std::cerr << "Initialization failed, exiting program" << std::endl;
            return 1;
        }
        camera_connected = true;// 标记为已连接

        // Start processing threads启动处理线程
        std::thread proc_thread(std::bind(processing_loop, std::ref(camera), std::ref(model),std::ref(writer),std::ref(show_image_flag)));
         // 启动重连监控线程
        std::thread reconnect_thread(reconnect_monitor, std::ref(camera));

        // Wait for threads to finish等待线程结束
        proc_thread.join();
        reconnect_thread.join();
    }else// 如果是ONNX文件，转换为TensorRT引擎
    {
        std::cout << "Converting onnx to engine..." <<std::endl;
        YOLO model(engine_file_path, logger);
    }
    return 0;
}
