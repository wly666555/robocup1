#include <sys/stat.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include "YOLO.h"

#include <thread>
#include <atomic>
#include <queue>
#include <mutex>
#include "dds/DetectionModule.hpp"
#include <dds/dds.hpp>
#include"common.h"
/**
 * @brief Setting up Tensorrt logger
*/
class Logger : public nvinfer1::ILogger {
    void log(Severity severity, const char* msg) noexcept override {
        // Only output logs with severity greater than warning
        if (severity <= Severity::kWARNING)
            std::cout << msg << std::endl;
    }
}logger;

float image_width = 640;
float image_height = 480; 
float init_fps = 30;
float horizontal_fov = 86;
float vertical_fov = 57;

// Camera reconnection logic
class CameraController {
    rs2::pipeline pipe;
    rs2::config cfg;
    std::string serial;
    
    const int width = image_width, height = image_height, fps = init_fps;
    
public:
    rs2_intrinsics intrinsics;
    CameraController() {
        cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, fps);
        cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);
    }

    bool initialize() {
        try {
            auto profile = pipe.start(cfg);
            auto stream = profile.get_stream(RS2_STREAM_COLOR)
                          .as<rs2::video_stream_profile>();
            intrinsics = stream.get_intrinsics();
            serial = profile.get_device().get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            return true;
        } catch (const rs2::error& e) {
            std::cerr << "Camera init failed: " << e.what() << std::endl;
            return false;
        }
    }

    void shutdown() {
        try { pipe.stop(); } 
        catch (...) {} // Safely stop the pipeline
    }

    rs2::frameset poll_frames(int max_retries = 100) {
        for (int i = 0; i < max_retries; ++i) {
            rs2::frameset frames;
            if (pipe.poll_for_frames(&frames)) { // Use poll_for_frames
                return frames;
            }
            std::this_thread::sleep_for(10ms);
        }
        return {}; // Return empty frame to indicate failure
    }

    std::string get_serial() const { return serial; }
};

// Global control variables
std::atomic<bool> camera_connected{false};
std::atomic<int> reconnect_attempts{0};
constexpr int MAX_RECONNECT = 500;

std::pair<float, float> calculate_angles_from_offsets(
    float dx, 
    float dy,
    int image_width,
    int image_height,
    float horizontal_fov,
    float vertical_fov) 
{
    // Normalize pixel offsets
    const float normalized_dx = dx / static_cast<float>(image_width / 2.0f);
    const float normalized_dy = dy / static_cast<float>(image_height / 2.0f);

    // Calculate angle offsets based on FOV
    const float yaw = normalized_dx * (horizontal_fov / 2.0f);
    const float pitch = -normalized_dy * (vertical_fov / 2.0f); // Invert Y-axis

    return {yaw, pitch};
}

std::pair<float, float> safe_calculate_angles(
    float dx, float dy,
    int image_width, int image_height,
    float horizontal_fov, float vertical_fov)
{
    // Parameter validation
    if (image_width <= 0 || image_height <= 0)
        throw std::invalid_argument("Invalid image dimensions");
    
    if (horizontal_fov <= 0 || vertical_fov <= 0)
        throw std::invalid_argument("FOV values must be positive");

    // Calculate normalized offsets (add clamp to ensure safe range)
    const float clamped_dx = std::clamp(dx, -image_width/2.0f, image_width/2.0f);
    const float clamped_dy = std::clamp(dy, -image_height/2.0f, image_height/2.0f);
    
    // Reuse basic calculation logic
    auto [yaw, pitch] = calculate_angles_from_offsets(
        clamped_dx, clamped_dy, 
        image_width, image_height,
        horizontal_fov, vertical_fov);

    // Constrain angle ranges
    yaw = std::clamp(yaw, -horizontal_fov/2, horizontal_fov/2);
    pitch = std::clamp(pitch, -vertical_fov/2, vertical_fov/2);

    return {yaw, pitch};
}

void publish_detection_results(const vector<Detection> objects,double image_width,double image_height,dds::pub::DataWriter<DetectionModule::DetectionResults> & writer) {
    DetectionModule::DetectionResults results;
    for (const auto& obj : objects) {
        DetectionModule::DetectionResult result;
        result.class_id(std::to_string(obj.class_id));
        result.class_name(CLASS_NAMES[obj.class_id]);
        result.score(obj.conf);

        // Set detection box
        std::array<float, 4> box = {
            static_cast<float>(obj.bbox.x),
            static_cast<float>(obj.bbox.y),
            static_cast<float>(obj.bbox.x + obj.bbox.width),
            static_cast<float>(obj.bbox.y + obj.bbox.height)
        };
        result.box(box);
        // Calculate center point coordinates
        const float u = obj.bbox.x + obj.bbox.width / 2.0f;
        const float v = obj.bbox.y + obj.bbox.height / 2.0f;
        float offset_x = u - image_width / 2;
        float offset_y = v - image_height / 2;
        // Set 3D coordinates
        std::array<float, 3> xyz = {obj.XYZ.x, obj.XYZ.y, obj.XYZ.z};
        result.xyz(xyz);
        // Set offsets
        std::array<float, 2> offset = {offset_x, offset_y};
        result.offset(offset);
        auto yaw_pitch = safe_calculate_angles(offset_x, offset_y, image_width, image_height, 86.0f, 57.0f);
        std::array<float, 2> offset_fov = {yaw_pitch.first, yaw_pitch.second};
        result.offset_fov(offset_fov);
        results.results().push_back(result);
    }

    writer.write(results);
    std::cout << "Published detection results" << std::endl;
}

// Image processing thread
void processing_loop(CameraController& camera,YOLO& model,dds::pub::DataWriter<DetectionModule::DetectionResults> & writer, string show_image_flag) {
    int specific_class_id = 0; // 0 means special processing for class id 0, only keeping the highest confidence result
    float conf_flag = 0.4;     // Results with confidence below this value will not be saved
    rs2::align align(RS2_STREAM_COLOR);
    // DetectionResults results;
    while(true) {
        if(!camera_connected) {
            std::this_thread::sleep_for(100ms);
            DetectionModule::DetectionResults results;
            DetectionModule::DetectionResult result;
            result.class_id(std::to_string(-1));
            result.class_name("");
            result.score(-1.0f);
    
            // Set detection box
            std::array<float, 4> box = {
                static_cast<float>(-1),
                static_cast<float>(-1),
                static_cast<float>(-1),
                static_cast<float>(-1)
            };
            result.box(box);
            // Set 3D coordinates
            std::array<float, 3> xyz = {-1, -1,-1};
            result.xyz(xyz);
            // Set offsets
            std::array<float, 2> offset = {-1,-1};
            result.offset(offset);
            std::array<float, 2> offset_fov = {-1,-1};
            result.offset_fov(offset_fov);
            results.results().push_back(result);
            writer.write(results);
            continue;
        }

        auto frameset = camera.poll_frames();
        if(!frameset) {
            camera_connected = false;
            continue;
        }

        // Align depth and color frames
        auto aligned = align.process(frameset);
        auto color_frame = aligned.get_color_frame();
        auto depth_frame = aligned.get_depth_frame();

        // Convert to OpenCV matrices
        cv::Mat image(cv::Size(640,480), CV_8UC3, 
                         (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat depth_image(cv::Size(640,480), CV_16UC1,(void*)depth_frame.get_data());
        if (image.empty()) break;
        vector<Detection> objects;
        model.preprocess(image);

        auto start = std::chrono::system_clock::now();
        model.infer();
        auto end = std::chrono::system_clock::now();

        model.postprocess(objects,depth_image,camera.intrinsics,conf_flag,specific_class_id);
        model.draw(image, objects);
        publish_detection_results(objects,image.cols,image.rows,writer);

        auto tc = (double)std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.;
        printf("cost %2.4lf ms\n", tc);
        if (show_image_flag=="1")
        {
            imshow("prediction", image);
            if(cv::waitKey(1) == 27) break;
        }
    }
}

void reconnect_monitor(CameraController& camera) {
    while(true) {
        if(camera_connected || reconnect_attempts >= MAX_RECONNECT) {
            std::this_thread::sleep_for(1s);
            continue;
        }

        std::cout << "Attempting to reconnect (" << ++reconnect_attempts << "/" 
                  << MAX_RECONNECT << ")...\n";
        
        camera.shutdown();
        if(camera.initialize()) {
            reconnect_attempts = 0;
            camera_connected = true;
            std::cout << "Camera reconnected successfully! Serial number: " 
                      << camera.get_serial() << std::endl;
        } else {
            std::this_thread::sleep_for(2s);
        }
    }
}

int main(int argc, char** argv)
{
    assert(argc >=2);
    const string engine_file_path{ argv[1] };
    string show_image_flag;
    if (argc==3)
    {
        show_image_flag= argv[2] ;
    }
    else
    {
       show_image_flag="0";
    }
    
    if (engine_file_path.find(".onnx") == std::string::npos)
    {
        YOLO model(engine_file_path, logger);
        // DDS entities
        dds::domain::DomainParticipant participant{0};
        // Register type and create Topic
        dds::topic::qos::TopicQos topic_qos = dds::topic::qos::TopicQos();
        dds::topic::Topic<DetectionModule::DetectionResults> topic(participant, "detectionresults", topic_qos);
        dds::pub::Publisher publisher{participant};
        dds::pub::DataWriter<DetectionModule::DetectionResults> writer{publisher, topic};

        CameraController camera;
        if(!camera.initialize()) {
            std::cerr << "Initialization failed, exiting program" << std::endl;
            return 1;
        }
        camera_connected = true;

        // Start processing threads
        std::thread proc_thread(std::bind(processing_loop, std::ref(camera), std::ref(model),std::ref(writer),std::ref(show_image_flag)));
        std::thread reconnect_thread(reconnect_monitor, std::ref(camera));

        // Wait for threads to finish
        proc_thread.join();
        reconnect_thread.join();
    }else
    {
        std::cout << "Converting onnx to engine..." <<std::endl;
        YOLO model(engine_file_path, logger);
    }
    return 0;
}