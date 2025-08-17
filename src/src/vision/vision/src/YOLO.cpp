#include "YOLO.h"
#include "logging.h"
#include "cuda_utils.h"
#include "macros.h"
#include "preprocess.h"
#include <NvOnnxParser.h>
#include "common.h"
#include <fstream>
#include <iostream>


static Logger logger;
#define isFP16 false
#define warmup true


YOLO::YOLO(string model_path, nvinfer1::ILogger& logger)
{
    this->_file_path = model_path;
    // Deserialize an engine
    if (model_path.find(".onnx") == std::string::npos)
    {
        init(model_path, logger);
    }
    // Build an engine from an onnx model
    else
    {
        build(model_path, logger);
        saveEngine(model_path);
    }

#if NV_TENSORRT_MAJOR < 10
    // Define input dimensions
    auto input_dims = engine->getBindingDimensions(0);
    input_h = input_dims.d[2];
    input_w = input_dims.d[3];
#else
    auto input_name = engine->getIOTensorName(0);
    auto input_dims = engine->getTensorShape(input_name);
    input_h = input_dims.d[2];
    input_w = input_dims.d[3];
#endif
}


void YOLO::init(std::string engine_path, nvinfer1::ILogger& logger)
{
    // Read the engine file
    ifstream engineStream(engine_path, ios::binary);
    engineStream.seekg(0, ios::end);
    const size_t modelSize = engineStream.tellg();
    engineStream.seekg(0, ios::beg);
    unique_ptr<char[]> engineData(new char[modelSize]);
    engineStream.read(engineData.get(), modelSize);
    engineStream.close();

    // Deserialize the tensorrt engine
    runtime = createInferRuntime(logger);
    engine = runtime->deserializeCudaEngine(engineData.get(), modelSize);
    context = engine->createExecutionContext();

    // Get input and output sizes of the model
    #if NV_TENSORRT_MAJOR < 10
        input_h = engine->getBindingDimensions(0).d[2];
        input_w = engine->getBindingDimensions(0).d[3];
        detection_attribute_size = engine->getBindingDimensions(1).d[1];
        num_detections = engine->getBindingDimensions(1).d[2];
    #else
        auto input_name = engine->getIOTensorName(0);
        auto output_name = engine->getIOTensorName(1);
        auto input_dims = engine->getTensorShape(input_name);
        auto output_dims = engine->getTensorShape(output_name);

        input_h = input_dims.d[2];
        input_w = input_dims.d[3];
        detection_attribute_size = output_dims.d[1];
        num_detections = output_dims.d[2];
    #endif
    num_classes = detection_attribute_size - 4;

    // Initialize input buffers
    cpu_output_buffer = new float[detection_attribute_size * num_detections];
    CUDA_CHECK(cudaMalloc(&gpu_buffers[0], 3 * input_w * input_h * sizeof(float)));
    // Initialize output buffer
    CUDA_CHECK(cudaMalloc(&gpu_buffers[1], detection_attribute_size * num_detections * sizeof(float)));

    cuda_preprocess_init(MAX_IMAGE_SIZE);

    CUDA_CHECK(cudaStreamCreate(&stream));


    if (warmup) {
        for (int i = 0; i < 10; i++) {
            this->infer();
        }
        printf("model warmup 10 times\n");
    }
}

YOLO::~YOLO()
{

    // Release stream and buffers
    if (this->_file_path.find(".onnx") == std::string::npos)
    {
        if (stream) {
            CUDA_CHECK(cudaStreamSynchronize(stream));
            CUDA_CHECK(cudaStreamDestroy(stream));
            stream = nullptr;
        }
        std::cout << "Free stream" << std::endl;
        // Free GPU buffers
        for (int i = 0; i < 2; i++) {
            if (gpu_buffers[i]) {
                CUDA_CHECK(cudaFree(gpu_buffers[i]));
                gpu_buffers[i] = nullptr;
            }
        }
    
        delete[] cpu_output_buffer;
        std::cout << "Free cpu_output_buffer" << std::endl;
        // Destroy the engine
        cuda_preprocess_destroy();
        delete context;
        delete engine;
        delete runtime;
    }

}

void YOLO::preprocess(Mat& image) {
    // Preprocessing data on gpu
    cuda_preprocess(image.ptr(), image.cols, image.rows, gpu_buffers[0], input_w, input_h, stream);
    CUDA_CHECK(cudaStreamSynchronize(stream));
}

void YOLO::infer()
{

#if NV_TENSORRT_MAJOR < 10
    context->enqueueV2((void**)gpu_buffers, stream, nullptr);
#else
    // Set input tensor address
    auto input_name = engine->getIOTensorName(0);
    auto output_name = engine->getIOTensorName(1);

    context->setInputTensorAddress(input_name, gpu_buffers[0]);
    context->setTensorAddress(output_name, gpu_buffers[1]);

    // Enqueue inference
    this->context->enqueueV3(this->stream);
#endif
}


std::pair<float, float> YOLO::calculate_stats(const std::vector<uint16_t>& data) {
    float sum = std::accumulate(data.begin(), data.end(), 0.0f);
    float mean = sum / data.size();
    
    float sq_sum = std::accumulate(data.begin(), data.end(), 0.0f,
        [mean](float acc, uint16_t v) {
            return acc + (v - mean) * (v - mean);
        });
    float std = std::sqrt(sq_sum / data.size());
    
    return {mean, std};
}



std::optional<cv::Point3f> YOLO::compute_3d_coordinates(
        const cv::Rect& box, const cv::Mat&  depth_image,const rs2_intrinsics intrinsics_)
    {
        auto avg_depth = calculate_average_depth_in_box_no_outliers(box, depth_image);
        if (!avg_depth)  return std::nullopt;
        if(*avg_depth < 0.9) return std::nullopt;

        // Calculate the center point coordinates
        const float u = box.x + box.width / 2.0f;
        const float v = box.y + box.height / 2.0f;
        
        // Project to 3D space
        float pixel[2] = {u, v};
        float point[3];
        rs2_deproject_pixel_to_point(point, &intrinsics_, pixel, *avg_depth);
        return cv::Point3f{point[0], point[1], point[2]}; 
    }

void YOLO::postprocess(vector<Detection>& output,const cv::Mat& depth_image,const rs2_intrinsics intrinsics,float conf_flag,int specific_class_id)
{
    // Memcpy from device output buffer to host output buffer
    CUDA_CHECK(cudaMemcpyAsync(cpu_output_buffer, gpu_buffers[1], num_detections * detection_attribute_size * sizeof(float), cudaMemcpyDeviceToHost, stream));
    CUDA_CHECK(cudaStreamSynchronize(stream));

    vector<Rect> boxes;
    vector<int> class_ids;
    vector<float> confidences;
    int image_rows = depth_image.rows;
    int image_cols = depth_image.cols;
    const float ratio_h = input_h / (float)image_rows;
    const float ratio_w = input_w / (float)image_cols;

    const Mat det_output(detection_attribute_size, num_detections, CV_32F, cpu_output_buffer);

    for (int i = 0; i < det_output.cols; ++i) {
        const Mat classes_scores = det_output.col(i).rowRange(4, 4 + num_classes);
        Point class_id_point;
        double score;
        minMaxLoc(classes_scores, nullptr, &score, nullptr, &class_id_point);

        if (score > conf_threshold) {
            const float cx = det_output.at<float>(0, i);
            const float cy = det_output.at<float>(1, i);
            const float ow = det_output.at<float>(2, i);
            const float oh = det_output.at<float>(3, i);
            Rect box;
            box.x = static_cast<int>((cx - 0.5 * ow));
            box.y = static_cast<int>((cy - 0.5 * oh));
            box.width = static_cast<int>(ow);
            box.height = static_cast<int>(oh);
            if (ratio_h > ratio_w)
            {
                box.x = box.x / ratio_w;
                box.y = (box.y - (input_h - ratio_w * image_rows) / 2) / ratio_w;
                box.width = box.width / ratio_w;
                box.height = box.height / ratio_w;
            }
            else
            {
                box.x = (box.x - (input_w - ratio_h * image_cols) / 2) / ratio_h;
                box.y = box.y / ratio_h;
                box.width = box.width / ratio_h;
                box.height = box.height / ratio_h;
            }
            if(class_id_point.y==0 && (box.width>120 || box.height>120))
            {
                continue;
            }
            boxes.push_back(box);
            class_ids.push_back(class_id_point.y);
            confidences.push_back(score);
        }
    }

    vector<int> nms_result;
    dnn::NMSBoxes(boxes, confidences, conf_threshold, nms_threshold, nms_result);
    if (specific_class_id != -1) {
        Detection specific_result;
        float specific_conf = 0.0f;
    
        for (int i = 0; i < nms_result.size(); i++) {
            Detection result;
            int idx = nms_result[i];
            result.class_id = class_ids[idx];
            if(confidences[idx] < conf_flag) continue;
            result.conf = confidences[idx];
            result.bbox = boxes[idx];
    
            // Check if compute_3d_coordinates return value is empty
            auto coordinates = compute_3d_coordinates(boxes[idx], depth_image, intrinsics);
            if (coordinates.has_value()) {
                result.XYZ = *coordinates; 
    
                if (result.class_id == specific_class_id) {
                    // Update specific_result to the result with the highest confidence
                    if (result.conf > specific_conf) {
                        specific_result = result;
                        specific_conf = result.conf;
                    }
                } else {
                    // The result of non-specific_class_id is directly added to output
                    output.push_back(result);
                }
            }
        }
    
        // If a result of specific_class_id is found, add it to output
        if (specific_conf > 0.0f) {
            output.push_back(specific_result);
        }
    } else {
        // If specific_class_id == -1, process all results directly
        for (int i = 0; i < nms_result.size(); i++) {
            Detection result;
            int idx = nms_result[i];
            result.class_id = class_ids[idx];
            if(confidences[idx] < conf_flag) continue;
            result.conf = confidences[idx];
            result.bbox = boxes[idx];
    
            // Check if compute_3d_coordinates return value is empty
            auto coordinates = compute_3d_coordinates(boxes[idx], depth_image, intrinsics);
            if (coordinates.has_value()) {
                result.XYZ = *coordinates; 
                output.push_back(result);
            }
        }
    }
    
}

void YOLO::build(std::string onnxPath, nvinfer1::ILogger& logger)
{
    std::cout << "Building engine from onnx model..." << std::endl;
    auto builder = createInferBuilder(logger);
    const auto explicitBatch = 1U << static_cast<uint32_t>(NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    INetworkDefinition* network = builder->createNetworkV2(explicitBatch);
    IBuilderConfig* config = builder->createBuilderConfig();
    if (isFP16)
    {
        config->setFlag(BuilderFlag::kFP16);
    }
    nvonnxparser::IParser* parser = nvonnxparser::createParser(*network, logger);
    bool parsed = parser->parseFromFile(onnxPath.c_str(), static_cast<int>(nvinfer1::ILogger::Severity::kINFO));
    IHostMemory* plan{ builder->buildSerializedNetwork(*network, *config) };

    runtime = createInferRuntime(logger);

    engine = runtime->deserializeCudaEngine(plan->data(), plan->size());

    context = engine->createExecutionContext();

    delete network;
    delete config;
    delete parser;
    delete plan;
    std::cout << "Build engine from onnx model success" << std::endl;
}

bool YOLO::saveEngine(const std::string& onnxpath)
{
    std::cout << "Saving engine to file..." << std::endl;
    // Create an engine path from onnx path
    std::string engine_path;
    size_t dotIndex = onnxpath.find_last_of(".");
    if (dotIndex != std::string::npos) {
        engine_path = onnxpath.substr(0, dotIndex) + ".engine";
    }
    else
    {
        return false;
    }

    // Save the engine to the path
    if (engine)
    {
        nvinfer1::IHostMemory* data = engine->serialize();
        std::ofstream file;
        file.open(engine_path, std::ios::binary | std::ios::out);
        if (!file.is_open())
        {
            std::cout << "Create engine file" << engine_path << " failed" << std::endl;
            return 0;
        }
        file.write((const char*)data->data(), data->size());
        file.close();

        delete data;
    }
    std::cout << "Create engine file" << engine_path << " success" << std::endl;
    return true;
}

void YOLO::draw(Mat& image, const vector<Detection>& output)
{
    for (int i = 0; i < output.size(); i++)
    {
        auto detection = output[i];
        auto box = detection.bbox;
        auto class_id = detection.class_id;
        auto conf = detection.conf;
        cv::Scalar color = cv::Scalar(COLORS[class_id][0], COLORS[class_id][1], COLORS[class_id][2]);
        rectangle(image, Point(box.x, box.y), Point(box.x + box.width, box.y + box.height), color, 3);
        // Detection box text
        string class_string = CLASS_NAMES[class_id] + ' ' + to_string(conf).substr(0, 4);
        Size text_size = getTextSize(class_string, FONT_HERSHEY_DUPLEX, 1, 2, 0);
        Rect text_rect(box.x, box.y - 40, text_size.width + 10, text_size.height + 20);
        rectangle(image, text_rect, color, FILLED);
        putText(image, class_string, Point(box.x + 5, box.y - 10), FONT_HERSHEY_DUPLEX, 1, Scalar(0, 0, 0), 2, 0);
    }
}

std::optional<float> YOLO::calculate_average_depth_in_box_no_outliers(
    const cv::Rect& box, 
    const cv::Mat& depth_image,
    float std_factor,
    int min_pixels_threshold)
{
    const float meter_scale_ = 0.001f; // Convert millimeters to meters
    const int radius = 5; // Circle radius

    // Ensure depth image is valid
    if (depth_image.empty() || depth_image.type() != CV_16UC1) {
        return std::nullopt;
    }

    // Ensure ROI is within image bounds
    cv::Rect valid_box = box & cv::Rect(0, 0, depth_image.cols, depth_image.rows);
    if (valid_box.area() == 0) return std::nullopt;

    // Calculate box center point
    cv::Point center(box.x + box.width/2, box.y + box.height/2);
    
    // Create circular mask
    cv::Mat mask = cv::Mat::zeros(depth_image.size(), CV_8UC1);
    cv::circle(mask, center, radius, cv::Scalar(255), -1);

    // Extract valid depth values
    std::vector<uint16_t> valid_depths;
    for (int y = valid_box.y; y < valid_box.y + valid_box.height; ++y) {
        for (int x = valid_box.x; x < valid_box.x + valid_box.width; ++x) {
            if (mask.at<uchar>(y, x) > 0) { // Inside circle
                uint16_t depth = depth_image.at<uint16_t>(y, x);
                if (depth > 0) valid_depths.push_back(depth);
            }
        }
    }

    if (valid_depths.empty()) return std::nullopt;

    // Calculate statistics
    auto [mean, std] = calculate_stats(valid_depths);

    // Filter outliers
    const float lower = mean - std_factor * std;
    const float upper = mean + std_factor * std;
    std::vector<uint16_t> filtered;
    std::copy_if(valid_depths.begin(), valid_depths.end(),
                 std::back_inserter(filtered),
                 [lower, upper](uint16_t v) { 
                     return v >= lower && v <= upper; 
                 });

    if (filtered.size() < min_pixels_threshold) 
        return std::nullopt;

    // Return average value (in meters)
    return std::accumulate(filtered.begin(), filtered.end(), 0.0f) 
           / filtered.size() * meter_scale_;
}

