#pragma once

#include "NvInfer.h"
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <numeric> 
#include <optional>
using namespace nvinfer1;
using namespace std;
using namespace cv;

struct Detection
{
    float conf;
    int class_id;
    Rect bbox;
    cv::Point3f XYZ; 
};

class YOLO
{

public:

    YOLO(string model_path, nvinfer1::ILogger& logger);
    ~YOLO();

    void preprocess(Mat& image);
    void infer();
    void postprocess(vector<Detection>& output,const cv::Mat& depth_image,const rs2_intrinsics intrinsics,float conf_flag=0.4,int specific_class_id = -1);
    void draw(Mat& image, const vector<Detection>& output);
    
    std::optional<float> calculate_average_depth_in_box_no_outliers(const cv::Rect& box, const cv::Mat& depth_image,float std_factor=2.0f,int min_pixels_threshold=10);
   
    std::optional<cv::Point3f> compute_3d_coordinates(const cv::Rect& box, const cv::Mat&  depth_image,const rs2_intrinsics intrinsics);
    std::pair<float, float> calculate_stats(const std::vector<uint16_t>& data);
private:
    void init(std::string engine_path, nvinfer1::ILogger& logger);

    float* gpu_buffers[2];               
    float* cpu_output_buffer;

    cudaStream_t stream;
    IRuntime* runtime;                 
    ICudaEngine* engine;               
    IExecutionContext* context;       

    // Model parameters
    int input_w;
    int input_h;
    int num_detections;
    int detection_attribute_size;
    int num_classes = 80;
    const int MAX_IMAGE_SIZE = 4096 * 4096;
    float conf_threshold = 0.3f;
    float nms_threshold = 0.4f;
    vector<Scalar> colors;

    void build(std::string onnxPath, nvinfer1::ILogger& logger);
    bool saveEngine(const std::string& filename);
    string _file_path;
};