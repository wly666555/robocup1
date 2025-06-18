#include <gtest/gtest.h>
#include <gtest/gtest.h>
#include "booster_vision/model//detector.h"
#include "booster_vision/model//segmentor.h"
#include <opencv2/opencv.hpp>

using booster_vision::YoloV8Detector;
using booster_vision::YoloV8Segmentor;
using booster_vision::DetectionRes;
using booster_vision::SegmentationRes;

class YoloV8DetectorTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize the detector with a configuration node
        YAML::Node config = YAML::LoadFile("config.yaml");
        detector = YoloV8Detector::CreateYoloV8Detector(config["detection_model"]);
        segmentor = YoloV8Segmentor::CreateYoloV8Segmentor(config["segmentation_model"]);
    }

    std::shared_ptr<YoloV8Detector> detector;
    std::shared_ptr<YoloV8Segmentor> segmentor;
};

TEST_F(YoloV8DetectorTest, SegmentObjectsInRealImage) {
    // Set confidence threshold
    std::cout << "current threshold: " << detector->getConfidenceThreshold() << std::endl;

    // Load a real image
    cv::Mat realImage = cv::imread("real_image.jpg");

    // Perform object detection
    std::vector<SegmentationRes> objects;
    for (int i = 0; i < 10; i++)
        objects = segmentor->Inference(realImage);

    cv::Mat res_image = booster_vision::YoloV8Segmentor::DrawSegmentation(realImage, objects);
    cv::imwrite("segment_image_display.jpg", res_image);
}

// Test case for setting and getting confidence threshold
TEST_F(YoloV8DetectorTest, SetAndGetConfidenceThreshold) {
    // Set confidence threshold
    float threshold = 0.5f;
    detector->setConfidenceThreshold(threshold);

    // Get confidence threshold
    float retrievedThreshold = detector->getConfidenceThreshold();

    // Assert that the retrieved threshold matches the set threshold
    ASSERT_EQ(retrievedThreshold, threshold);
}

// Test case for detecting objects with a specific confidence threshold
TEST_F(YoloV8DetectorTest, DetectObjectsWithConfidenceThreshold) {
    // Set confidence threshold
    float threshold = 0.1f;
    detector->setConfidenceThreshold(threshold);

    // Test input image
    cv::Mat image = cv::imread("test_image.jpg");

    // Perform object detection
    std::vector<DetectionRes> objects = detector->Inference(image);

    // Assert that at least one object is detected
    ASSERT_GT(objects.size(), 0);
}

// Test case for detecting objects in an empty image
TEST_F(YoloV8DetectorTest, DetectObjectsInEmptyImage) {
    // Set confidence threshold
    float threshold = 0.5f;
    detector->setConfidenceThreshold(threshold);

    // Create an empty image
    cv::Mat emptyImage = cv::Mat::zeros(cv::Size(640, 480), CV_8UC3);

    // Perform object detection
    std::vector<DetectionRes> objects = detector->Inference(emptyImage);

    // Assert that no objects are detected
    ASSERT_EQ(objects.size(), 0);
}

// Test case for detecting objects in a real image
TEST_F(YoloV8DetectorTest, DetectObjectsInRealImage) {
    // Set confidence threshold
    std::cout << "current threshold: " << detector->getConfidenceThreshold() << std::endl;

    // Load a real image
    cv::Mat realImage = cv::imread("real_image.jpg");

    // Perform object detection
    std::vector<DetectionRes> objects = detector->Inference(realImage);

    for (auto &det_res : objects) {
        std::cout << "class_id: " << det_res.class_id << ", confidence: " << det_res.confidence << std::endl;
        std::cout << "bbox: " << det_res.bbox << std::endl;
        cv::rectangle(realImage, det_res.bbox, cv::Scalar(255, 255, 255), 2);
    }

    cv::imwrite("real_image_output.jpg", realImage);

    // Assert that objects are detected
    ASSERT_GT(objects.size(), 0);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}