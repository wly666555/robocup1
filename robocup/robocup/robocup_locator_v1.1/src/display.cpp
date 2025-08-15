
#include <opencv2/opencv.hpp>

#include "display.h"
#include "types.h"

void DisplayBoard::drawFieldBoard() {

    int line_width = 5;

    cv::rectangle(debug_field, cv::Point(-fd.length / 2 + board_width / 2, -fd.width / 2 + board_height / 2), 
                                cv::Point(fd.length / 2 + board_width / 2, fd.width / 2 + board_height / 2), 
                                cv::Scalar(255, 255, 255), line_width);

    cv::rectangle(debug_field, cv::Point(-fd.length / 2 + board_width / 2, -fd.penaltyAreaWidth / 2 + board_height / 2), 
                                cv::Point(-fd.length / 2 + fd.penaltyAreaLength + board_width / 2, fd.penaltyAreaWidth / 2 + board_height / 2), 
                                cv::Scalar(255, 255, 255), line_width);

    cv::rectangle(debug_field, cv::Point(fd.length / 2 - fd.penaltyAreaLength + board_width / 2, -fd.penaltyAreaWidth / 2 + board_height / 2), 
                                cv::Point(fd.length / 2 + board_width / 2, fd.penaltyAreaWidth / 2 + board_height / 2), 
                                cv::Scalar(255, 255, 255), line_width);

    cv::rectangle(debug_field, cv::Point(-fd.length / 2 + board_width / 2, -fd.goalAreaWidth / 2 + board_height / 2), 
                                cv::Point(-fd.length / 2 + fd.goalAreaLength + board_width / 2, fd.goalAreaWidth / 2 + board_height / 2), 
                                cv::Scalar(255, 255, 255), line_width);

    cv::rectangle(debug_field, cv::Point(fd.length / 2 - fd.goalAreaLength + board_width / 2, -fd.goalAreaWidth / 2 + board_height / 2), 
                                cv::Point(fd.length / 2 + board_width / 2, fd.goalAreaWidth / 2 + board_height / 2), 
                                cv::Scalar(255, 255, 255), line_width);

    cv::circle(debug_field, cv::Point(0 + board_width / 2, 0 + board_height / 2), fd.circleRadius, cv::Scalar(255, 255, 255), line_width);
    cv::line(debug_field, cv::Point(board_width / 2, -fd.width / 2 + board_height / 2), 
                                cv::Point(board_width / 2, fd.width / 2 + board_height / 2), 
                                cv::Scalar(255, 255, 255), line_width);
}

void DisplayBoard::init(FieldDimensions _fd, int board_width, int board_height, float scale_factor) {
    
    fd = _fd;

    debug_field = cv::Mat(board_height, board_width, CV_8UC3, cv::Scalar(107, 176, 60));
    this -> board_width = board_width;
    this -> board_height = board_height;
    this -> scale_factor = scale_factor;

    fd.length *= scale_factor;
    fd.width *= scale_factor;
    fd.penaltyDist *= scale_factor;
    fd.goalWidth *= scale_factor;
    fd.circleRadius *= scale_factor;
    fd.penaltyAreaLength *= scale_factor;
    fd.penaltyAreaWidth *= scale_factor;
    fd.goalAreaLength *= scale_factor;
    fd.goalAreaWidth *= scale_factor;

    drawFieldBoard();
}

void DisplayBoard::display() {

    cv::imshow("DisplayBoard", debug_field);
    cv::waitKey();  
    
}

void DisplayBoard::clearMarkers() {
    markers.clear();
}

void DisplayBoard::addMarker(string type, double x, double y) {
    markers.push_back(FieldMarker{type[0], x, y, 0});
}
    
void DisplayBoard::displayMarkers() {
    cv::Mat debug_field_copy = debug_field.clone();

    for (auto &m : markers) {
        
        cv::Scalar color;
        switch(m.type) {
            case 'L':
                color = cv::Scalar(0, 0, 255);
                break;
            case 'T':
                color = cv::Scalar(0, 255, 0);
                break;
            case 'X':
                color = cv::Scalar(255, 0, 0);
                break;
            default:
                color = cv::Scalar(255, 255, 0);
        }
        cv::circle(debug_field_copy, cv::Point(m.x * scale_factor + board_width / 2, - m.y * scale_factor + board_height / 2), 10, color, -1);
    }

    cv::imshow("Markers", debug_field_copy);
    cv::waitKey(1);  
}

cv::Point DisplayBoard::rotateVector(int x, int y, float theta) {
    int new_x = x * cos(theta) + y * sin(theta);
    int new_y = -x * sin(theta) + y * cos(theta);
    return cv::Point(new_x, new_y);
}

void DisplayBoard::displayRobotPose(double x, double y, double theta) {

    cv::Mat debug_field_copy = debug_field.clone();
    
    // display robot position
    cv::Point center(x * scale_factor + board_width / 2, - y * scale_factor + board_height / 2);

    int vector_length = 10;
    auto pt0 = rotateVector(vector_length, vector_length, theta);
    auto pt1 = rotateVector(vector_length, -vector_length, theta);

    cv::line(debug_field_copy, center, center + pt0, cv::Scalar(0, 0, 255), 5);
    cv::line(debug_field_copy, center, center + pt1, cv::Scalar(0, 0, 255), 5);

    // display markers
    for (auto &m : markers) {
        
        cv::Scalar color;
        switch(m.type) {
            case 'L':
                color = cv::Scalar(255, 0, 0);
                break;
            case 'T':
                color = cv::Scalar(0, 255, 0);
                break;
            case 'X':
                color = cv::Scalar(0, 0, 255);
                break;
            default:
                color = cv::Scalar(255, 255, 0);
        }
        cv::circle(debug_field_copy, cv::Point(m.x * scale_factor + board_width / 2, - m.y * scale_factor + board_height / 2), 10, color, -1);
    }

    cv::imshow("DisplayBoard", debug_field_copy);
    cv::waitKey(1);  
}
