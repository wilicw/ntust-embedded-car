#ifndef DRIVER_VISION_H
#define DRIVER_VISION_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>

using namespace std;

typedef struct {
    cv::Mat cropped;
    int area;
    cv::Point center_position;
    float distance;
} sign_info_t;

class Vision {
public:
    sign_info_t processing(cv::Mat);
    float distance(cv::Point);
};

#endif  // DRIVER_VISION_H
