#ifndef DRIVER_VISION_H
#define DRIVER_VISION_H

#include "communication.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>

using namespace std;

class Vision {
public:
    sign_item_t process(cv::Mat);
    float distance(cv::Point);
};

#endif  // DRIVER_VISION_H
