#ifndef DRIVER_VISION_H
#define DRIVER_VISION_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>

using namespace std;

class Vision {
public:
    float distance(cv::Point);
};

#endif  // DRIVER_VISION_H
