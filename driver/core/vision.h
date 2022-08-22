#ifndef DRIVER_VISION_H
#define DRIVER_VISION_H

#include <cmath>
#include <opencv2/opencv.hpp>
#include <vector>

#include "communication.h"

using namespace std;

class Vision {
   private:
    void contrast_normalization(cv::Mat);
    bool laplacian_check(cv::Mat);

   public:
    sign_item_t process(cv::Mat);
    float distance(cv::Point);
};

#endif  // DRIVER_VISION_H
