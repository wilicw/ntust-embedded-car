#ifndef DRIVER_VISION_H
#define DRIVER_VISION_H

#include <opencv2/opencv.hpp>
#include <vector>

using namespace std;

typedef struct {
    cv::Mat cropped;
    int area = 0;
    cv::Point center_position = cv::Point(0, 0);
} sign_info_t;

class Vision {
   public:
    sign_info_t processing(cv::Mat);
};

#endif  // DRIVER_VISION_H
