#ifndef DRIVER_VISION_H
#define DRIVER_VISION_H

#include <opencv2/opencv.hpp>
#include <vector>

using namespace std;

class Vision {
   public:
    void process(cv::Mat&, cv::Mat&);
};

#endif  // DRIVER_VISION_H
