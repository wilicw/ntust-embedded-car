#ifndef DRIVER_VISION_H
#define DRIVER_VISION_H

#include <opencv2/opencv.hpp>
#include <vector>

using namespace std;

class Vision {
#ifdef ENABLE_CV
   public:
    void process(cv::Mat&, cv::Mat&);
#endif
};

#endif  // DRIVER_VISION_H
