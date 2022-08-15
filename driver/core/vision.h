#ifndef DRIVER_VISION_H
#define DRIVER_VISION_H

#include <opencv2/opencv.hpp>
#include <vector>

using namespace std;

struct sign_info {
	cv::Mat cropped = cv::Mat();
	int area = 0;
	cv::Point center_position = cv::Point(0, 0);
};
typedef struct sign_info sign_info;

class Vision {

public:
	void sign_finding(cv::Mat&, sign_info&);
};

#endif  // DRIVER_VISION_H
