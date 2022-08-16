#pragma once
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace raw_data_processing {
using Contour = std::vector<cv::Point>;
class rect_info {
   public:
    int contour_area = 0;
    cv::Point contour_center = cv::Point(0, 0);
    cv::Rect rect = cv::Rect();
    Contour con;
};

cv::Mat emphasize_first(const cv::Mat first, const cv::Mat second,
                        const cv::Mat third);
cv::Mat white_filter(const cv::Mat);
std::vector<rect_info> draw_contours_and_rectangle(const std::vector<cv::Mat>&);
};  // namespace raw_data_processing
