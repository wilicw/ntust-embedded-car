#pragma once
#include<opencv2/opencv.hpp>
#include<string>
#include<vector>
#include"rectangles_information.h"

namespace raw_data_processing {
	cv::Mat& imgSharpen(const cv::Mat& img, char* arith);
	using Contour = std::vector< cv::Point>;
	class rect_info {
	public:
		int contour_area = 0;
		cv::Point contour_center = cv::Point(0, 0);
		cv::Rect rect = cv::Rect();
		Contour con;
	};
	cv::Mat padding(const cv::Mat&);
	void contrast_increasing(cv::Mat&, float, float);
	cv::Mat emphasize_first(const cv::Mat& first, const cv::Mat& second, const cv::Mat& third);
	cv::Mat white_filter(const cv::Mat&);
	std::vector<rect_info> draw_contours_and_rectangle(const std::vector<cv::Mat>&);
};
