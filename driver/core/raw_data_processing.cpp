#include "raw_data_processing.h"

#define CON_MAX 1800
#define CON_MIN 650
#define CON_LEN 100

cv::Mat raw_data_processing::emphasize_first(const cv::Mat first,
                                             const cv::Mat second,
                                             const cv::Mat third) {
    int mn = 255, mx = 0;
    const int H = first.rows, W = first.cols;
    cv::Mat ret = first.clone();
    for (int i = 0; i < H; ++i) {
        for (int j = 0; j < W; ++j) {
            // float weighted = ((int)second.at<uchar>(i, j) +
                              // (int)(third.at<uchar>(i, j)) / 2) *
                             // 1.45;
			float mx_channel = (float)std::max( (int)second.at<uchar>(i, j),
												(int)third.at<uchar>(i, j));
			mx_channel *= 1.38;
            if (int(first.at<uchar>(i, j)) > mx_channel) {
                // ret.at<uchar>(i, j) -= weighted;
            } else {
                ret.at<uchar>(i, j) = 0;
            }
            mn = std::min(mn, int(first.at<uchar>(i, j)));
            mx = std::max(mx, int(first.at<uchar>(i, j)));
        }
    }
    cv::normalize(ret, ret, 255, 0, cv::NORM_INF);
    return ret;
}
using Contours = std::vector<std::vector<cv::Point>>;

std::vector<raw_data_processing::rect_info>
raw_data_processing::draw_contours_and_rectangle(
    const std::vector<cv::Mat>& imgs) {
    std::vector<raw_data_processing::rect_info> ret;
    for (const auto& img : imgs) {
        threshold( img, img, 100, 255, 0);
		// input, output, val, max, 0->binary, 1-> inverted binary
        Contours contours;
        std::vector<cv::Vec4i> hierarchy;

        cv::findContours(img, contours, hierarchy, cv::RETR_EXTERNAL,
                         cv::CHAIN_APPROX_NONE);
        for (int i = 0; i < contours.size(); ++i) {
            if (contours[i].size() > CON_MIN and contours[i].size() < CON_MAX) {
                approxPolyDP(cv::Mat(contours[i]), contours[i], 3, true);

                raw_data_processing::rect_info inf;
                // inf.con = contours[i];
                double area = cv::contourArea(contours[i], false);
                inf.contour_area = int(area);
                inf.rect = cv::boundingRect(contours[i]);
				if (std::min(inf.rect.height, inf.rect.width) < CON_LEN) { continue; }
                cv::Moments M = cv::moments(contours[i]);
                cv::Point center(M.m10 / M.m00, M.m01 / M.m00);
                inf.contour_center = center;
				// app_contours.push_back(contours[i]);
                ret.push_back(inf);
            }
        }
    }
    return ret;
}

cv::Mat raw_data_processing::white_filter(const cv::Mat img) {
    cv::Mat ret;
    cv::inRange(img, cv::Scalar(200, 200, 200), cv::Scalar(255, 255, 255), ret);
    int morph_size1 = 8, morph_size2 = 4;
    cv::Mat element1 = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(2 * morph_size1 + 1, 2 * morph_size1 + 1),
        cv::Point(morph_size1, morph_size1));
    cv::Mat element2 = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(2 * morph_size2 + 1, 2 * morph_size2 + 1),
        cv::Point(morph_size2, morph_size2));
    // For Dilation
    cv::dilate(ret, ret, element1, cv::Point(-1, -1), 1);
    // For Erosion
    cv::erode(ret, ret, element2, cv::Point(-1, -1), 1);
    return ret;
}
