#include "vision.h"

void Vision::process(cv::Mat &frame, cv::Mat &output) {
    vector<cv::Mat> channel;
    cv::split(frame, channel);
    cv::Mat &red_frame = channel[0];

    cv::adaptiveThreshold(red_frame, output, 255,
                          cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 25,
                          25);
    //    cv::threshold(red_frame, thresh, 100, 255, cv::THRESH_BINARY);
}