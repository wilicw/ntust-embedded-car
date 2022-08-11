#include "vision.h"

void Vision::process(cv::Mat &frame, cv::Mat &final) {
    vector<cv::Mat> channel;
    cv::split(frame, channel);
    cv::Mat &red_frame = channel[0];
    cv::Mat thresh;

    cv::adaptiveThreshold(red_frame, thresh, 255,
                          cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 25,
                          25);
    //    cv::threshold(red_frame, thresh, 100, 255, cv::THRESH_BINARY);

    final = thresh;
}