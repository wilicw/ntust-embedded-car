#include "vision.h"

float Vision::distance(cv::Point center) {
    constexpr int w = 640, h = 480;
    constexpr double camera_height = 11.5;
    constexpr double camera_near = 10.0;

    const double thetaEnd = atan(camera_height / camera_near);
    const double thetaStart = thetaEnd - 0.6920252660101123;  // By camera DFOV
    double theta = thetaStart + (thetaEnd - thetaStart) * center.y / h;

    double thetasq = tan(theta);
    thetasq = thetasq * thetasq;

    double etaStart = atan((-7 / camera_height) * sqrt(thetasq / (1 + thetasq)));
    double etaEnd = -etaStart;
    double eta = etaStart + (etaEnd - etaStart) * center.x / w;

    double etasq = tan(eta);
    etasq = etasq * etasq;

    double result = camera_height * sqrt(1 / thetasq + etasq + etasq / thetasq);
    return result;
}

sign_item_t Vision::process(cv::Mat image) {
    const static sign_item_t empty_result = {
        .center = nullptr,
        .cropped = nullptr,
    };

    sign_item_t result = {
        .center = nullptr,
        .cropped = nullptr,
    };

    if (!this->laplacian_check(image)) return empty_result;

    cv::copyMakeBorder(image, image, 10, 10, 10, 10, cv::BORDER_CONSTANT);

    return result;
}
bool Vision::laplacian_check(cv::Mat image) {
    constexpr int blurry_threshold = 17;
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_RGB2GRAY);
    cv::Laplacian(gray, gray, CV_64F);
    cv::Scalar mean, stddev;
    cv::meanStdDev(gray, mean, stddev, cv::Mat());
    double variance = stddev.val[0] * stddev.val[0];
    return variance > blurry_threshold;
}

void Vision::contrast_normalization(cv::Mat image) {
}
