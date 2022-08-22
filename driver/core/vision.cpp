#include "vision.h"

#define blurThreshold 15.0

class CON_PARM_t {
   public:
    CON_PARM_t() {};
    CON_PARM_t(const int thr, const int min, const int max, const int wid, const int hei):
        THRES(thr), CON_MIN(min), CON_MAX(max), RECT_WIDTH(wid), RECT_HEIGHT(hei){};
    int THRES = 0;
    int CON_MIN = 0;
    int CON_MAX = 0;
    int RECT_WIDTH = 0;
    int RECT_HEIGHT = 0;
};

static std::unordered_map<char, CON_PARM_t> CON_PARM = {
    //THRES, MIN,  MAX, WID, HEI
    std::make_pair('R',CON_PARM_t(100, 650, 2000, 150, 150)),
    std::make_pair('B',CON_PARM_t(100, 650, 2000, 150, 100)),
    std::make_pair('W',CON_PARM_t(100, 500, 1100, 100, 100)) };

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

    vector<cv::Mat> HSV_channels = cvt_HSV(image); // [0]:H, [1]:S, [2]:V

    cv::Mat thres;
    cv::threshold(HSV_channels[1]*HSV_channels[2], thres, 0.23, 255, cv::THRESH_BINARY);

    vector<vector<cv::Point>> contours = find_contours(thres);

    return result;
}

vector<cv::Mat> Vision::cvt_HSV(cv::Mat image) {
    cv::Mat img_hsv;
    cv::cvtColor(image, img_hsv, cv::COLOR_BGR2HSV);

    vector<cv::Mat> channels;
    split(img_hsv, channels); // H S V

    cv::normalize(channels[0],channels[0], 0, 1, cv::NORM_MINMAX);
    cv::normalize(channels[1],channels[1], 0, 1, cv::NORM_MINMAX);
    cv::normalize(channels[2],channels[2], 0, 1, cv::NORM_MINMAX);

    return channels;
}


std::vector<std::vector<cv::Point>> Vision::find_contours(cv::Mat image) {
    cv::Mat copy = image.clone();
    cv::threshold(image, copy, 100, 255, 0);
    //input, output, val, max, 0->binary, 1-> inverted binary
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(copy, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    return contours;
}

pair<cv::Rect, cv::Point> Vision::find_rectangle(cv::Mat image, vector<vector<cv::Point>> contours) {
    cv::Rect rect;
    cv::Point contour_center;
    for (int i = 0; i < contours.size(); ++i) {
        if (contours[i].size() > 300 && contours[i].size() < 3000) {
            approxPolyDP(cv::Mat(contours[i]), contours[i], 3, true);

            //inf.con = contours[i];
            double area = cv::contourArea(contours[i], false);
            double len = contours[i].size();
            double min_area = len * len * 4 / 100, max_area = len * len / 18;
            cv::Rect rect = cv::boundingRect(contours[i]);
            if (rect.height < 50 && rect.width < 50) { continue; }
            if (area > max_area || area < min_area) { std::cout << "D" << std::endl; continue; }
            cv::Moments M = cv::moments(contours[i]);
            cv::Point center(M.m10 / M.m00, M.m01 / M.m00);
            contour_center = center;
        }
    }
    return make_pair(rect, contour_center);
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
