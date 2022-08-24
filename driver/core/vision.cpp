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
#ifdef DEBUG
    cv::Mat cloned = image.clone();
#endif
    const static sign_item_t empty_result = {nullptr, nullptr};

    cv::copyMakeBorder(image, image, 10, 10, 10, 10, cv::BORDER_CONSTANT);

    vector<cv::Mat> HSV_channels = this->cvt_HSV(image); // [0]:H, [1]:S, [2]:V

    ///--------------RED channel--------------

    pair<cv::Rect, cv::Point> red_pair = this->RED_CH(image, HSV_channels);

    ///--------------Blue channel--------------

    pair<cv::Rect, cv::Point> blue_pair = this->BLUE_CH(image, HSV_channels);

    ///--------------White channel--------------



    ///--------------Judge R B W three channel's size--------------
    if (red_pair.first.area() == 0 && blue_pair.first.area() == 0) {
        return empty_result;
    }

    vector<pair<cv::Rect, cv::Point>> pairs({red_pair, blue_pair});

    int pair_index = this->find_max_rect(pairs);
    if (pair_index == -1) return empty_result;

    cv::Mat cropped = image(pairs[pair_index].first);
    cv::resize(cropped, cropped, cv::Size(50, 50));

    if (!this->laplacian_check(cropped, 500)) return empty_result;

    sign_item_t ret;

    ret.cropped = new cv::Mat(cropped);
    ret.center = new cv::Point(pairs[pair_index].second);
    return ret;
}

bool Vision::laplacian_check(cv::Mat image, const int &threshold) {
    constexpr int blurry_threshold = 17;
    cv::Mat gray, laplacian;
    cv::cvtColor(image, gray, cv::COLOR_RGB2GRAY);
    cv::Laplacian(gray, laplacian, CV_64F);

    cv::Scalar mean, stddev;
    cv::meanStdDev(laplacian, mean, stddev, cv::Mat());

    double variance = stddev.val[0] * stddev.val[0];
    return variance > threshold;
}

vector<cv::Mat> Vision::cvt_HSV(cv::Mat image) {
    constexpr int NORM_MAX = 307200;
    constexpr int NORM_MIN = 0;
    cv::Mat img_hsv;
    //    img_hsv.convertTo(img_hsv, CV_32F);
    cv::cvtColor(image, img_hsv, cv::COLOR_BGR2HSV);

    vector<cv::Mat> channels;
    split(img_hsv, channels); // H S V

    channels[1].convertTo(channels[1], CV_32F);
    channels[2].convertTo(channels[2], CV_32F);

    cv::normalize(channels[1], channels[1], 0, 1, cv::NORM_MINMAX);
    cv::normalize(channels[2], channels[2], 0, 1, cv::NORM_MINMAX);

    return channels;
}



pair<cv::Rect, cv::Point> Vision::RED_CH(cv::Mat image, vector<cv::Mat> HSV_channels) {

    cv::Mat H_channels = this->H_filter(HSV_channels[0].clone(), 10, 165);
    H_channels = 255 - H_channels;

    cv::Mat Sthres;

    cv::threshold(HSV_channels[1], Sthres, 0.3, 255, cv::THRESH_TOZERO);

    cv::Mat SxV = Sthres.mul(HSV_channels[2]);

    cv::Mat SxVthres;

    cv::threshold(SxV, SxVthres, 0.3, 255, cv::THRESH_BINARY);

    H_channels.convertTo(H_channels, CV_32F);

    cv::Mat HxSxV = SxVthres.mul(H_channels);

    HxSxV.convertTo(HxSxV, CV_8UC1);

    vector<vector<cv::Point>> contours = this->find_contours(HxSxV);
    vector<vector<cv::Point>> hulls = this->contours_to_hulls(contours);
    pair<cv::Rect, cv::Point> ret = this->find_rectangle(hulls);

    return ret;
}

pair<cv::Rect, cv::Point> Vision::BLUE_CH(cv::Mat image, vector<cv::Mat> HSV_channels) {
    cv::Mat H_channel = this->H_filter(HSV_channels[0].clone(), 95, 125);

    cv::Mat Sthres;

    cv::threshold(HSV_channels[1], Sthres, 0.2, 255, cv::THRESH_TOZERO);

    cv::Mat SxV = Sthres.mul(HSV_channels[2]);

    cv::Mat SxVthres;
    cv::threshold(SxV, SxVthres, 0.01, 255, cv::THRESH_BINARY);

    H_channel.convertTo(H_channel, CV_32F);

    cv::Mat HxSxV = SxVthres.mul(H_channel);

    HxSxV.convertTo(HxSxV, CV_8UC1);

    vector<vector<cv::Point>> contours = this->find_contours(HxSxV);
    vector<vector<cv::Point>> hulls = this->contours_to_hulls(contours);
    pair<cv::Rect, cv::Point> ret = this->find_rectangle(hulls);

    return ret;
}


pair<cv::Rect, cv::Point> Vision::WHITE_CH(cv::Mat image, vector<cv::Mat> HSV) {
    return pair<cv::Rect, cv::Point>();
}


cv::Mat Vision::H_filter(cv::Mat H_channel, const int& min, const int& max) {
    cv::inRange(H_channel, min, max, H_channel);
    return H_channel;
}

cv::Mat Vision::dilation(cv::Mat image, const int &morph_size) {
    cv::Mat element1 = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(2 * morph_size + 1, 2 * morph_size + 1),
        cv::Point(morph_size, morph_size));
    cv::dilate(image, image, element1, cv::Point(-1, -1), 1);
    return image;
}

cv::Mat Vision::erosion(cv::Mat image, const int &morph_size) {
    cv::Mat element2 = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(2 * morph_size + 1, 2 * morph_size + 1),
        cv::Point(morph_size, morph_size));
    cv::erode(image, image, element2, cv::Point(-1, -1), 1);
    return image;
}

vector<vector<cv::Point>> Vision::find_contours(cv::Mat image) {
    cv::Mat copy = image.clone();
    vector<vector<cv::Point>> contours;
    vector<cv::Vec4i> hierarchy;
    cv::findContours(copy, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    return contours;
}

vector<vector<cv::Point>> Vision::contours_to_hulls(vector<vector<cv::Point>> contours){
    vector<vector<cv::Point>> hulls;
    for(auto& contour:contours){
        approxPolyDP(contour, contour, 8, true);
        vector<cv::Point> hull;
        cv::convexHull(contour, hull);
        hulls.push_back(hull);
    }
    return hulls;
}

pair<cv::Rect, cv::Point> Vision::find_rectangle(vector<vector<cv::Point>> hulls) {
    cv::Rect rect;
    cv::Point contour_center;

    for (auto &hull: hulls) {

        //find rectangle
        cv::Rect this_rect = cv::boundingRect(hull);
        if (this_rect.height < 100 && this_rect.width < 100) {
            continue;
        }

        if (this_rect.area() < rect.area()) {
            continue;
        }

        cv::Moments M = cv::moments(hull);
        contour_center = cv::Point(M.m10 / M.m00, M.m01 / M.m00);
        rect = cv::Rect(this_rect);
    }
    //    }
    return make_pair(rect, contour_center);
}


int Vision::find_max_rect(vector<pair<cv::Rect, cv::Point>> pairs) {
    int max_index = -1, max_area = 0;
    for (int i = 0; i < pairs.size(); i++) {
        if (pairs[i].first.area() > max_area) {
            max_area = pairs[i].first.area();
            max_index = i;
        }
    }
    return max_index;
}

void Vision::contrast_normalization(const cv::Mat image) {
}


