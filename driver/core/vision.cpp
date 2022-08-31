#include "vision.h"

const unordered_map<char, Vision::PARM_t> Vision::PARM =
    unordered_map<char, Vision::PARM_t>({{'R', {10, 150, 76, 51, 3}},
                                         {'B', {105, 130, 51, 17, 3}}});

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
    cv::Mat cloned = image;
#endif
    const static sign_item_t empty_result = {nullptr, nullptr};

    cv::copyMakeBorder(image, image, 2, 2, 2, 2, cv::BORDER_CONSTANT, cv::Scalar(0, 40, 0));

    vector<cv::Mat> HSV_channels = this->cvt_HSV(image);  // [0]:H, [1]:S, [2]:V

    cv::Mat out_RED_hsv;
    ///--------------RED channel--------------

    pair<cv::Rect, cv::Point> red_pair = this->RED_CH(image, HSV_channels, out_RED_hsv);

    ///--------------Blue channel--------------

    pair<cv::Rect, cv::Point> blue_pair = this->BLUE_CH(image, HSV_channels);

    ///--------------White channel--------------
    HSV_channels = this->cvt_HSV(image);  // [0]:H, [1]:S, [2]:V
    pair<cv::Rect, cv::Point> white_pair = this->WHITE_CH(image, HSV_channels, out_RED_hsv);

#ifdef DEBUG
    cv::waitKey(0);
    cv::destroyAllWindows();
#endif

    ///--------------Judge R B W three channel's size--------------
    if (red_pair.first.area() == 0 && blue_pair.first.area() == 0 && white_pair.first.area() == 0) {
        return empty_result;
    }

    vector<pair<cv::Rect, cv::Point>> pairs({red_pair, blue_pair, white_pair});

    int pair_index = this->find_max_rect(pairs);
    if (pair_index == -1) {
        return empty_result;
    }

    cv::Mat cropped = image(pairs[pair_index].first);
    cv::resize(cropped, cropped, cv::Size(50, 50));

    if (pair_index != 2) {
        double lap = this->laplacian(cropped);
        if (lap < 400) {
            return empty_result;
        }
    }

    sign_item_t ret;

    ret.cropped = new cv::Mat(cropped);
    ret.center = new cv::Point(pairs[pair_index].second);
    return ret;
}

double Vision::laplacian(cv::Mat image) {
    constexpr int blurry_threshold = 17;
    cv::Mat gray, laplacian;
    cv::cvtColor(image, gray, cv::COLOR_RGB2GRAY);
    cv::Laplacian(gray, laplacian, CV_64F);
    cv::Scalar mean, stddev;
    cv::meanStdDev(laplacian, mean, stddev, cv::Mat());
    return stddev.val[0] * stddev.val[0];
}

vector<cv::Mat> Vision::cvt_HSV(cv::Mat image) {
    cv::Mat img_hsv;
    cv::cvtColor(image, img_hsv, cv::COLOR_BGR2HSV);

    vector<cv::Mat> channels;
    split(img_hsv, channels);  // H S V
    return channels;
}

pair<cv::Rect, cv::Point> Vision::RED_CH(cv::Mat image, vector<cv::Mat> HSV_channels, cv::Mat& out_hsv) {
    cv::Mat img_H_channel = HSV_channels[0];
    cv::Mat img_S_channel = HSV_channels[1];
    cv::Mat img_V_channel = HSV_channels[2];

    cv::Mat H_channel = this->H_filter(img_H_channel, Vision::PARM.at('R').H_filter_MIN, Vision::PARM.at('R').H_filter_MAX);
    cv::bitwise_not(H_channel, H_channel);

    cv::Mat Sthres;

    cv::threshold(img_S_channel, Sthres, Vision::PARM.at('R').S_threshold, 255, cv::THRESH_TOZERO);

    cv::Mat SxV;
    cv::bitwise_and(Sthres, img_V_channel, SxV);

    cv::Mat SxVthres;
    cv::threshold(SxV, SxVthres, Vision::PARM.at('R').SxV_threshold, 255, cv::THRESH_BINARY);
    SxVthres = this->dilation(SxVthres, Vision::PARM.at('R').SxV_dilation_scale);
    SxVthres = this->erosion(SxVthres, Vision::PARM.at('R').SxV_dilation_scale);

    cv::Mat HxSxV;
    cv::bitwise_and(SxVthres, H_channel, HxSxV);

    out_hsv = HxSxV;

    vector<vector<cv::Point>> contours = this->find_contours(HxSxV);
    vector<vector<cv::Point>> hulls = this->contours_to_hulls(contours);
    pair<cv::Rect, cv::Point> ret = this->find_rectangle(hulls);

    return ret;
}

pair<cv::Rect, cv::Point> Vision::BLUE_CH(cv::Mat image, vector<cv::Mat> HSV_channels) {
    cv::Mat img_H_channel = HSV_channels[0];
    cv::Mat img_S_channel = HSV_channels[1];
    cv::Mat img_V_channel = HSV_channels[2];

    cv::Mat H_channel = this->H_filter(img_H_channel, Vision::PARM.at('B').H_filter_MIN, Vision::PARM.at('B').H_filter_MAX);

    cv::Mat Sthres;
    cv::threshold(img_S_channel, Sthres, Vision::PARM.at('B').S_threshold, 255, cv::THRESH_TOZERO);

    cv::Mat SxV;
    cv::bitwise_and(Sthres, img_V_channel, SxV);

    cv::Mat SxVthres;
    cv::threshold(SxV, SxVthres, Vision::PARM.at('B').SxV_threshold, 255, cv::THRESH_BINARY);
    SxVthres = this->dilation(SxVthres, Vision::PARM.at('B').SxV_dilation_scale);
    SxVthres = this->erosion(SxVthres, Vision::PARM.at('B').SxV_dilation_scale);

    cv::Mat HxSxV;
    cv::bitwise_and(SxVthres, H_channel, HxSxV);

    vector<vector<cv::Point>> contours = this->find_contours(HxSxV);
    vector<vector<cv::Point>> hulls = this->contours_to_hulls(contours);
    pair<cv::Rect, cv::Point> ret = this->find_rectangle(hulls);

    return ret;
}

pair<cv::Rect, cv::Point> Vision::WHITE_CH(cv::Mat image, vector<cv::Mat> HSV_channels, cv::Mat RED_hsv) {
    cv::Mat img_H_channel = HSV_channels[0];
    cv::Mat img_S_channel = HSV_channels[1];
    cv::Mat img_V_channel = HSV_channels[2];

    //------BLUE MASK------

    cv::Mat Blue_MASK_V, Blue_MASK_H, Blue_MASK;
    Blue_MASK_H = this->H_filter(img_H_channel, 90, 105);
    cv::threshold(img_V_channel, Blue_MASK_V, 160, 255, cv::THRESH_BINARY);
    cv::bitwise_and(Blue_MASK_H, Blue_MASK_V, Blue_MASK);

    //------white channel------

    cv::Mat WHITE_Sthres, WHITE_Vthres;

    // white S channel
    cv::threshold(img_S_channel, WHITE_Sthres, 28, 255, cv::THRESH_BINARY);
    cv::bitwise_not(WHITE_Sthres, WHITE_Sthres);

    // take white V channel
    cv::threshold(img_V_channel, WHITE_Vthres, 102, 255, cv::THRESH_BINARY);

    // multiply S V
    cv::Mat WHITE_SxV;
    cv::bitwise_and(WHITE_Sthres, WHITE_Vthres, WHITE_SxV);

    // OR with BLUE mask
    cv::Mat WHITE_SxV_blue;
    cv::bitwise_or(WHITE_SxV, Blue_MASK, WHITE_SxV_blue);

    // dilation SV
    cv::Mat WHITE_SxV_ero = this->erosion(WHITE_SxV_blue, 5);
    cv::Mat WHITE_SxV_orgin = this->dilation(WHITE_SxV_ero, 5);
    cv::Mat WHITE_SxV_dil = this->dilation(WHITE_SxV_orgin, 10);

    cv::Mat WHITE_SxV_diff;
    cv::subtract(WHITE_SxV_dil, WHITE_SxV_orgin, WHITE_SxV_diff);
    cv::Mat WHITE_SxV_diff_dil = this->dilation(WHITE_SxV_diff, 7);

    //------black channel------

    // black -> 0.16 * S + V < 58.65(23%) OR V < 38(15%)

    // V < 15%
    cv::Mat Black_Vthres;
    cv::threshold(img_V_channel, Black_Vthres, 38, 255, cv::THRESH_BINARY);
    cv::bitwise_not(Black_Vthres, Black_Vthres);

    // 0.16 * S + V < 58.65(23%)
    cv::Mat img_S_channel_32F, img_V_channel_32F;
    img_S_channel.convertTo(img_S_channel_32F, CV_32F);
    img_V_channel.convertTo(img_V_channel_32F, CV_32F);

    cv::Mat BLACK_016S_V = 0.16 * img_S_channel_32F + img_V_channel_32F;

    cv::Mat BLACK_SxVthres;
    cv::threshold(BLACK_016S_V, BLACK_SxVthres, 59, 255, cv::THRESH_BINARY);
    cv::bitwise_not(BLACK_SxVthres, BLACK_SxVthres);
    BLACK_SxVthres.convertTo(BLACK_SxVthres, CV_8UC1);

    // OR both togeter
    cv::bitwise_or(Black_Vthres, BLACK_SxVthres, BLACK_SxVthres);

    //dilation SV
    BLACK_SxVthres = this->dilation(BLACK_SxVthres, 5);

    //------sub red and black------

    //dilation red
    RED_hsv = this->dilation(RED_hsv, 12);
    cv::Mat R_B_diff;
    cv::subtract(BLACK_SxVthres, RED_hsv, R_B_diff);

    //------Black * White------
    cv::Mat BxW;
    cv::bitwise_and(R_B_diff, WHITE_SxV_diff_dil, BxW);

    //------judge sum val------
    int whitesum = cv::sum(BxW)[0];

    if (whitesum < 60000) {
        return pair<cv::Rect, cv::Point>();
    }

    //------judge if white is in black------

    cv::Mat R_B_diff_dil = this->dilation(R_B_diff, 7);
    cv::Mat anothor_BxW;
    cv::bitwise_and(R_B_diff_dil, WHITE_SxV_orgin, anothor_BxW);

    cv::Point white_point;
    cv::minMaxLoc(anothor_BxW, NULL, NULL, NULL, &white_point);

    cv::Mat Black_filled_img = WHITE_SxV_orgin;
    cv::floodFill(Black_filled_img, white_point, 127);

    cv::inRange(Black_filled_img, 120, 130, Black_filled_img);

    vector<vector<cv::Point>> black_countour;
    vector<cv::Vec4i> hierarchy;
    cv::findContours(Black_filled_img, black_countour, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);

    if (black_countour.size() != 1) {
        return pair<cv::Rect, cv::Point>();
    }

    //------fill white image------
    cv::minMaxLoc(BxW, NULL, NULL, NULL, &white_point);

    cv::Mat White_filled_img = WHITE_SxV_diff_dil;
    cv::floodFill(White_filled_img, white_point, 127);

    cv::inRange(White_filled_img, 120, 130, White_filled_img);

    //------BxW to find contours------
    cv::Mat BandW;
    cv::bitwise_and(White_filled_img, Black_filled_img, BandW);

    //------find contours and rectangle------
    vector<vector<cv::Point>> contours = this->find_contours(BandW);
    vector<vector<cv::Point>> hulls = this->contours_to_hulls(contours);
    pair<cv::Rect, cv::Point> ret = this->find_rectangle(hulls);

    //------judge rectangle area
    if (ret.first.area() > 75000) {
        return pair<cv::Rect, cv::Point>();
    }

    return ret;
}

cv::Mat Vision::H_filter(cv::Mat H_channel, const int& min, const int& max) {
    cv::Mat ret;
    cv::inRange(H_channel, min, max, ret);
    return ret;
}

cv::Mat Vision::dilation(cv::Mat image, const int& morph_size) {
    cv::Mat kernal = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(morph_size, morph_size));
    cv::Mat ret;
    cv::dilate(image, ret, kernal);
    return ret;
}

cv::Mat Vision::erosion(cv::Mat image, const int& morph_size) {
    cv::Mat kernal = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(morph_size, morph_size));
    cv::Mat ret;
    cv::erode(image, ret, kernal);
    return ret;
}

vector<vector<cv::Point>> Vision::find_contours(cv::Mat image) {
    cv::Mat copy = image.clone();
    vector<vector<cv::Point>> contours;
    vector<cv::Vec4i> hierarchy;
    cv::findContours(copy, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    return contours;
}

vector<vector<cv::Point>> Vision::contours_to_hulls(vector<vector<cv::Point>> contours) {
    vector<vector<cv::Point>> hulls;
    for (auto& contour : contours) {
        approxPolyDP(contour, contour, 8, true);
        // create hull array for convex hull points
        vector<cv::Point> hull;
        cv::convexHull(contour, hull);
        hulls.push_back(hull);
    }
    return hulls;
}

pair<cv::Rect, cv::Point> Vision::find_rectangle(vector<vector<cv::Point>> hulls) {
    cv::Rect rect;
    cv::Point contour_center;

    for (auto& hull : hulls) {
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
