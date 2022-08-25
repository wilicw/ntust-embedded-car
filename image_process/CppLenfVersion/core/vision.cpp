#include "vision.h"

//#define DEBUG
//#define laplacian_check_debug


const unordered_map<char, Vision::PARM_t> Vision::PARM =
    unordered_map<char, Vision::PARM_t>({
        {'R', {10, 150, 0.3, 0.26, 1, 0, 0}},
        {'B', {95, 125, 0.2, 0.18, 1, 0, 0}},
        {'W', {0, 0, 0.18, 0.75, 14, 0.465, 0.389}}});

#ifdef DEBUG
#include "pictureIO.h"
#define __coutline cout<<"LINE:"<<__LINE__<<"; "
#endif


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

    cv::copyMakeBorder(image, image, 10, 10, 10, 10, cv::BORDER_CONSTANT, cv::Scalar(0,255,0));

    vector<cv::Mat> HSV_channels = this->cvt_HSV(image); // [0]:H, [1]:S, [2]:V

///--------------RED channel--------------

    pair<cv::Rect, cv::Point> red_pair = this->RED_CH(image, HSV_channels);

///--------------Blue channel--------------

    pair<cv::Rect, cv::Point> blue_pair = this->BLUE_CH(image, HSV_channels);

///--------------White channel--------------

    pair<cv::Rect, cv::Point> white_pair = this->WHITE_CH(image, HSV_channels);

///--------------Judge R B W three channel's size--------------
    if (red_pair.first.area() == 0 && blue_pair.first.area() == 0) {
        return empty_result;
    }

    vector<pair<cv::Rect, cv::Point>> pairs({red_pair, blue_pair, white_pair});

    int pair_index = this->find_max_rect(pairs);
    if (pair_index == -1) return empty_result;

    cv::Mat cropped = image(pairs[pair_index].first);
    cv::resize(cropped, cropped, cv::Size(50, 50));

    if(pair_index != 2){
        if (!this->laplacian_check(cropped, 500)) return empty_result;
    }

#ifdef DEBUG
    cv::waitKey(0);
    cv::destroyAllWindows();
#endif

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

#ifdef laplacian_check_debug
    cv::imshow("laplacian", laplacian);
    cv::waitKey(0);
    cv::destroyAllWindows();
#endif

    cv::Scalar mean, stddev;
    cv::meanStdDev(laplacian, mean, stddev, cv::Mat());

#ifdef laplacian_check_debug
    cv::Mat element2 = cv::getStructuringElement(
            cv::MORPH_RECT, cv::Size(1 +1, 1 + 1),
            cv::Point(0, 0));
    cv::erode(laplacian, laplacian, element2, cv::Point(0, 0), 1);
    cv::dilate(laplacian, laplacian, element2, cv::Point(0, 0), 1);
//    laplacian = this->dilation(laplacian, 1);
    cv::imshow("lap dil ero", laplacian);
    cv::waitKey(0);
    cv::destroyAllWindows();
#endif

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

#ifdef DEBUG
//    pictureIO::printMat(channels);
#endif
//    channels[0].convertTo(channels[0], CV_32F);
    channels[1].convertTo(channels[1], CV_32F);
    channels[2].convertTo(channels[2], CV_32F);

    cv::normalize(channels[1], channels[1], 0, 1, cv::NORM_MINMAX);
    cv::normalize(channels[2], channels[2], 0, 1, cv::NORM_MINMAX);
#ifdef DEBUG
//    pictureIO::printMat(channels);
#endif

    return channels;
}



pair<cv::Rect, cv::Point> Vision::RED_CH(cv::Mat image, vector<cv::Mat> HSV_channels) {
#ifdef DEBUG
    cv::Mat show_img;
#endif

    cv::Mat H_channels = this->H_filter(HSV_channels[0].clone(), Vision::PARM.at('R').H_filter_MIN, Vision::PARM.at('R').H_filter_MAX);
    H_channels = 255 - H_channels;

#ifdef DEBUG
    cv::imshow("RED_H", H_channels.clone());
#endif

#ifdef DEBUG
    show_img = HSV_channels[1].clone() * 255;
    show_img.convertTo(show_img, CV_8UC1);
    cv::imshow("before S thres", show_img);
#endif

    cv::Mat Sthres;

    cv::threshold(HSV_channels[1], Sthres, Vision::PARM.at('R').S_threshold, 255, cv::THRESH_TOZERO);

#ifdef DEBUG
    show_img = Sthres.clone() * 255;
    show_img.convertTo(show_img, CV_8UC1);
    cv::imshow("after S thres", show_img);
#endif

    cv::Mat SxV = Sthres.mul(HSV_channels[2]);

#ifdef DEBUG
    show_img = SxV.clone() * 255;
    show_img.convertTo(show_img, CV_8UC1);
    cv::imshow("SxV", show_img);
#endif
    cv::Mat SxVthres;
    cv::threshold(SxV, SxVthres, Vision::PARM.at('R').SxV_threshold, 255, cv::THRESH_BINARY);
    SxVthres = this->dilation(SxVthres, Vision::PARM.at('R').SxV_dilation_scale);
    SxVthres = this->erosion(SxVthres, Vision::PARM.at('R').SxV_dilation_scale);

#ifdef DEBUG
    cv::imshow("SxVthres", SxVthres.clone());
#endif
    H_channels.convertTo(H_channels, CV_32F);

    cv::Mat HxSxV = SxVthres.mul(H_channels);

    HxSxV.convertTo(HxSxV, CV_8UC1);

#ifdef DEBUG
    cv::imshow("HxSxV", HxSxV.clone());
#endif

    vector<vector<cv::Point>> contours = this->find_contours(HxSxV);
    vector<vector<cv::Point>> hulls = this->contours_to_hulls(contours);
    pair<cv::Rect, cv::Point> ret = this->find_rectangle(hulls);

#ifdef DEBUG
    show_img = image.clone();
    cv::drawContours(show_img, hulls, -1, cv::Scalar(0,255,255), 3);
    cv::rectangle(show_img, ret.first, cv::Scalar(0,255,0), 3);
    cv::imshow("rect&hull", show_img);
    cv::waitKey(0);
    cv::destroyAllWindows();
#endif

#ifdef DEBUG
    cv::waitKey(0);
    cv::destroyAllWindows();
#endif
    return ret;
}

pair<cv::Rect, cv::Point> Vision::BLUE_CH(cv::Mat image, vector<cv::Mat> HSV_channels) {
#ifdef DEBUG
    cv::Mat show_img;
#endif
    cv::Mat H_channel = this->H_filter(HSV_channels[0].clone(), Vision::PARM.at('B').H_filter_MIN, Vision::PARM.at('B').H_filter_MAX);

#ifdef DEBUG
    cv::imshow("BLUE_H", H_channel.clone());
#endif

#ifdef DEBUG
    show_img = HSV_channels[1].clone() * 255;
    show_img.convertTo(show_img, CV_8UC1);
    cv::imshow("beforeS", show_img);
#endif

    cv::Mat Sthres;

    cv::threshold(HSV_channels[1], Sthres, Vision::PARM.at('B').S_threshold, 255, cv::THRESH_TOZERO);

#ifdef DEBUG
    show_img = Sthres.clone() * 255;
    show_img.convertTo(show_img, CV_8UC1);
    cv::imshow("afterS", show_img);
#endif

    cv::Mat SxV = Sthres.mul(HSV_channels[2]);

#ifdef DEBUG
    show_img = SxV.clone() * 255;
    show_img.convertTo(show_img, CV_8UC1);
    cv::imshow("SxV", show_img);
#endif
    cv::Mat SxVthres;
    cv::threshold(SxV, SxVthres, Vision::PARM.at('B').SxV_threshold, 255, cv::THRESH_BINARY);
    SxVthres = this->dilation(SxVthres, Vision::PARM.at('B').SxV_dilation_scale);
    SxVthres = this->erosion(SxVthres, Vision::PARM.at('B').SxV_dilation_scale);

#ifdef DEBUG
    cv::imshow("SxVthres", SxVthres.clone());
#endif
    H_channel.convertTo(H_channel, CV_32F);

    cv::Mat HxSxV = SxVthres.mul(H_channel);

    HxSxV.convertTo(HxSxV, CV_8UC1);

#ifdef DEBUG
    cv::imshow("HxSxV", HxSxV.clone());
#endif

    vector<vector<cv::Point>> contours = this->find_contours(HxSxV);
    vector<vector<cv::Point>> hulls = this->contours_to_hulls(contours);
    pair<cv::Rect, cv::Point> ret = this->find_rectangle(hulls);

#ifdef DEBUG
    show_img = image.clone();
    cv::drawContours(show_img, hulls, -1, cv::Scalar(0,255,255), 3);
    cv::rectangle(show_img, ret.first, cv::Scalar(0,255,0), 3);
    cv::imshow("rect&hull", show_img);
    cv::waitKey(0);
    cv::destroyAllWindows();
#endif

#ifdef DEBUG
    cv::waitKey(0);
    cv::destroyAllWindows();
#endif
    return ret;
}


pair<cv::Rect, cv::Point> Vision::WHITE_CH(cv::Mat image, vector<cv::Mat> HSV_channels) {
#ifdef DEBUG
    cv::Mat show_img;
#endif

    cv::Mat img_S_channel = HSV_channels[1].clone();
    cv::Mat img_V_channel = HSV_channels[2].clone();

    cv::Mat WHITE_Sthres, WHITE_Vthres, BLACK_Vthres, BLACK_SxV, BLACK_SxVthres, S_Bar;

    // take white S channel
    cv::threshold(img_S_channel, WHITE_Sthres, Vision::PARM.at('W').S_threshold, 255, cv::THRESH_BINARY);
    WHITE_Sthres = 255 - WHITE_Sthres;
    WHITE_Sthres.convertTo(WHITE_Sthres, CV_8UC1);

    // take white V channel
    cv::threshold(img_V_channel, WHITE_Vthres, Vision::PARM.at('W').white_V_threshold, 255, cv::THRESH_BINARY);
    WHITE_Vthres.convertTo(WHITE_Vthres, CV_8UC1);

    // take black V channel
    cv::threshold(img_V_channel, BLACK_Vthres, Vision::PARM.at('W').black_V_threshold, 255, cv::THRESH_TOZERO);
    BLACK_Vthres = 1 - BLACK_Vthres;

    S_Bar = 1 - img_S_channel;

    BLACK_Vthres.convertTo(BLACK_Vthres, CV_32F);
    BLACK_SxV = S_Bar.mul(BLACK_Vthres);

    cv::threshold(BLACK_SxV, BLACK_SxVthres, Vision::PARM.at('W').SxV_threshold, 255, cv::THRESH_BINARY);
//    BLACK_SxVthres = 255 - BLACK_SxVthres;
    BLACK_SxVthres.convertTo(BLACK_SxVthres, CV_8UC1);



    BLACK_SxVthres = this->dilation(BLACK_SxVthres, Vision::PARM.at('W').SxV_dilation_scale);
//    BLACK_SxVthres = this->erosion(BLACK_SxVthres, Vision::PARM.at('W').SxV_dilation_scale /5);













#ifdef DEBUG
    show_img = img_S_channel.clone() * 255;
    show_img.convertTo(show_img, CV_8UC1);
    cv::imshow("only_S", show_img);
//
    show_img = img_V_channel.clone() * 255;
    show_img.convertTo(show_img, CV_8UC1);
    cv::imshow("only_V", show_img);

    show_img = S_Bar.clone() * 255;
    show_img.convertTo(show_img, CV_8UC1);
    cv::imshow("S_Bar", show_img);

    show_img = WHITE_Sthres.clone();
    cv::imshow("WHITE_Sthres", show_img);

    show_img = WHITE_Vthres.clone();
    cv::imshow("WHITE_Vthres", show_img);

    show_img = BLACK_Vthres.clone() * 255;
    show_img.convertTo(show_img, CV_8UC1);
    cv::imshow("BLACK_Vthres", show_img);

    show_img = BLACK_SxV.clone();
    cv::imshow("BLACK_SxV", show_img);

    show_img = BLACK_SxVthres.clone();
    cv::imshow("BLACK_SxVthres", show_img);
#endif

    cv::Mat BxW = BLACK_SxVthres.mul(WHITE_Vthres).mul(WHITE_Sthres);
#ifdef DEBUG

    show_img = BxW.clone();
//    show_img.convertTo(show_img, CV_8UC1);
    cv::imshow("BxW_thres", show_img);
#endif

    vector<vector<cv::Point>> contours = this->find_contours(BxW);
    vector<vector<cv::Point>> hulls = this->contours_to_hulls(contours);
    pair<cv::Rect, cv::Point> ret = this->find_rectangle(hulls);

#ifdef DEBUG
    show_img = image.clone();
    cv::drawContours(show_img, contours, -1, cv::Scalar(255,255,0), 3);
    cv::drawContours(show_img, hulls, -1, cv::Scalar(0,255,255), 2);
    cv::rectangle(show_img, ret.first, cv::Scalar(0,255,0), 1);
    cv::imshow("rect&hull", show_img);
    cv::waitKey(0);
    cv::destroyAllWindows();
#endif


#ifdef DEBUG
    cv::waitKey(0);
    cv::destroyAllWindows();
#endif
    return ret;
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
    //cv::threshold(image, copy, 100, 255, 0);
    //input, output, val, max, 0->binary, 1-> inverted binary
    vector<vector<cv::Point>> contours;
    vector<cv::Vec4i> hierarchy;
    cv::findContours(copy, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    return contours;
}

vector<vector<cv::Point>> Vision::contours_to_hulls(vector<vector<cv::Point>> contours){
    vector<vector<cv::Point>> hulls;
    for(auto& contour:contours){
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

    for (auto &hull: hulls) {

        const double length = cv::arcLength(hull, true);
#ifdef DEBUG
//        __coutline << hull.size() << endl;
#endif
        //judge if area is too big or small
//        double area = cv::contourArea(hull, false);
//        double len = length;
//        double min_area = len * len * 9.0 / 400.0, max_area = len * len / 16.0;
//        if (area < min_area || max_area < area) {
//            continue;
//        }

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


