#include "vision.h"
//#define MANUAL_SVM
//#define DEBUG
//#define laplacian_check_debug
//#define TIME_DEBUG
int Vision::index = 0;

const unordered_map<char, Vision::PARM_t> Vision::PARM =
        unordered_map<char, Vision::PARM_t>({
                                                    {'R', {10, 150, 76, 51, 3}},
                                                    {'B', {105, 130, 51, 17, 3}}});

#define __coutline cout<<"LINE"<<__LINE__<<"; "

#ifdef DEBUG
#define DEBUG_RED
#define DEBUG_BLUE
#define DEBUG_WHITE
#define DEBUG_BLACK
#define DEBUG_BW_MIX
#include "pictureIO.h"
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
//    std::clock_t clk = std::clock();
    const static sign_item_t empty_result = {nullptr, nullptr};

    cv::copyMakeBorder(image, image, 2, 2, 2, 2, cv::BORDER_CONSTANT, cv::Scalar(0,40,0));

    vector<cv::Mat> HSV_channels = this->cvt_HSV(image); // [0]:H, [1]:S, [2]:V
//    __coutline << "  TO HSV : " << std::clock() - clk << endl;
//    clk = std::clock();

    cv::Mat out_RED_hsv;
///--------------RED channel--------------

    pair<cv::Rect, cv::Point> red_pair = this->RED_CH(image, HSV_channels, out_RED_hsv);
//    __coutline << "  RED CH : " << std::clock() - clk << endl;
//    clk = std::clock();
///--------------Blue channel--------------

    pair<cv::Rect, cv::Point> blue_pair = this->BLUE_CH(image, HSV_channels);
//    __coutline << " BLUE CH : " << std::clock() - clk << endl;
//    clk = std::clock();
///--------------White channel--------------

    pair<cv::Rect, cv::Point> white_pair = this->WHITE_CH(image, HSV_channels, out_RED_hsv);
//    __coutline << "WHITE CH : " << std::clock() - clk << endl;
#ifdef DEBUG
    cv::waitKey(0);
    cv::destroyAllWindows();
#endif

///--------------Judge R B W three channel's size--------------
    if (red_pair.first.area() == 0 && blue_pair.first.area() == 0 && white_pair.first.area() == 0) {
//        cout << Vision::index << " " << "rect nothing found" << endl;
        return empty_result;
    }

    vector<pair<cv::Rect, cv::Point>> pairs({red_pair, blue_pair, white_pair});

    int pair_index = this->find_max_rect(pairs);
    if (pair_index == -1){
//        cout << Vision::index << " " << "rect nothing found" << endl;
        return empty_result;
    }

    cv::Mat cropped = image(pairs[pair_index].first);
    cv::resize(cropped, cropped, cv::Size(50, 50));

    if(pair_index != 2){
        double lap = this->laplacian(cropped);
        if (lap < 400){
#ifdef MANUAL_SVM
            __coutline << Vision::index << " lap too small:" << lap << endl;
#endif
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

    return stddev.val[0] * stddev.val[0];
}

vector<cv::Mat> Vision::cvt_HSV(cv::Mat image) {
    cv::Mat img_hsv;
    cv::cvtColor(image, img_hsv, cv::COLOR_BGR2HSV);

    vector<cv::Mat> channels;
    split(img_hsv, channels); // H S V

#ifdef DEBUG
//    pictureIO::printMat(channels);
#endif

    return channels;
}



pair<cv::Rect, cv::Point> Vision::RED_CH(cv::Mat image, vector<cv::Mat> HSV_channels, cv::Mat& out_hsv) {
#ifdef DEBUG
    cv::Mat show_img;
#endif

    cv::Mat img_H_channel = HSV_channels[0];
    cv::Mat img_S_channel = HSV_channels[1];
    cv::Mat img_V_channel = HSV_channels[2];

    cv::Mat H_channel = this->H_filter(img_H_channel, Vision::PARM.at('R').H_filter_MIN, Vision::PARM.at('R').H_filter_MAX);
    cv::bitwise_not(H_channel, H_channel);

#if defined(DEBUG) and defined(DEBUG_RED)
    show_img = H_channel.clone();
    cv::imshow("RED_H", show_img);
#endif

#if defined(DEBUG) and defined(DEBUG_RED)
    show_img = img_S_channel.clone();
    cv::imshow("before S thres", show_img);
#endif

    cv::Mat Sthres;

    cv::threshold(img_S_channel, Sthres, Vision::PARM.at('R').S_threshold, 255, cv::THRESH_TOZERO);

#if defined(DEBUG) and defined(DEBUG_RED)
    show_img = Sthres.clone();
    cv::imshow("after S thres", show_img);
#endif

    cv::Mat SxV;
    cv::bitwise_and(Sthres, img_V_channel, SxV);

#if defined(DEBUG) and defined(DEBUG_RED)
    show_img = SxV.clone();
    cv::imshow("SxV", show_img);
#endif

    cv::Mat SxVthres;
    cv::threshold(SxV, SxVthres, Vision::PARM.at('R').SxV_threshold, 255, cv::THRESH_BINARY);
    SxVthres = this->dilation(SxVthres, Vision::PARM.at('R').SxV_dilation_scale);
    SxVthres = this->erosion(SxVthres, Vision::PARM.at('R').SxV_dilation_scale);

#if defined(DEBUG) and defined(DEBUG_RED)
    show_img = SxVthres.clone();
    cv::imshow("SxVthres", show_img);
#endif


    cv::Mat HxSxV;
    cv::bitwise_and(SxVthres, H_channel, HxSxV);

    out_hsv = HxSxV;

#if defined(DEBUG) and defined(DEBUG_RED)
    show_img = HxSxV.clone();
    cv::imshow("HxSxV", show_img);
#endif

    vector<vector<cv::Point>> contours = this->find_contours(HxSxV);
    vector<vector<cv::Point>> hulls = this->contours_to_hulls(contours);
    pair<cv::Rect, cv::Point> ret = this->find_rectangle(hulls);

#if defined(DEBUG) and defined(DEBUG_RED)
    show_img = image.clone();
    cv::drawContours(show_img, hulls, -1, cv::Scalar(0,255,255), 2);
    cv::rectangle(show_img, ret.first, cv::Scalar(0,255,0), 1);
    cv::imshow("RED rect&hull", show_img);
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

    cv::Mat img_H_channel = HSV_channels[0];
    cv::Mat img_S_channel = HSV_channels[1];
    cv::Mat img_V_channel = HSV_channels[2];

    cv::Mat H_channel = this->H_filter(img_H_channel, Vision::PARM.at('B').H_filter_MIN, Vision::PARM.at('B').H_filter_MAX);

#if defined(DEBUG) and defined(DEBUG_BLUE)
    show_img = H_channel.clone();
    cv::imshow("BLUE_H", show_img);
#endif

#if defined(DEBUG) and defined(DEBUG_BLUE)
    show_img = img_S_channel;
    cv::imshow("beforeS", show_img);
#endif

    cv::Mat Sthres;
    cv::threshold(img_S_channel, Sthres, Vision::PARM.at('B').S_threshold, 255, cv::THRESH_TOZERO);

#if defined(DEBUG) and defined(DEBUG_BLUE)
    show_img = Sthres.clone();
    cv::imshow("afterS", show_img);
#endif

    cv::Mat SxV;
    cv::bitwise_and(Sthres, img_V_channel, SxV);

#if defined(DEBUG) and defined(DEBUG_BLUE)
    show_img = SxV.clone();
    cv::imshow("SxV", show_img);
#endif

    cv::Mat SxVthres;
    cv::threshold(SxV, SxVthres, Vision::PARM.at('B').SxV_threshold, 255, cv::THRESH_BINARY);
    SxVthres = this->dilation(SxVthres, Vision::PARM.at('B').SxV_dilation_scale);
    SxVthres = this->erosion(SxVthres, Vision::PARM.at('B').SxV_dilation_scale);

#if defined(DEBUG) and defined(DEBUG_BLUE)
    show_img = SxVthres.clone();
    cv::imshow("SxVthres", show_img);
#endif

    cv::Mat HxSxV;
    cv::bitwise_and(SxVthres, H_channel, HxSxV);

#if defined(DEBUG) and defined(DEBUG_BLUE)
    show_img = HxSxV.clone();
    cv::imshow("HxSxV", show_img);
#endif

    vector<vector<cv::Point>> contours = this->find_contours(HxSxV);
    vector<vector<cv::Point>> hulls = this->contours_to_hulls(contours);
    pair<cv::Rect, cv::Point> ret = this->find_rectangle(hulls);

#if defined(DEBUG) and defined(DEBUG_BLUE)
    show_img = image.clone();
    cv::drawContours(show_img, hulls, -1, cv::Scalar(0,255,255), 2);
    cv::rectangle(show_img, ret.first, cv::Scalar(0,255,0), 1);
    cv::imshow("BLUE rect&hull", show_img);
    cv::waitKey(0);
    cv::destroyAllWindows();
#endif

#ifdef DEBUG
    cv::waitKey(0);
    cv::destroyAllWindows();
#endif
    return ret;
}


pair<cv::Rect, cv::Point> Vision::WHITE_CH(cv::Mat image, vector<cv::Mat> HSV_channels, cv::Mat RED_hsv) {
#ifdef DEBUG
    cv::Mat show_img;
#endif

    cv::Mat img_H_channel = HSV_channels[0];
    cv::Mat img_S_channel = HSV_channels[1];
    cv::Mat img_V_channel = HSV_channels[2];

//------BLUE MASK------
#ifdef TIME_DEBUG
    std::clock_t clk = std::clock();
#endif
    cv::Mat Blue_MASK_V, Blue_MASK_H, Blue_MASK;
    Blue_MASK_H = this->H_filter(img_H_channel, 90,105);
    cv::threshold(img_V_channel, Blue_MASK_V, 160, 255, cv::THRESH_BINARY);
    cv::bitwise_and(Blue_MASK_H, Blue_MASK_V, Blue_MASK);

#ifdef TIME_DEBUG
    cout << "W: blue mask:" << std::clock() - clk << endl;
    clk = std::clock();
#endif

#if defined(DEBUG) and defined(DEBUG_BW_MIX)
    show_img = img_S_channel.clone();
    cv::imshow("only_S", show_img);

    show_img = img_V_channel.clone();
    cv::imshow("only_V", show_img);
#endif

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
    cv::bitwise_or(WHITE_SxV,Blue_MASK, WHITE_SxV_blue);

#ifdef TIME_DEBUG
//    cout << "W: white SV:" << std::clock() - clk << endl;
//    clk = std::clock();
#endif

    // dilation SV
    cv::Mat WHITE_SxV_ero = this->erosion(WHITE_SxV_blue, 5);
    cv::Mat WHITE_SxV_orgin = this->dilation(WHITE_SxV_ero, 5);
    cv::Mat WHITE_SxV_dil = this->dilation(WHITE_SxV_orgin, 10);

    cv::Mat WHITE_SxV_diff;
    cv::subtract(WHITE_SxV_dil,  WHITE_SxV_orgin, WHITE_SxV_diff);
    cv::Mat WHITE_SxV_diff_dil = this->dilation(WHITE_SxV_diff, 7);

#ifdef TIME_DEBUG
    cout << "W: white SV whole:" << std::clock() - clk << endl;
    clk = std::clock();
#endif

#if defined(DEBUG) and defined(DEBUG_WHITE)

    show_img = Blue_MASK.clone();
    cv::imshow("Blue_MASK", show_img);

    show_img = WHITE_Sthres.clone();
    cv::imshow("WHITE_Sthres", show_img);

    show_img = WHITE_Vthres.clone();
    cv::imshow("WHITE_Vthres", show_img);

    show_img = WHITE_SxV.clone();
    cv::imshow("WHITE_SxV", show_img);

    show_img = WHITE_SxV_blue.clone();
    cv::imshow("WHITE_SxV_blue", show_img);

    show_img = WHITE_SxV_dil.clone();
    cv::imshow("WHITE_SxV_dil", show_img);

    show_img = WHITE_SxV_diff_dil.clone();
    cv::imshow("WHITE_SxV_diff", show_img);
#endif

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
    cv::bitwise_or(Black_Vthres,BLACK_SxVthres,BLACK_SxVthres);

    //dilation SV
    BLACK_SxVthres = this->dilation(BLACK_SxVthres, 5);

#ifdef TIME_DEBUG
    cout << "W: black SV whole:" << std::clock() - clk << endl;
    clk = std::clock();
#endif

#if defined(DEBUG) and defined(DEBUG_BLACK)
    BLACK_016S_V.convertTo(show_img, CV_8UC1);
    cv::imshow("BLACK_0.16S+V", show_img);

    show_img = BLACK_SxVthres.clone();
    cv::imshow("BLACK_SxVthres", show_img);
#endif

//------sub red and black------

    //dilation red
    RED_hsv = this->dilation(RED_hsv, 12);
    cv::Mat R_B_diff;
    cv::subtract(BLACK_SxVthres, RED_hsv, R_B_diff);

#ifdef TIME_DEBUG
    cout << "W: sub red and black:" << std::clock() - clk << endl;
    clk = std::clock();
#endif

#if defined(DEBUG) and defined(DEBUG_BLACK)
    show_img = R_B_diff.clone();
    cv::imshow("R_B_diff",show_img);
#endif


//------Black * White------
    cv::Mat BxW;
    cv::bitwise_and(R_B_diff, WHITE_SxV_diff_dil, BxW);

#if defined(DEBUG) and defined(DEBUG_BW_MIX)
    show_img = BxW.clone();
    cv::imshow("BxW", show_img);
#endif

#ifdef TIME_DEBUG
    clk = std::clock();
#endif


//------judge sum val------
    int whitesum = cv::sum(BxW)[0];

    if(whitesum < 60000){
#ifdef MANUAL_SVM
        if(whitesum > 100) {
            __coutline << Vision::index << " " << "WHITE sum " << whitesum << endl;
        }
#endif
        return pair<cv::Rect, cv::Point>();
    }

#ifdef TIME_DEBUG
    clk = std::clock();
#endif


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

#if defined(DEBUG) and defined(DEBUG_BW_MIX)
    show_img = filled_img.clone();
    cv::imshow("WHITE_SxV_orgin_filled", show_img);

    show_img = image.clone();
    cv::drawContours(show_img, black_countour, -1, cv::Scalar(0,255,0), 1);
    cv::imshow("contoured img", show_img);
#endif

    if(black_countour.size() != 1){
#ifdef MANUAL_SVM
        __coutline << Vision::index << "black_countour size too much : " << black_countour.size() << endl;
#endif
        return pair<cv::Rect, cv::Point>();
    }

#ifdef TIME_DEBUG
    cout << "W: white is in black:" << std::clock() - clk << endl;
    clk = std::clock();
#endif

//------fill white image------
    // cv::Point white_point;
    cv::minMaxLoc(BxW, NULL, NULL, NULL, &white_point);

    cv::Mat White_filled_img = WHITE_SxV_diff_dil;
    cv::floodFill(White_filled_img, white_point, 127);

    cv::inRange(White_filled_img, 120, 130, White_filled_img);

#if defined(DEBUG) and defined(DEBUG_BW_MIX)
    show_img = filled_img.clone();
    cv::imshow("WHITE_SxV_diff_dil_filled_img", show_img);
#endif
#ifdef TIME_DEBUG
    cout << "W: floodfill:" << std::clock() - clk << endl;
    clk = std::clock();
#endif

//------BxW to find contours------
    cv::Mat BandW;
    cv::bitwise_and(White_filled_img, Black_filled_img, BandW);

    //------find contours and rectangle------
    vector<vector<cv::Point>> contours = this->find_contours(BandW);
    vector<vector<cv::Point>> hulls = this->contours_to_hulls(contours);
    pair<cv::Rect, cv::Point> ret = this->find_rectangle(hulls);

    //------judge rectangle area
    if(ret.first.area() > 75000){
#ifdef MANUAL_SVM
        __coutline << Vision::index << " " << "WHITE rectangle area " << ret.first.area() << endl;
#endif
        return pair<cv::Rect, cv::Point>();
    }

#if defined(DEBUG) and defined(DEBUG_BW_MIX)
    show_img = image.clone();
    cv::drawContours(show_img, contours, -1, cv::Scalar(255,255,0), 2);
    cv::drawContours(show_img, hulls, -1, cv::Scalar(0,255,255), 2);
    cv::rectangle(show_img, ret.first, cv::Scalar(0,255,0), 1);
    cv::imshow("white rect&hull", show_img);

    cv::waitKey(0);
    cv::destroyAllWindows();
#endif


#ifdef DEBUG
    cv::waitKey(0);
    cv::destroyAllWindows();
#endif
#ifdef TIME_DEBUG
    cout << "W: contours:" << std::clock() - clk << endl;
    clk = std::clock();
#endif
    return ret;
}


cv::Mat Vision::H_filter(cv::Mat H_channel, const int& min, const int& max) {
    cv::Mat ret;
    cv::inRange(H_channel, min, max, ret);
    return ret;
}

cv::Mat Vision::dilation(cv::Mat image, const int &morph_size) {
    cv::Mat kernal = cv::getStructuringElement(
            cv::MORPH_RECT, cv::Size(morph_size, morph_size));
    cv::Mat ret;
    cv::dilate(image, ret, kernal);
    return ret;
}

cv::Mat Vision::erosion(cv::Mat image, const int &morph_size) {
    cv::Mat kernal = cv::getStructuringElement(
            cv::MORPH_RECT, cv::Size(morph_size, morph_size));
    cv::Mat ret;
    cv::erode(image, ret, kernal);
    return ret;
}

vector<vector<cv::Point>> Vision::find_contours(cv::Mat image) {
    cv::Mat copy = image;
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


