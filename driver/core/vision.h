#ifndef DRIVER_VISION_H
#define DRIVER_VISION_H


#include <cmath>
#include <opencv2/opencv.hpp>
#include <vector>

#include "communication.h"
#include "unordered_map"

using namespace std;

class Vision {
   private:
    struct PARM_t{
    public:
        int H_filter_MIN;
        int H_filter_MAX;
        double S_threshold;
        double SxV_threshold;
        int SxV_dilation_scale;
    };
    const static unordered_map<char, PARM_t> PARM;
    void contrast_normalization(const cv::Mat);
    inline double laplacian(cv::Mat);
    inline vector<cv::Mat> cvt_HSV(cv::Mat);

    pair<cv::Rect, cv::Point> RED_CH(cv::Mat, vector<cv::Mat>, cv::Mat&);
    pair<cv::Rect, cv::Point> BLUE_CH(cv::Mat, vector<cv::Mat>);
    pair<cv::Rect, cv::Point> WHITE_CH(cv::Mat, vector<cv::Mat>, cv::Mat);

    inline cv::Mat H_filter(cv::Mat, const int&, const int&);
    inline cv::Mat dilation(cv::Mat, const int&);
    inline cv::Mat erosion(cv::Mat, const int&);
    inline vector<vector<cv::Point>> find_contours(cv::Mat);
    inline vector<vector<cv::Point>> contours_to_hulls(vector<vector<cv::Point>>);
    inline pair<cv::Rect, cv::Point> find_rectangle(vector<vector<cv::Point>>);
    inline int find_max_rect(vector<pair<cv::Rect, cv::Point>>);

   public:
    sign_item_t process(cv::Mat);
    float distance(cv::Point);

};

#endif  // DRIVER_VISION_H
