#include "vision.h"
#include "raw_data_processing.h"
#define blurThreshold 15.0

sign_info_t Vision::processing(cv::Mat picture) {
    sign_info_t ret_sign_info;
	
	//read the image and convert it to grayscale:
	cv::Mat gray;
	cv::cvtColor(picture, gray, cv::COLOR_RGB2GRAY);

	//Cool, let's compute the laplacian of the gray image:
	cv::Laplacian(gray, gray, CV_64F);

	//Prepare to compute the mean and standard deviation of the laplacian:
	cv::Scalar mean, stddev;
	cv::meanStdDev(laplacianImage, mean, stddev, cv::Mat());

	//Let’s compute the variance:
	double variance = stddev.val[0] * stddev.val[0];

	if (variance <= blurThreshold) {
		return ret_sign_info;
	}
	
    vector<cv::Mat> channels;
    vector<cv::Mat> cropped_images;
    cv::copyMakeBorder(picture, picture, 10, 10, 10, 10, cv::BORDER_CONSTANT,
                       0);
    cv::Mat white_specialized = raw_data_processing::white_filter(picture);
    //picture = picture * 3 + (-200.0);  // contrast_increasing
    split(picture, channels);          //[0]->b, [1]->g [2]->r

    cv::Mat R_single_channel = raw_data_processing::emphasize_first(
        channels[2], channels[0], channels[1]);
    cv::Mat B_single_channel = raw_data_processing::emphasize_first(
        channels[0], channels[2], channels[1]);
    vector<raw_data_processing::rect_info> searched_rectangles =
        raw_data_processing::draw_contours_and_rectangle(
            {B_single_channel, R_single_channel, white_specialized});
    int mxI = -1, mx_area = -1e9;
    for (int j = 0; j < int(searched_rectangles.size()); ++j) {
        int area = searched_rectangles[j].rect.width *
                   searched_rectangles[j].rect.height;
        if (area > mx_area) {
            mx_area = area;
            mxI = j;
        }
    }
    if (mxI != -1) {
        cv::Rect rect = searched_rectangles[mxI].rect;
        cv::Mat cropped = picture(rect);
        resize(cropped, cropped, cv::Size(50, 50));
        ret_sign_info.cropped = cropped;
        ret_sign_info.center_position = searched_rectangles[mxI].contour_center;
        ret_sign_info.area = searched_rectangles[mxI].contour_area;
    }
	return ret_sign_info;
}
