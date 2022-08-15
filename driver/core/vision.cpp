#include "vision.h"
#include "raw_data_processing.h"
#include "pictureIO.h"

void sign_finding(cv::Mat& picture, sign_info& ret_sign_info) {
	vector<Mat> channels;
	vector<Mat> cropped_images;
	picture = raw_data_processing::padding(picture);
	char arith[9] = {
		  0, -1, 0, -1, 5, -1, 0, -1, 0 };       //使用拉普拉斯算子
	// picture = raw_data_processing::imgSharpen(picture, arith);
	Mat white_specialized = raw_data_processing::white_filter(picture);
	raw_data_processing::contrast_increasing(picture, 3.0, -200);
	split(picture, channels); //[0]->b, [1]->g [2]->r

	Mat R_single_channel = raw_data_processing::emphasize_first(channels[2], channels[0], channels[1]);
	Mat B_single_channel = raw_data_processing::emphasize_first(channels[0], channels[2], channels[1]);
	vector<raw_data_processing::rect_info> searched_rectangles =
		raw_data_processing::draw_contours_and_rectangle({ B_single_channel, R_single_channel,white_specialized });
	int mxI = -1, mx_area = -1e9;
	for (int j = 0; j<int(searched_rectangles.size()); ++j) {
		int area = searched_rectangles[j].rect.width * searched_rectangles[j].rect.height;
		if (area > mx_area) {
			mx_area = area;
			mxI = j;
		}
	}
	if (mxI != -1) {

		cv::Rect rect = searched_rectangles[mxI].rect;
		cv::Point center_of_rect = (rect.br() + rect.tl()) * 0.5;
		Mat cropped = picture(rect);
		Mat bck = picture.clone();
		//std::cout << bck.rows << std::endl;
		resize(cropped, cropped, cv::Size(50, 50));
		ret_sign_info.cropped = cropped;
		ret_sign_info.center_position = searched_rectangles[mxI].contour_center;
		ret_sign_info.area = searched_rectangles[mxI].contour_area;
		//cv::drawContours(bck, searched_rectangles[mxI].con, int(0), cv::Scalar(0, 255, 0), 2, cv::LINE_8);
		//std::cout << searched_rectangles[mxI].con.size() << std::endl;
		//pictureIO::printMat({ bck });
		//std::cout << "found" << std::endl;
	}
	else {
		throw("nothing captured");
	}
}


