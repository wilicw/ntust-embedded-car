#include "raw_data_processing.h"
cv::Mat& raw_data_processing::imgSharpen(const cv::Mat& img, char* arith)
{
	int rows = img.rows;        
	int cols = img.cols * img.channels();   
	int offsetx = img.channels();      

	static cv::Mat dst = cv::Mat::ones(img.rows - 2, img.cols - 2, img.type());

	for (int i = 1; i < rows - 1; i++)
	{

		const uchar* previous = img.ptr<uchar>(i - 1);
		const uchar* current = img.ptr<uchar>(i);
		const uchar* next = img.ptr<uchar>(i + 1);
		uchar* output = dst.ptr<uchar>(i - 1);
		for (int j = offsetx; j < cols - offsetx; j++)
		{
			output[j - offsetx] =
				cv::saturate_cast<uchar>(previous[j - offsetx] * arith[0] + previous[j] * arith[1] + previous[j + offsetx] * arith[2] +
					current[j - offsetx] * arith[3] + current[j] * arith[4] + current[j + offsetx] * arith[5] +
					next[j - offsetx] * arith[6] + next[j] * arith[7] + next[j - offsetx] * arith[8]);
		}
	}
	return dst;
}
cv::Mat raw_data_processing::padding(const cv::Mat& img) {
	const int H = img.rows, W = img.cols;
	cv::Mat expanded_image(H + 20, W + 20, CV_8UC3, cv::Scalar(0, 0, 0));
	for (int i = 0; i < H; ++i) {
		for (int j = 0; j < W; ++j) {
			expanded_image.at<cv::Vec3b>(i + 10, j + 10) = img.at<cv::Vec3b>(i, j);
		}
	}
	return expanded_image;
}
void raw_data_processing::contrast_increasing(cv::Mat& img, float a, float b) {
	//const int H = img.rows, W = img.cols;
	//for (int i = 0; i < H; ++i) {
	//	for (int j = 0; j < W; ++j) {
	//		for (int k = 0; k < 3; ++k) {
	//			int transformed = float(img.at<cv::Vec3b>(i, j)[k]) * (a)+b;
	//			if (transformed > 255) { transformed = 255; }
	//			else if (transformed < 0) { transformed = 0; }
	//			img.at<cv::Vec3b>(i, j)[k] = transformed;
	//		}
	//	}
	//}
	img = img * a + b;
}
cv::Mat raw_data_processing::emphasize_first(const cv::Mat& first,const  cv::Mat& second,const  cv::Mat& third) {
	int mn = 255, mx = 0;
	const int H = first.rows, W = first.cols;
	cv::Mat ret = first.clone();
	for (int i = 0; i < H; ++i) {
		for (int j = 0; j < W; ++j) {
			float weighted = ((int)second.at<uchar>(i, j) + (int)(third.at<uchar>(i, j)) / 2) * 1.45;
			if (int(first.at<uchar>(i, j)) > weighted) {
				ret.at<uchar>(i, j) -= weighted;
			}
			else{
				ret.at<uchar>(i, j) = 0;
			}
			mn = std::min(mn, int(first.at<uchar>(i, j)));
			mx = std::max(mx, int(first.at<uchar>(i, j)));
		}
	}
	cv::normalize(ret,ret, 255, 0, cv::NORM_INF);
	return ret;
}
using Contours = std::vector<std::vector< cv::Point>>;

std::vector<raw_data_processing::rect_info> 
	raw_data_processing::draw_contours_and_rectangle(const std::vector<cv::Mat>& imgs) {
	std::vector<raw_data_processing::rect_info> ret;
	for (const auto& img : imgs) {
		threshold(img, img, 100, 255, 0);//input, output, val, max, 0->binary, 1-> inverted binary
		Contours contours, app_contours;
		std::vector<cv::Vec4i> hierarchy;

		cv::findContours(img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
		for (int i = 0; i < contours.size(); ++i) {
			if (contours[i].size() > 400 and contours[i].size() < 1800) {
				approxPolyDP(cv::Mat(contours[i]), contours[i], 3, true);

				raw_data_processing::rect_info inf;
				//inf.con = contours[i];
				double area = cv::contourArea(contours[i], false);
				inf.contour_area = int(area);
				inf.rect = cv::boundingRect(contours[i]);
				cv::Moments M = cv::moments(contours[i]);
				cv::Point center(M.m10 / M.m00, M.m01 / M.m00);
				inf.contour_center = center;
				
				ret.push_back(inf);
			}
		}
	}
	return ret;
}

cv::Mat raw_data_processing::white_filter(const cv::Mat& img) {
	cv::Mat ret = img.clone();
	const int H = ret.rows, W = ret.cols;
	for (int i = 0; i < H; ++i) {
		for (int j = 0; j < W; ++j) {
			bool good = 1;
			for (int k = 0; k < 3; ++k) {
				if (img.at<cv::Vec3b>(i, j)[k] > 200) { continue; }
				else { good = 0; break; }
			}
			if (good) {
				ret.at<cv::Vec3b>(i, j) = { 255, 255, 255 };
			}
			else {
				ret.at<cv::Vec3b>(i, j) = { 0, 0,0 };
			}
		}
	}
	cv::cvtColor(ret, ret, cv::COLOR_BGR2GRAY);
	int morph_size1 = 8, morph_size2 = 4;
	cv::Mat element1 = cv::getStructuringElement(
		cv::MORPH_RECT, cv::Size(2 * morph_size1 + 1,
			2 * morph_size1 + 1),
		cv::Point(morph_size1, morph_size1));
	cv::Mat element2 = cv::getStructuringElement(
		cv::MORPH_RECT, cv::Size(2 * morph_size2 + 1,
			2 * morph_size2 + 1),
		cv::Point(morph_size2, morph_size2));
	// For Dilation
	cv::dilate(ret,ret, element1,
		cv::Point(-1, -1), 1);
	// For Erosion
	cv::erode(ret, ret, element2,
		cv::Point(-1, -1), 1);
	return ret;
}