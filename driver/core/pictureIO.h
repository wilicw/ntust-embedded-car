#pragma once
#include<opencv2/opencv.hpp>
#include<string>
#include<vector>
using cv::Mat;
using std::string;
using std::vector;
using std::to_string;
namespace pictureIO {
	vector<Mat> load_pictures(const string&, const string&, const int&, const int&);
	void write_pictures(const string&, const vector<Mat>& images);
	void printMat(const vector<Mat>&);
};
