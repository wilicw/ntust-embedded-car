#pragma once
#include<opencv2/opencv.hpp>
#include<string>
#include<vector>
using cv::Mat;
using std::string;
using std::vector;
using std::to_string;
namespace pictureIO {
	class pic_info {
	public:
		pic_info(int ind, cv::Mat img) : index(ind), img(img) {};
		int index;
		cv::Mat img;
	};
	vector<pic_info> load_pictures(const string&, const string&, const int&, const int&);
	void write_pictures(const string&, const vector<pictureIO::pic_info>& images);
	void printMat(const vector<Mat>&);
};
