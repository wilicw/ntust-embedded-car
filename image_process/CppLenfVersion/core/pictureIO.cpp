#include "pictureIO.h"


vector<pictureIO::pic_info> pictureIO::load_pictures
	(const string& folder_path, const string& file_name ,const int& size, const int& scale = 1) {
	vector<pictureIO::pic_info> ret;
	for (int i = 0; i<size; i++) {
		try{
			string path = folder_path + file_name + to_string(i* scale) +".jpg";
			cv::Mat img = cv::imread(path)(cv::Rect(5, 5, 630, 470));
			ret.push_back(pictureIO::pic_info(i * scale, img));
		}catch (...){
//            std::cout << "ERROR!!!" << std::endl;
			continue;
		}
		
	}
	return ret;
}

void pictureIO::printMat(const vector<Mat>& images) {
	for (int i = 0; i<int(images.size()); ++i) {
		cv::imshow("test"+to_string(i), images[i]);
	}
}

void pictureIO::write_pictures(const string& path, const vector<pictureIO::pic_info>& images) {
	for (const auto& i : images) {
		cv::imwrite(path + "res" + to_string(i.index) + ".png", i.img);
	}
	std::cout << "end" << std::endl;
}