#include "pictureIO.h"

vector<Mat> pictureIO::load_pictures(const string& folder_path,
                                     const string& file_name, const int& size,
                                     const int& scale = 1) {
    vector<Mat> ret;
    for (int i = 0; i < size; i++) {
        try {
            string path =
                folder_path + file_name + to_string(i * scale) + ".jpg";
            ret.push_back(cv::imread(path)(cv::Rect(5, 5, 630, 470)));
        } catch (...) {
            continue;
        }
    }
    return ret;
}
void pictureIO::printMat(const vector<Mat>& images) {
    for (int i = 0; i < int(images.size()); ++i) {
        string idx = to_string(i);
        cv::imshow("test" + idx, images[i]);
    }
    cv::waitKey(0);
    cv::destroyAllWindows();
}
void pictureIO::write_pictures(const string& path, const vector<Mat>& images) {
    for (int i = 0; i < int(images.size()); ++i) {
        imwrite(path + "res" + to_string(i) + ".png", images[i]);
    }
    std::cout << "end" << std::endl;
}