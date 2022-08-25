#include "vision.h"
#include "communication.h"
#include<iostream>
#include<ctime>
#include<opencv2/opencv.hpp>
#include "pictureIO.h"

//#define PREPARE_TRAINGING_DATA
//#define DEBUG
//#define CAR

using std::cout;
using std::endl;

int main(int argc, char** argv) {
    Vision v;
	string folder_path = "D:/Team/rasp_car/program/image_process/all_data_classifications/debug/";
    folder_path = "D:/Team/rasp_car/program/image_process/dataset0817/debug/";
//    folder_path = "D:\\Team\\rasp_car\\program\\image_process\\all_mixdataset(can't use)\\picture\\";

    string file_name = "run";

    if(argc >= 2){
//        folder_path = argv[1];
//        file_name = argv[2];
    }

    const int Q = 10000;
    cout << folder_path << " " << file_name << endl;
	vector<pictureIO::pic_info> pictures = pictureIO::load_pictures(folder_path, file_name, Q, 1);
	vector<pictureIO::pic_info> cropped_images;
    cout << "_START REG_" << endl;
    std::clock_t clk = std::clock();
	for (pictureIO::pic_info & pic : pictures) {
		try {
            sign_item_t found = v.process(pic.img);
            if(found.cropped != nullptr){
                cropped_images.push_back(pictureIO::pic_info(pic.index, *found.cropped));
            }else{
//                cout << "nothing found: " << pic.index << endl;
            }
//            cout << "LOADED: " << pic.index << endl;
		}
		catch (const char* str) {
			cout << pic.index << " " << str << "\n";
		}
	}
    cout << clk << endl;
	pictureIO::write_pictures(folder_path + "testdata/", cropped_images);
}