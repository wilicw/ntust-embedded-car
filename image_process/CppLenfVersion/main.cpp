#include "vision.h"
#include "communication.h"
#include<iostream>
#include<ctime>
#include<opencv2/opencv.hpp>
#include "pictureIO.h"

using std::cout;
using std::endl;

int main(int argc, char** argv) {
    Vision v;
    string folder_path;
    string file_name;
    folder_path = "D:/Team/rasp_car/program/image_process/all_data_classifications/stop_line/";
    file_name = "stop_line_";
    folder_path = "D:/Team/rasp_car/program/image_process/dataset0817/debug/";
    file_name = "run";
//    folder_path = "D:\\Team\\rasp_car\\program\\image_process\\all_mixdataset(can't use)\\picture\\";

    if(argc >= 2 && argv[1][0] == 'D'){
        folder_path = argv[1];
        file_name = argv[2];
    }

    const int Q = 10000;
    cout << folder_path << " " << file_name << endl;
    vector<pictureIO::pic_info> pictures = pictureIO::load_pictures(folder_path, file_name, Q, 1);
    vector<pictureIO::pic_info> cropped_images;
    cout << "!!! START REG !!!" << endl;
    std::clock_t clk = std::clock();
    for (pictureIO::pic_info & pic : pictures) {
        try {
            Vision::index = pic.index;
            sign_item_t found = v.process(pic.img);
            if(found.cropped != nullptr){
//                cout << pic.index << " founded" << endl;
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