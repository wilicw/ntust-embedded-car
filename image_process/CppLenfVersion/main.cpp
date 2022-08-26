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

    cout << "!!! START REG !!!" << endl;
    std::clock_t clk = std::clock();

    for (int i = 0; i<Q; i++) {
        try{
            string path = folder_path + file_name + to_string(i) +".jpg";
            cv::Mat img = cv::imread(path)(cv::Rect(5, 5, 630, 470));
            Vision::index = i;
            sign_item_t found = v.process(img);
            if(found.cropped != nullptr){
//                cout << pic.index << " founded" << endl;
                string ind = to_string(i);
                ind = string(6 - std::min(size_t(6), ind.length()), '0') + ind;
                cv::imwrite(folder_path + "testdata/res" + ind + ".png", *(found.cropped));
            }else{

            }
        }catch (...){
//            std::cout << "ERROR!!!" << std::endl;
            continue;
        }

    }
    cout << "end with clock(ms) : " << std::clock() - clk << endl;
}