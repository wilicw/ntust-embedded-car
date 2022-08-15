#ifndef DRIVER_MODEL_H
#define DRIVER_MODEL_H

#include <tensorflow/lite/interpreter.h>
#include <tensorflow/lite/kernels/register.h>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

using namespace std;

class Model {
   private:
    unique_ptr<tflite::FlatBufferModel> model;

   public:
    Model(const char *);
    void evaluate(cv::Mat&);
};

#endif  // DRIVER_MODEL_H
