#ifndef DRIVER_MODEL_H
#define DRIVER_MODEL_H

#include <tensorflow/lite/interpreter.h>
#include <tensorflow/lite/kernels/register.h>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

using namespace std;

typedef struct {
    float possibility;
    int index;
} predict_t;

class Model {
   private:
    unique_ptr<tflite::FlatBufferModel> model;
    tflite::ops::builtin::BuiltinOpResolver resolver;
    unique_ptr<tflite::Interpreter> interpreter;

   public:
    Model(const char *);
    predict_t evaluate(cv::Mat);
};

#endif  // DRIVER_MODEL_H
