#ifndef DRIVER_MODEL_H
#define DRIVER_MODEL_H

#include <tensorflow/lite/interpreter.h>
#include <tensorflow/lite/kernels/register.h>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

using namespace std;

typedef enum {
    SIGN_GO_LEFT,
    SIGN_GO_RIGHT,
    SIGN_NOT_GO,
    SIGN_ONLY_GO,
    SIGN_ONLY_LEFT,
    SIGN_ONLY_RIGHT,
    SIGN_STOP_LINE,
    SIGN_STOP_PIC,
    SIGN_NO_LEFT,
    SIGN_NO_RIGHT,
    SIGN_NO_STOP
} sign_classification_t;

typedef struct {
    float possibility;
    sign_classification_t index;
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
