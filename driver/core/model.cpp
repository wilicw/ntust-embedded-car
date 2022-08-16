#include "model.h"

Model::Model(const char *model_path) {
    this->model = tflite::FlatBufferModel::BuildFromFile(model_path);
    tflite::InterpreterBuilder(*model, this->resolver)(&(this->interpreter), 4);
    this->interpreter->AllocateTensors();
}

predict_t Model::evaluate(cv::Mat input) {
    predict_t predict;
    float *inputTensor = this->interpreter->typed_input_tensor<float>(0);
    float *y = this->interpreter->typed_output_tensor<float>(0);

    input.convertTo(input, CV_32F, 1.f/255, 0);
    memcpy(inputTensor, input.data, 50 * 50 * 3 * sizeof(float));
    this->interpreter->Invoke();

    float *max_y = max_element(y, y+11*sizeof(float ));
    predict.possibility = *max_y;
    predict.index = distance(y, max_y);

    return predict;
}
