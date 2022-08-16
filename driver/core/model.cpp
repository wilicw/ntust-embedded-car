#include "model.h"

Model::Model(const char *model_path) {
    this->model = tflite::FlatBufferModel::BuildFromFile(model_path);
    tflite::InterpreterBuilder(*model, this->resolver)(&(this->interpreter), 4);
    this->interpreter->ResizeInputTensor(0, {1, 50, 50, 3});
    this->interpreter->AllocateTensors();
}

void Model::evaluate(cv::Mat input) {
    float *inputTensor = this->interpreter->typed_input_tensor<float>(0);
    float *y = this->interpreter->typed_output_tensor<float>(0);

    input /= 255.0;
    memcpy(inputTensor, input.ptr<float>(0), 50 * 50 * 3 * sizeof(float));
    this->interpreter->Invoke();

    for (int i = 0; i < 11; i++) cout << y[i] << " ";
    cout << endl;
    return;
}
