#include "model.h"

Model::Model(const char* model_path) {
    this->model = tflite::FlatBufferModel::BuildFromFile(model_path);
}

void Model::evaluate(cv::Mat& input) { return; }
