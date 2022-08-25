#ifndef DRIVER_COMMUNICATION_H
#define DRIVER_COMMUNICATION_H

#include <opencv2/opencv.hpp>

typedef enum {
    CMD_HALT,
    CMD_GO,
    CMD_LEFT,
    CMD_RIGHT,
    CMD_TURN,
    CMD_NONE,
} cmd_t;

typedef struct {
    cv::Mat* cropped;
    cv::Point* center;
} sign_item_t;

typedef struct {
    cmd_t command;
    float distance;
} cmd_item_t;

#endif  // DRIVER_COMMUNICATION_H
