#ifndef DRIVER_COMMUNICATION_H
#define DRIVER_COMMUNICATION_H

#include <atomic>
#include <boost/lockfree/queue.hpp>
#include <opencv2/opencv.hpp>

using namespace std;

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

class Communication {
   public:
    Communication();
    void halt_process();
    void continue_process();
    void exit_process();
    boost::lockfree::queue<sign_item_t, boost::lockfree::fixed_sized<true>>* sign_queue;
    volatile static atomic<bool> is_exit_thread;
    volatile static atomic<bool> is_halt_process;
    static atomic<cmd_t> sign_command;
    static atomic<float> sign_distance;
};

#endif  // DRIVER_COMMUNICATION_H
