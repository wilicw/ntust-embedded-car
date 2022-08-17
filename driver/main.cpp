#include <wiringPi.h>

#include <atomic>
#include <boost/lockfree/queue.hpp>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <thread>

#include "bsp.h"
#include "ir.h"
#include "model.h"
#include "motor.h"
#include "servo.h"
#include "sign_flag.h"
#include "ultrasonic.h"
#include "vision.h"

#define ENABLE_MOTOR
#define ENABLE_CV
#define ENABLE_TF

#ifdef ENABLE_CV
#include <opencv2/opencv.hpp>
#endif

#define WALL false

using namespace std;

volatile static atomic<bool> exit_thread(false);

Motor motor(I2C_ADDR);
IR ir(IR_LEFT, IR_RIGHT);
Ultrasonic ur(EchoPin, TrigPin);
Servo servo(I2C_ADDR);
Vision v;
Model m("/home/pi/project/carNN/model.tflite");

boost::lockfree::queue<cv::Mat*, boost::lockfree::fixed_sized<true>> cv2model_queue(256);
boost::lockfree::queue<int, boost::lockfree::fixed_sized<true>> model2driver_queue(256);

void signal_callback_handler(int signum) {
    exit_thread = true;
    motor.stop();
    exit(signum);
}

void control_task() {
    static int left_sensor, right_sensor;
    static double distance;

    const static int right_speed = 180;
    const static int turning_speed = 120;
    const static int forward_speed = 80;
    const static float turning_scale = 0.9;
    const static int init_speed = 50;
    static int current_speed = init_speed;
    static double barrier;
    static uint8_t left_analog = 0, right_analog = 0;

    while (!exit_thread) {
        while (!model2driver_queue.empty()) {
            sign_cmd_t cmd;
            model2driver_queue.pop(cmd);
            cout << cmd << endl;
        }

        servo.turn(1, 125);
        servo.turn(2, 90);
        left_sensor = ir.left();
        right_sensor = ir.right();
        distance = ur.distance();

        barrier = current_speed * 0.09 + 12;
#ifdef ENABLE_MOTOR
        if (distance <= barrier) {
            cout << distance << endl;
            if (left_sensor == WALL)
                motor.turn(right_speed, -right_speed + 20);
            else
                motor.turn(-right_speed + 20, right_speed);
            delay(70);
            current_speed = init_speed;
        } else if (!(left_sensor ^ right_sensor)) {
            cout << "Forward" << endl;
            left_analog = right_analog = 0;
            if (current_speed <= forward_speed) current_speed += 5;
            motor.turn(current_speed, current_speed);
        } else if (left_sensor == WALL) {
            if (left_analog < 255) left_analog += 5;
            right_analog = 0;
            cout << "Right" << endl;
            motor.turn(
                turning_speed * turning_scale,
                turning_speed * (turning_scale - 0.2 - left_analog / 255.0));
        } else if (right_sensor == WALL) {
            if (right_analog < 255) right_analog += 5;
            left_analog = 0;
            cout << "Left" << endl;
            motor.turn(
                turning_speed * (turning_scale - 0.2 - right_analog / 255.0),
                turning_speed * turning_scale);
        }
#endif
        /* !!! */
        delay(10);  // !! WARNING DO NOT REMOVE !!
        /* !!! */
    }
    motor.stop();
}

void vision_task() {
#ifdef ENABLE_CV

opencamera:
    static cv::VideoCapture cap(0);
    if (!cap.isOpened()) goto opencamera;

    cap.set(cv::CAP_PROP_FPS, 90);

    static int ret;
    static cv::Mat frame;
    while (!exit_thread) {
        ret = cap.read(frame);
        if (!ret) continue;

        //cv::imwrite("frame.jpg", frame);
        sign_info_t found = std::move(v.processing(frame));
        if (found.area == 0) continue;
        auto cropped = new cv::Mat(found.cropped);
        cv2model_queue.push(cropped);
    }
    cap.release();

#endif
}

void model_task() {
#ifdef ENABLE_TF
    while (!exit_thread) {
        while (!cv2model_queue.empty()) {
            cv::Mat* sign;
            cv2model_queue.pop(sign);

            //cv::imwrite("test.jpg", *sign);
            predict_t p = m.evaluate(*sign);
            delete sign;
            cout << p.possibility << " " << p.index << endl;

            switch (p.index) {
                case SIGN_STOP_LINE:
                case SIGN_STOP_PIC:
                    model2driver_queue.push(CMD_HALT);
                    break;
                case SIGN_ONLY_GO:
                case SIGN_GO_LEFT:
                case SIGN_GO_RIGHT:
                case SIGN_NO_STOP:
                case SIGN_NO_LEFT:
                case SIGN_NO_RIGHT:
                    model2driver_queue.push(CMD_GO);
                    break;
                case SIGN_ONLY_LEFT:
                    model2driver_queue.push(CMD_LEFT);
                    break;
                case SIGN_ONLY_RIGHT:
                    model2driver_queue.push(CMD_RIGHT);
                    break;
                case SIGN_NOT_GO:
                    model2driver_queue.push(CMD_TURN);
                    break;
                default:
                    break;
            }
        }
    }
#endif
}

int main() {
    signal(SIGINT, signal_callback_handler);
    signal(SIGABRT, signal_callback_handler);

    wiringPiSetup();
    servo.init();
    motor.init();
    ir.init();
    ur.init();

    thread control_thread(control_task);
    thread vision_thread(vision_task);
    thread model_thread(model_task);

    control_thread.join();
    vision_thread.join();
    model_thread.join();

    motor.stop();

    return 0;
}
