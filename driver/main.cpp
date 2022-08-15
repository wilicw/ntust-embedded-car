#include <signal.h>
#include <stdint.h>
#include <wiringPi.h>
#include <atomic>
#include <iostream>
#include <thread>

#include "bsp.h"

#include "ir.h"
#include "model.h"
#include "motor.h"
#include "servo.h"
#include "ultrasonic.h"
#include "vision.h"

#define ENABLE_MOTOR
#define ENABLE_CV

#ifdef ENABLE_CV
#include <opencv2/opencv.hpp>
#endif

#define NOWALL false

using namespace std;

volatile static atomic<bool> exit_thread(false);

Motor motor(I2C_ADDR);
IR ir(IR_LEFT, IR_RIGHT);
Ultrasonic ur(EchoPin, TrigPin);
Servo servo(I2C_ADDR);

Vision v;

// Model m("model.tflite");

void signal_callback_handler(int signum) {
    exit_thread = true;
    motor.stop();
    exit(signum);
}

void control_task() {
    static int left_sensor, right_sensor;
    static double distance;

    const static int turning_speed = 185;
    const static int forward_speed = 180;
    const static float turning_scale = 0.9;
    const static int init_speed = 50;
    static int current_speed = init_speed;
    static double barrier;
    static uint8_t left_analog = 0, right_analog = 0;

    while (!exit_thread) {
        servo.turn(1, 90);
        servo.turn(2, 125);
        left_sensor = ir.left();
        right_sensor = ir.right();
        distance = ur.distance();

        barrier = current_speed * 0.09 + 13;
#ifdef ENABLE_MOTOR
        if (distance < barrier) {
            cout << distance << endl;
            if (left_sensor == NOWALL)
                motor.turn(turning_speed, -turning_speed + 20);
            else
                motor.turn(-turning_speed + 20, turning_speed);
            current_speed = init_speed;
        } else if (!(left_sensor ^ right_sensor)) {
            cout << "Forward" << endl;
            left_analog = right_analog = 0;
            if (current_speed <= forward_speed) current_speed += 5;
            motor.turn(current_speed, current_speed);
        } else if (left_sensor == NOWALL) {
            if (left_analog < 255) left_analog += 5;
            right_analog = 0;
            cout << "Right" << endl;
            motor.turn(
                turning_speed * turning_scale,
                turning_speed * (turning_scale - 0.2 - left_analog / 255.0));
        } else if (right_sensor == NOWALL) {
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
    static cv::VideoCapture cap(0);
    while (!cap.isOpened()) {
    }

    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 680);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    // cap.set(cv::CAP_PROP_FPS, 90);

    static int ret;
    static cv::Mat frame;
    static cv::Mat result;

    while (!exit_thread) {
        ret = cap.read(frame);
        if (!ret) continue;

        v.process(frame, result);

        cv::imwrite("test.jpg", frame);
        delay(10);
    }
    cap.release();
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

    servo.turn(1, 90);
    servo.turn(2, 125);

    thread control_thread(control_task);
    thread vision_thread(vision_task);

    control_thread.join();
    vision_thread.join();

    motor.stop();

    return 0;
}
