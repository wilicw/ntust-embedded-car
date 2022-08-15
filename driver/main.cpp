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

// #define ENABLE_MOTOR
#define ENABLE_CV
#define CAPTURE_IMG
#define CAR_RUNNING

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

//Model m("model.tflite");

void signal_callback_handler(int signum) {
    exit_thread = true;
    motor.stop();
    exit(signum);
}

void control_task() {
    static int left_sensor, right_sensor;
    static double distance;

    const static int turning_speed = 220;
    const static int forward_speed = 180;
    const static float turning_scale = 0.9;
    const static int init_speed = 80;
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
            if (left_sensor == WALL)
                motor.turn(turning_speed, -turning_speed + 20);
            else
                motor.turn(-turning_speed + 20, turning_speed);
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
#ifdef CAPTURE_IMG
	
opencamera:
    static cv::VideoCapture cap(0);
    //    cout << "opening" << endl;
    if (!cap.isOpened()) goto opencamera;

    cap.set(cv::CAP_PROP_FPS, 90);
    static unsigned int pics = 0;
    static int ret;
    static cv::Mat frame;
    cout << "CV camera opened" << endl;
    while (!exit_thread) {
        ret = cap.read(frame);
        if (!ret) continue;
        // cout << ss.str() << endl;
        cv::imwrite("mypicture/pic" + to_string(pics) + ".jpg", frame);
        pics++;
        delay(50);
    }
    cap.release();
#elif defined(CAR_RUNNING)
	cv::Mat a = cv::imread("D:/Team/rasp_car/program/image_process/notright_pic/frame2.jpg");
	//Mat a = imread("D:/RayChang/IMG_0102.jpg");
	clock_t clk = clock();
	try {
		cv::resize(a, a, cv::Size(640, 480));
		sign_info found = sign_finding(a);
		std::cout << found.area << " " << found.center_position << std::endl;
		imshow("FND", found.cropped);
		cv::waitKey(0);
		cv::destroyAllWindows();
	}
	catch (...) {
		std::cout << "Nothing found" << std::endl;
	}
#endif
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
