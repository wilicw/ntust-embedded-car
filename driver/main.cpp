#include <wiringPi.h>

#include <csignal>
#include <cstdint>
#include <iostream>
#include <thread>

#include "bsp.h"
#include "communication.h"
#include "ir.h"
#include "model.h"
#include "motor.h"
#include "servo.h"
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

Motor motor(I2C_ADDR);
IR ir(IR_LEFT, IR_RIGHT);
Ultrasonic ur(EchoPin, TrigPin);
Servo servo(I2C_ADDR);
Vision v;
Communication commu;
Model m("/home/pi/project/carNN/model20.tflite");

void signal_callback_handler(int signum) {
    commu.exit_process();
    motor.stop();
    exit(signum);
}

void clear_queue() {
    static sign_item_t _;
    while (!commu.sign_queue->empty())
        commu.sign_queue->pop(_);
}

void control_task() {
    static int left_sensor, right_sensor;
    static double sign_distance;
    static double ur_distance;

    const static int right_speed = 150;
    const static int turning_speed = 120;
    const static int forward_speed = 50;
    const static float turning_scale = 0.9;
    const static int init_speed = 50;
    static int current_speed = init_speed;
    static double barrier;
    static uint32_t last_time_halt = 0;
    static uint8_t left_analog = 0, right_analog = 0;

    while (!Communication::is_exit_thread) {
        left_sensor = ir.left();
        right_sensor = ir.right();
        ur_distance = ur.distance();
        sign_distance = commu.sign_distance;

        barrier = current_speed * 0.085 + 10;
#ifdef ENABLE_MOTOR
        if (sign_distance <= barrier) {
            commu.halt_process();
            if (commu.sign_command == CMD_LEFT) {
                motor.turn(-right_speed + 20, right_speed);
                current_speed = init_speed;
                delay(180);
            } else if (commu.sign_command == CMD_RIGHT) {
                motor.turn(right_speed, -right_speed + 20);
                current_speed = init_speed;
                delay(180);
            } else if (commu.sign_command == CMD_HALT) {
                if (micros() - last_time_halt >= 1000 * 1000) {
                    motor.stop();
                    delay(3000);
                    motor.turn(init_speed, init_speed);
                    delay(400);
                    last_time_halt = micros();
                    current_speed = init_speed;
                }
            } else if (commu.sign_command == CMD_TURN) {
                current_speed = init_speed;
            } else if (commu.sign_command == CMD_GO) {
                if (left_sensor == WALL)
                    motor.turn(right_speed, -right_speed + 20);
                else
                    motor.turn(-right_speed + 20, right_speed);
                current_speed = init_speed;
            }
            commu.sign_command = CMD_NONE;
            commu.sign_distance = 1e9;
            clear_queue();
            commu.continue_process();
        } else if (ur_distance <= barrier * 0.85) {
            if (left_sensor == WALL)
                motor.turn(right_speed, -right_speed + 20);
            else
                motor.turn(-right_speed + 20, right_speed);
            current_speed = init_speed;
            cout << "Turning" << endl;
        } else if (!(left_sensor ^ right_sensor)) {
            left_analog = right_analog = 0;
            if (current_speed <= forward_speed) current_speed += 5;
            motor.turn(current_speed, current_speed);
            cout << "Forward" << endl;
        } else if (left_sensor == WALL) {
            if (left_analog < 255) left_analog += 5;
            right_analog = 0;
            motor.turn(
                turning_speed * turning_scale,
                turning_speed * (turning_scale - 0.15 - left_analog / 255.0));
        } else if (right_sensor == WALL) {
            if (right_analog < 255) right_analog += 5;
            left_analog = 0;
            motor.turn(
                turning_speed * (turning_scale - 0.15 - right_analog / 255.0),
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
    while (!Communication::is_exit_thread) {
        if (Communication::is_halt_process) continue;
        ret = cap.read(frame);
        if (!ret) continue;
        sign_item_t sign_item = v.process(frame);
        if (sign_item.cropped != nullptr && sign_item.center != nullptr)
            commu.sign_queue->push(sign_item);
    }
    cap.release();

#endif
}

void model_task() {
#ifdef ENABLE_TF
    while (!Communication::is_exit_thread) {
        while (!commu.sign_queue->empty()) {
            sign_item_t sign;
            commu.sign_queue->pop(sign);

            predict_t p = m.evaluate(*sign.cropped);
            sign.cropped->release();
            float distance = v.distance(*sign.center) - 12;
            cout << "model predict: " << p.possibility << " " << p.index << endl;
            cout << "distance: " << distance << endl;

            if (!Communication::is_halt_process && p.possibility >= 0.9f && commu.sign_distance > 5) {
                switch (p.index) {
                    case SIGN_STOP_LINE:
                    case SIGN_STOP_PIC:
                        commu.sign_command = CMD_HALT;
                        commu.sign_distance = distance - 8;
                        break;
                    case SIGN_GO_LEFT:
                    case SIGN_GO_RIGHT:
                    case SIGN_NO_LEFT:
                    case SIGN_NO_RIGHT:
                        commu.sign_command = CMD_GO;
                        commu.sign_distance = distance;
                        break;
                    case SIGN_ONLY_LEFT:
                        commu.sign_command = CMD_LEFT;
                        commu.sign_distance = distance;
                        break;
                    case SIGN_ONLY_RIGHT:
                        commu.sign_command = CMD_RIGHT;
                        commu.sign_distance = distance;
                        break;
                    case SIGN_NOT_GO:
                        commu.sign_command = CMD_TURN;
                        commu.sign_distance = distance;
                        break;
                    case SIGN_NO_STOP:
                    case SIGN_ONLY_GO:
                    default:
                        commu.sign_command = CMD_NONE;
                        commu.sign_distance = distance;
                        break;
                }
            } else {
                commu.sign_command = CMD_NONE;
                commu.sign_distance = 1e9;
                cout << "Halt" << endl;
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
