#include <signal.h>
#include <stdint.h>
#include <wiringPi.h>
#include <iostream>
#include <thread>

#include "bsp.h"

#include "ir.h"
#include "motor.h"
#include "servo.h"
#include "ultrasonic.h"

//#define RUN_CAR

using namespace std;

Motor motor(I2C_ADDR);
IR ir(IR_LEFT, IR_RIGHT);
Ultrasonic ur(EchoPin, TrigPin);
Servo servo(I2C_ADDR);

void signal_callback_handler(int signum) {
    motor.stop();
    exit(signum);
}

void control_task() {
    static int left_sensor, right_sensor;
    static double distance;

    const static double barrier = 35.50;
    const static int turning_speed = 180;
    const static int forward_speed = 150;

    for (;;) {
        servo.turn(1, 90);
        servo.turn(2, 125);
        left_sensor = ir.left();
        right_sensor = ir.right();
        distance = ur.distance();

        if (distance < barrier) {
#ifdef RUN_CAR
            motor.turn(turning_speed, -turning_speed);
#else
            cout << "Turn Right" << endl;
#endif
        } else if (false) {
#ifdef RUN_CAR
            motor.turn(-turning_speed, turning_speed);
#else
            cout << "Turn Left" << endl;
#endif
        } else if (left_sensor && right_sensor == true) {
#ifdef RUN_CAR
            motor.turn(forward_speed, forward_speed);
#else
            cout << "Forward" << endl;
#endif
        } else if (left_sensor == false) {
#ifdef RUN_CAR
            motor.turn(turning_speed * 0.8, -turning_speed * 0.8);
#else
            cout << "Turn Right by 0.8" << endl;
#endif
        } else if (right_sensor == false) {
#ifdef RUN_CAR
            motor.turn(-turning_speed * 0.8, turning_speed * 0.8);
#else
            cout << "Turn Left by 0.8" << endl;
#endif
        } else {
#ifdef RUN_CAR
            motor.turn(forward_speed, forward_speed);
#else
            cout << "Forward" << endl;
#endif
        }

        /* !!! */
        delay(10);  // !! WARNING DO NOT REMOVE !!
        /* !!! */
    }
}

void vision_task() {
    for (;;) {
        delay(100);
    }
}

int main() {
    signal(SIGINT, signal_callback_handler);

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
