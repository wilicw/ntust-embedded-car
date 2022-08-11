#include <signal.h>
#include <stdint.h>
#include <wiringPi.h>
#include <iostream>

#include "bsp.h"

#include "ir.h"
#include "motor.h"
#include "servo.h"
#include "ultrasonic.h"

//#define RUNRUNRUN

using namespace std;

Motor motor(I2C_ADDR);
IR ir(IR_LEFT, IR_RIGHT);
Ultrasonic ur(EchoPin, TrigPin);
Servo servo(I2C_ADDR);

void signal_callback_handler(int signum) {
    motor.stop();
    exit(signum);
}

int main() {
    wiringPiSetup();
    servo.init();
    motor.init();
    ir.init();
    ur.init();

    static int left_sensor, right_sensor;
    static double distance;

    signal(SIGINT, signal_callback_handler);

    const static double barrier = 35.50;
    const static int turning_speed = 180;
    const static int forward_speed = 150;
    servo.turn(1, 90);
    servo.turn(2, 125);

    try {
        for (;;) {
            servo.turn(1, 90);
            servo.turn(2, 125);
            left_sensor = ir.left();
            right_sensor = ir.right();
            distance = ur.distance();

            if (distance < barrier) {
#ifdef RUNRUNRUN
                motor.turn(turning_speed, -turning_speed);
#else
                cout << "Turn Right" << endl;
#endif
            } else if (false) {
#ifdef RUNRUNRUN
                motor.turn(-turning_speed, turning_speed);
#else
                cout << "Turn Left" << endl;
#endif
            } else if (left_sensor && right_sensor == true) {
#ifdef RUNRUNRUN
                motor.turn(forward_speed, forward_speed);
#else
                cout << "Forward" << endl;
#endif
            } else if (left_sensor == false) {
#ifdef RUNRUNRUN
                motor.turn(turning_speed * 0.8, -turning_speed * 0.8);
#else
                cout << "Turn Right by 0.8" << endl;
#endif
            } else if (right_sensor == false) {
#ifdef RUNRUNRUN
                motor.turn(-turning_speed * 0.8, turning_speed * 0.8);
#else
                cout << "Turn Left by 0.8" << endl;
#endif
            } else {
#ifdef RUNRUNRUN
                motor.turn(forward_speed, forward_speed);
#else
                cout << "Forward" << endl;
#endif
            }

            /* !!! */
            delay(10);  // !! WARNING DO NOT REMOVE !!
            /* !!! */
        }
    } catch (...) {
        motor.stop();
    }

    motor.stop();

    return 0;
}
