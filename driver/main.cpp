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
    
#ifdef RUNRUNRUN
    try {
        for (;;) {
			servo.turn(1, 90);
			servo.turn(2, 125);
            left_sensor = ir.left();
            right_sensor = ir.right();
            distance = ur.distance();

            if (distance < barrier) {
                motor.turn(turning_speed, -turning_speed);
            } else if (false) {
                motor.turn(-turning_speed, turning_speed);
            } else if (left_sensor && right_sensor == true) {
                motor.turn(forward_speed, forward_speed);
            } else if (left_sensor == false) {
                motor.turn(turning_speed * 0.8, -turning_speed * 0.8);
            } else if (right_sensor == false) {
                motor.turn(-turning_speed * 0.8, turning_speed * 0.8);
            } else {
                motor.turn(forward_speed, forward_speed);
            }

            /* !!! */
            delay(10); // !! WARNING DO NOT REMOVE !!
            /* !!! */
        }
    }
    catch (...) {
        motor.stop();
    }
#endif

	motor.stop();
	
    return 0;
}
