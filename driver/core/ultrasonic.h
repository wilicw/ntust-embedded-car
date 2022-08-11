#ifndef DRIVER_ULTRASONIC_H
#define DRIVER_ULTRASONIC_H

#include <wiringPi.h>

class Ultrasonic {
   private:
    int echo_pin, trig_pin;

   public:
    Ultrasonic(int, int);

    void init();

    double distance();
};

#endif  // DRIVER_ULTRASONIC_H
