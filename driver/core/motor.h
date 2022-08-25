#ifndef DRIVER_MOTOR_H
#define DRIVER_MOTOR_H

#include <stdint.h>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>

class Motor {
   private:
    int addr;
    int fd;
    int _left = 0;
    int _right = 0;
    uint8_t data[5] = {0x1, 0, 0, 0, 0};

   public:
    Motor(int);

    void init();

    void turn(int, int);

    void stop();
};

#endif  // DRIVER_MOTOR_H
