#ifndef DRIVER_MOTOR_H
#define DRIVER_MOTOR_H

#include <stdint.h>
#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <unistd.h>

class Motor {
private:
  int fd;
  uint8_t data[5] = {0x1, 0, 0, 0, 0};

public:
  void init();

  void turn(int, int);

  void stop();
};

#endif //DRIVER_MOTOR_H
