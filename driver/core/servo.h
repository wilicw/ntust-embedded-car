#ifndef DRIVER_SERVO_H
#define DRIVER_SERVO_H

#include <stdint.h>
#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <unistd.h>

class Servo {
private:
  int fd;
  uint16_t data[3] = {0x3, 0, 0};
public:
  void init();

  void turn(int, int);
};

#endif //DRIVER_SERVO_H
