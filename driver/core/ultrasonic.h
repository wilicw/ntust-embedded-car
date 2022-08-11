#ifndef DRIVER_ULTRASONIC_H
#define DRIVER_ULTRASONIC_H

#include <wiringPi.h>

class Ultrasonic {
public:
  void init();

  double distance();
};

#endif //DRIVER_ULTRASONIC_H
