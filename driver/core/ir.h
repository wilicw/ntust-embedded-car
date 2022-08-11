#ifndef DRIVER_IR_H
#define DRIVER_IR_H

#include <wiringPi.h>

class IR {
   private:
    int left_pin, right_pin;

   public:
    IR(int, int);

    void init();

    int left();

    int right();
};

#endif  // DRIVER_IR_H
