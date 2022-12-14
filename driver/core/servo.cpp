#include "servo.h"

Servo::Servo(int addr) : addr(addr) {}

void Servo::init() { this->fd = wiringPiI2CSetup(this->addr); }

void Servo::turn(int id, int angle) {
    data[1] = id;
    data[2] = angle < 0 ? 0 : angle > 180 ? 180
                                          : angle;
    write(fd, data, 3);
}
