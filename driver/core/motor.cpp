#include "motor.h"

Motor::Motor(int addr) : addr(addr) {}

void Motor::init() { this->fd = wiringPiI2CSetup(this->addr); }

void Motor::turn(int left, int right) {
    data[1] = left < 0 ? 0 : 1;
    data[3] = right < 0 ? 0 : 1;

    data[2] = left < 0 ? -left : left;
    data[4] = right < 0 ? -right : right;

    write(fd, data, 5);
}

void Motor::stop() {
    data[1] = 0;
    data[2] = 0;
    data[3] = 0;
    data[4] = 0;
    write(fd, data, 5);
}
