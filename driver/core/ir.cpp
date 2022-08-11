#include "ir.h"

IR::IR(int left_pin, int right_pin)
    : left_pin(left_pin), right_pin(right_pin){};

void IR::init() {
    pinMode(this->right_pin, INPUT);
    pinMode(this->left_pin, INPUT);
}

int IR::left() { return digitalRead(this->left_pin); }

int IR::right() { return digitalRead(this->right_pin); }
