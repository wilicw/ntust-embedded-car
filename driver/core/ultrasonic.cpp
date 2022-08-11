#include "ultrasonic.h"

Ultrasonic::Ultrasonic(int echo, int trig) : echo_pin(echo), trig_pin(trig) {}

void Ultrasonic::init() {
    pinMode(this->echo_pin, INPUT);
    pinMode(this->trig_pin, OUTPUT);
}

double Ultrasonic::distance() {
    digitalWrite(this->trig_pin, LOW);
    delayMicroseconds(2);
    digitalWrite(this->trig_pin, HIGH);
    delayMicroseconds(15);
    digitalWrite(this->trig_pin, LOW);

    unsigned int Trig_time;
    Trig_time = micros();
    while (!digitalRead(this->echo_pin))
        if ((micros() - Trig_time) > 30000) return -1.0;

    Trig_time = micros();
    while (digitalRead(this->echo_pin))
        if ((micros() - Trig_time) > 30000) return -1.0;

    return (micros() - Trig_time) * 340.0 / 2.0 / 10000.0;
}
