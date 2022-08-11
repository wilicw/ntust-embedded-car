#include "ultrasonic.h"

void Ultrasonic::init() {
  pinMode(EchoPin, INPUT);
  pinMode(TrigPin, OUTPUT);
}

double Ultrasonic::distance() {
  digitalWrite(TrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(TrigPin, HIGH);
  delayMicroseconds(15);
  digitalWrite(TrigPin, LOW);

  unsigned int Trig_time;
  Trig_time = micros();
  while (!digitalRead(EchoPin))
    if ((micros() - Trig_time) > 30000) return -1.0;

  Trig_time = micros();
  while (digitalRead(EchoPin))
    if ((micros() - Trig_time) > 30000) return -1.0;

  return (micros() - Trig_time) * 340.0 / 2.0 / 10000.0;
}