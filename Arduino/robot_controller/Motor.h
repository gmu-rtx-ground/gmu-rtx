#ifndef MOTOR_H_
#define MOTOR_H_

#include <Arduino.h>
#include <stdint.h>

extern int servoValues[2];


void init_servos();
void write_servos();
void servo_failsafe();
int throttle_PID(int, int);

#endif