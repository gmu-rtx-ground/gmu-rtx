#ifndef PID_H_
#define PID_H_

#include <Arduino.h>
#include "def.h"

extern float kP, kI, kD, setAcc;
void init_pid();
int compute_pid(float curSpeed, float setSpeed, float dTime);

#endif