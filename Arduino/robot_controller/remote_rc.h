#ifndef REMOTE_RC_H_
#define REMOTE_RC_H_

#include <Arduino.h>
#include "def.h"
#include "robot_controller.h"

#define maxThrCap 450

extern unsigned long lastRcTime;
extern volatile uint16_t rcValue[PCINT_PIN_COUNT];
extern float rcChannels[6];
void init_rc();
void update_rc();


#endif