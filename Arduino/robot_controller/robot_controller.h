#ifndef ROBOT_CONTROLLER_H_
#define ROBOT_CONTROLLER_H_

extern int controlMode;
extern uint16_t controlLoop;
extern uint16_t rcRate;
extern uint16_t commRate;
extern float cmdThrottle;
extern float cmdSteering;
extern float cmdAux;
extern float rcThr;
extern float rosThr;
extern float rosAux;
extern float rcSteering;
extern float rosSteering;
extern float rcAcc;
extern int ctrlEn;
extern int minStr, maxStr, minThr, maxThr, offsetSteering;
extern uint8_t rosMsgFlag;

uint8_t failsafe();
void joy_step();

#endif




