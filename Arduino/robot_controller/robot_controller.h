#ifndef ROBOT_CONTROLLER_H_
#define ROBOT_CONTROLLER_H_

namespace controller {
    extern uint8_t controlMode;
    extern uint16_t controlLoop;
    extern uint16_t commRate;
    extern uint16_t rcRate;
    extern float cmdThrottle;
    extern float cmdSteering;
    extern float cmdAux;
}   // namespace controller

uint8_t failsafe();
void joy_step();

#endif  // ROBOT_CONTROLLER_H_
