#ifndef TASKS_H_
#define TASKS_H_

#include "remote_rc.h"
#include "ros_node.h"
#include "Motor.h"
#include "def.h"
#include "pid.h"
#include "robot_controller.h"
#include "Sensors.h"

#define failsafec_id 0 
#define pid_id 1
#define joy_id 2
#define rpm_id 3
#define status_id 4

// #define maxRpm  (maxSpeed*1000*60) / (3.14285*tire_dia)
#define maxRpm  1910
#define thrRange (maxTh-minTh)/2

void init_task();
int failsafe();
void pid_step();
void rpm_step();
void status_step();
void joy_step();

#endif