#ifndef SENSORS_H_
#define SENSORS_H_

#include <Arduino.h>
#include <Encoder.h>
#include <AS5600.h>
#include "def.h"

#define rev_to_rpm (60000 * f_diff_ratio )
void init_sensors();

/******************************
      Quadrature Encoder
******************************/
extern long rpms[quadEncCnt];
extern float dt_rpm;

void init_quad_enc();
long update_rpm();
void reset_qenc();

/******************************
      Magnetic Encoder
******************************/
extern float angles[magEncCt];

void init_mag_enc();
void update_angle();
void reset_mag_enc();


/******************************
      Batt Sensor
******************************/
extern float battVoltage[batPinCnt];
extern float battper[batPinCnt];

void init_batt();
void update_voltage();

#endif