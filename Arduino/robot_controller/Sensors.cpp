#include "Arduino.h"
#include "Sensors.h"
#include "util.h"
#include "math.h"

void init_sensors(){
  init_quad_enc();
  init_mag_enc();
  init_batt();
}


/******************************
      Quadrature Encoder
******************************/
 
uint8_t qePins[quadEncCnt][2] = quadEncPins;
long rpms[quadEncCnt];
float dt_rpm = pidLoopTime;
int cpr[quadEncCnt] = encCpr;
long previous_micros = 0;
Encoder quadenc[2] = {Encoder(qePins[0][0], qePins[0][1]), Encoder(qePins[1][1], qePins[1][0])};

void init_quad_enc() {
  for (uint8_t i = 0; i < quadEncCnt; i++) {
    // quadenc[i] = Encoder(qePins[i][0], qePins[i][1]);
    rpms[i] = 0;
    cpr[i] = 4 * cpr[i];
    previous_micros = micros();
  }
}

long update_rpm() {
  float count = 0;
  float rev = 0;
  dt_rpm = (micros() - previous_micros) / 1000;
  previous_micros = micros();
  for (uint8_t i = 0; i < quadEncCnt; i++) {
    count =(float) quadenc[i].readAndReset();
    rev = count / cpr[i];
    rpms[i] = (int)(rev * rev_to_rpm / dt_rpm );
  }
}

void reset_qenc(uint8_t i) {
  quadenc[i].write(0);
}


/******************************
      Magnetic Encoder
******************************/
AS5600 magEnc[magEncCt];
uint8_t mePins[magEncCt] = magEncPins;
float angles[magEncCt];
float offsets[magEncCt];

void init_mag_enc() {
  for (uint8_t i = 0; i < magEncCt; i++) {
    magEnc[i].begin(20);
    magEnc[i].setDirection(AS5600_CLOCK_WISE);
    angles[i] = 0.0;
    delay(100);
    offsets[i] = magEnc[i].rawAngle() * AS5600_RAW_TO_RADIANS;
    if(offsets[i] > PI ){
      offsets[i] -= 2 * PI;
    }
    Serial.print("Steering Offset: ");
    Serial.println(offsets[i]);
    delay(1000);
  }
}

void update_angle() {
  long rawAngle = 0;
  for (uint8_t i = 0; i < magEncCt; i++) {
    rawAngle = magEnc[i].rawAngle();
    angles[i] = rawAngle * AS5600_RAW_TO_RADIANS;
    if(angles[i] > PI ){
      angles[i] -= 2 * PI;
    }
    angles[i] -= offsets[i];
    if(angles[i] > PI ){
      angles[i] -= 2 * PI;
    }
    if(angles[i] < -PI ){
      angles[i] += 2 * PI;
    }

  }
}

void reset_mag_enc(uint8_t i) {
  offsets[i] = magEnc[i].rawAngle();
}


/******************************
      Batt Sensor
******************************/
float battVoltage[batPinCnt];
float battper[batPinCnt];
uint8_t bPins[batPinCnt] = batSenPin;

void init_batt() {
  for (uint8_t i = 0; i < batPinCnt; i++) {
    pinMode(bPins[i], INPUT);
    battVoltage[i] = 16.8;
    battper[i] = 100.0;
  }
}

void update_voltage() {
  for (uint8_t i = 0; i < batPinCnt; i++) {
    battVoltage[i] = (analogRead(bPins[i]) * 5 / 1024) * battscale;
    battper[i] = fmap(battVoltage[i], minbattV, maxbattV, 0, 100);
  }
}