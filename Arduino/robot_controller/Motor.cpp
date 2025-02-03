#include "def.h"
#include "util.h"
#include "Motor.h"
#include <Servo.h>

Servo steeringServo;
Servo ESC;

static int servoValues[2] = {0, 0};
int minStr = minSt;
int maxStr = maxSt;
int minThr = minTh;
int maxThr = maxTh;
int offsetSteering = 0;

void init_servos(){
  // Attach the servos to actual pins
  steeringServo.attach(steeringPin);
  ESC.attach(escPin); 
  
  servoValues[0] = 1500;
  servoValues[1] = 1500;

  // Initialize Servos and ESC setting
  steeringServo.writeMicroseconds(1500);
  ESC.writeMicroseconds(1500);
  delay(200);
  steeringServo.writeMicroseconds(1000);
  delay(500);
  steeringServo.writeMicroseconds(1500);
  delay(1000);
}

void write_servos() {

  steeringServo.writeMicroseconds(constrain(servoValues[0] + offsetSteering, minSt, maxSt));
  ESC.writeMicroseconds(servoValues[1]);
  // ESC.writeMicroseconds(1500);
}

void servo_failsafe(){
  ESC.writeMicroseconds(1500);
  steeringServo.writeMicroseconds(1850);
}

