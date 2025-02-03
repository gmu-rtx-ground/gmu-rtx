#include "pid.h"
#include "util.h"

float integral;
float lastError;
float error;
float kP = kp;
float kI = ki;
float kD = kd;
float setAcc = desACC ;

void init_pid(){
  integral = 0.0;
  lastError = 0.0;
}

int compute_pid(float curSpeed, float setSpeed, float dTime){ 
  float output = 0;
   float esc_val = 0;
  if (abs(setSpeed)<0.03){
    setSpeed = 0.0;
  }
  if(setSpeed != 0){
     esc_val = (setSpeed * setSpeed) * 100 *(setSpeed/abs(setSpeed));

  }
  else{
    integral = 0;
  }
  setSpeed *= 100;
  error = (setSpeed - curSpeed); 
  integral = (integral +  clip(error, -maxDesc*dTime/maxSpeed, maxAcc*dTime/maxSpeed )) ;
  integral = clip(integral, -80, 80);
  float derivative = (error - lastError) * dTime;
  error *= setAcc/maxAcc;
  output = esc_val + ( (kP * error) + (kI * integral) + (kD * derivative));
  output = clip(output, -100.0, 100.0);
  // Serial.print("cmdthr: ");
  // Serial.print(setSpeed);
  // Serial.print("\t curSpeed: ");
  // Serial.print(curSpeed);
  // Serial.print(" \t Error: ");
  // Serial.print(error); 
  // Serial.print(" \t output: ");
  // Serial.println(output);

  return output;  
}