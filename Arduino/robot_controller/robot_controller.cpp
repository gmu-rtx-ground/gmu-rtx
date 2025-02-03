
#include "Arduino.h"
#include "def.h"
#include "ros_node.h"
#include "remote_rc.h"
#include "Sensors.h"
#include "pid.h"
#include "Motor.h"
#include "tasks.h"
#include "robot_controller.h"
#include "util.h"

int controlMode = 0;
float commRate = 0.0;
float rcRate = 0.0;
float comm = 0.0;
float controlLoop = 0.0;
float cmdThrottle = 0.0 ;
float cmdSteering = 0.0;

void setup() {
  Serial.begin(57600);
  init_ros_node();
  init_rc();
  init_servos();
  update_rc();
  servoValues[0] = (int) fmap(rcChannels[0], -1.0, 1.0, minStr, maxStr);
  servoValues[1] = 1500;
  write_servos();
  delay(200);
  init_sensors();
  init_pid();
  init_task();  
}

void loop (){
  update_rc();
  int fState = failsafe();
  if(fState > 1){
    Serial.print("Failsafe Active Error Code: ");
    Serial.println(fState);
    nodeHandle.spinOnce();
    delayMicroseconds(5);
    return;
  }
  rpm_step();
  if(controlMode == 0){
    if(ctrlEn){
      if(ctrlEn == 2){
        cmdThrottle = fmap(rcChannels[1], -1.0, 1.0, -((float)(1500-minThr)/500), ((float)(maxThr-1500)/500));
      }
      else{
        cmdThrottle = 0;
      }
      cmdSteering = rcChannels[0];
      setAcc = desACC;
    }
    else{
      cmdThrottle = 0;
      cmdSteering = 0;
    }
    
  }
  
  if(controlMode == 1){
    if(ctrlEn){
      if(ctrlEn == 2){
        cmdThrottle = fmap(rosThr, -1.0, 1.0, -((float)(1500-minThr)/500), ((float)(maxThr-1500)/500));
      }
      else{
        cmdThrottle = 0;
      }
      cmdSteering = rosSteering;
      setAcc = rosAcc;
      }
    else{
      cmdThrottle = 0;
      cmdSteering = 0;
    }
  }
  pid_step();
  joy_step();
  status_step();
  nodeHandle.spinOnce();
  delayMicroseconds(5);
}
