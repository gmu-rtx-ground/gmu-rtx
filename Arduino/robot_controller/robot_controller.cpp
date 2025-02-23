
#include "Arduino.h"
#include "def.h"
#include "ros_node.h"
#include "remote_rc.h"
#include "Motor.h"
#include "robot_controller.h"
#include "util.h"
#include <avr/wdt.h>

int controlMode = 0;
uint16_t commRate = 0.0;
uint16_t rcRate = 0.0;
// uint16_t comm = 0.0;
uint16_t controlLoop = 0.0;
float cmdThrottle = 0.0 ;
float cmdSteering = 0.0;
float cmdAux = 0.0;
uint16_t prevTime = 0;
uint16_t prevJoy = 0;

void setup() {
  Serial.begin(115200);
  init_rc();
  initServos();
  for(uint8_t i = 0; i < 8; i++){
    update_rc();;
  }
  servoValues[0] = (int) fmap(rcChannels[0], -1.0, 1.0, minStr, maxStr);
  servoValues[1] = 1500;
  servoValues[2] = 1000;
  write_servos();
  delay(1000);
  init_ros_node(); 
  Serial.println("Initialization Complete");
  wdt_enable(WDTO_250MS);
  wdt_reset();
}

uint8_t failsafe(){
  uint8_t f_stat = 0;
  long cftime = micros();
  //check the last message time from ROS
  if ((cftime - lastMsgTime)/10 > commLossTime*50 && ((cftime > lastMsgTime) || rosMsgFlag > 0)){
    f_stat = 1;
    rosThr = 0.0;
    rosSteering = 0.0;
  }
  if(cftime > lastMsgTime){
    rosMsgFlag = 1;
  }

  //check the last message time from RC
  if((cftime - lastRcTime)/10 > rcFailSafeTime*100 && cftime > lastRcTime){
    f_stat = 2;
  }
  if(f_stat > 1){
    servo_failsafe();
  }
  if(f_stat == 1){
    controlMode = 0;
  }
  return f_stat; 

}

void joy_step(){
  joyMsg.header.stamp = nodeHandle.now();
  joyMsg.header.frame_id = "base_link";
  joyMsg.axes_length = 6;
  joyMsg.buttons_length = 1; 
  float joyAxes[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  int32_t joyButtons[1] = {0}; 
  if(rcRate<100){
    for(int i=0; i < 6; i++){
      joyAxes[i] = rcChannels[i];
    }
  }
  joyMsg.axes = joyAxes;
  joyMsg.buttons = joyButtons;
  joyPub.publish(&joyMsg);
}

void loop (){
  wdt_reset();
  uint16_t timeNow = (uint16_t) millis();
  if(timeNow - prevTime <= 10){
    return;
  }
  prevTime = timeNow;
  update_rc();
  int fState = failsafe();
  if(fState > 1){
    servo_failsafe();
    Serial.print("Failsafe Active Error Code: ");
    Serial.println(fState);
    nodeHandle.spinOnce();
    delayMicroseconds(5);
    return;
  }
  if (fState == 1){
    // Serial.print("Failsafe Active Error Code: ");
    // Serial.println(fState);
    controlMode = 0;
  }

  if(controlMode == 0){
    cmdThrottle = fmap(rcChannels[1], -1.0, 1.0, minThr, maxThr);
    cmdSteering = fmap(rcChannels[0], -1.0, 1.0, minSt, maxSt);  
    cmdAux = 1000;  
  }
  
  if(controlMode == 1){
      cmdThrottle = fmap(rosThr, -1.0, 1.0, minThr, maxThr);
      cmdSteering = fmap(rosSteering, -1.0, 1.0, minSt, maxSt);
      cmdAux = fmap(rosAux, -1.0,  1.0, 900, 2100);
      cmdAux = constrain(cmdAux, 900, 2100);
  }

  servoValues[0] = (uint16_t) cmdSteering;
  servoValues[1] = (uint16_t) cmdThrottle;
  servoValues[2] = (uint16_t) cmdAux;
  write_servos();
  if(timeNow - prevJoy >= 20){
    prevJoy = timeNow;
    joy_step();
  }
  joy_step();
  nodeHandle.spinOnce();
  delay(5);
}
