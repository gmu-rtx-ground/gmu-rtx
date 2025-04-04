#include <Arduino.h>
#include <avr/wdt.h>
#include "def.h"
#include "Motor.h"
#include "ros_node.h"
#include "remote_rc.h"
#include "robot_controller.h"
#include "util.h"

namespace controller {
    uint8_t controlMode = 0;
    uint16_t controlLoop = 0;
    uint16_t commRate = 0;
    uint16_t rcRate = 0;
    float cmdThrottle = 0.0;
    float cmdSteering = 0.0;
    float cmdAux = 0.0;
}
// uint16_t comm = 0.0;
uint16_t prevTime = 0;
uint16_t prevJoy = 0;

void setup() {
  Serial.begin(115200);
  init_rc();
  initServos();
  for(uint8_t i = 0; i < 8; i++){
    update_rc();;
  }
  Motor::servoValues[0] = (int) fmap(rc::rcChannels[0], -1.0, 1.0, Motor::minStr, Motor::maxStr);
  Motor::servoValues[1] = 1500;
  Motor::servoValues[2] = 1000;
  write_servos();
  delay(1000);
  init_ros_node(); 
  Serial.println("Initialization Complete");
  wdt_enable(WDTO_250MS);
  wdt_reset();
}

uint8_t failsafe(){
  uint8_t f_stat = 0;
  uint32_t cftime = micros();
  //check the last message time from ROS
  if ((cftime - ros_node::lastMsgTime)/10 > commLossTime*50 && ((cftime > ros_node::lastMsgTime) || ros_node::rosMsgFlag > 0)){
    f_stat = 1;
    ros_node::rosThr = 0.0;
    ros_node::rosSteering = 0.0;
  }
  if(cftime > ros_node::lastMsgTime){
    ros_node::rosMsgFlag = 1;
  }

  //check the last message time from RC
  if((cftime - rc::lastRcTime)/10 > rcFailSafeTime*100 && cftime > rc::lastRcTime){
    f_stat = 2;
  }
  if(f_stat > 1){
    servo_failsafe();
  }
  if(f_stat == 1){
    controller::controlMode = 0;
  }
  return f_stat; 

}

void joy_step(){
  ros_node::joyMsg.header.stamp = ros_node::nodeHandle.now();
  ros_node::joyMsg.header.frame_id = "base_link";
  ros_node::joyMsg.axes_length = 6;
  ros_node::joyMsg.buttons_length = 1; 
  float joyAxes[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  int32_t joyButtons[1] = {0}; 
  if(controller::rcRate<100){
    for(int i=0; i < 6; i++){
      joyAxes[i] = rc::rcChannels[i];
    }
  }
  ros_node::joyMsg.axes = joyAxes;
  ros_node::joyMsg.buttons = joyButtons;
  ros_node::joyPub.publish(&ros_node::joyMsg);
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
    ros_node::nodeHandle.spinOnce();
    delayMicroseconds(5);
    return;
  }
  if (fState == 1){
    // Serial.print("Failsafe Active Error Code: ");
    // Serial.println(fState);
    controller::controlMode = 0;
  }

  if(controller::controlMode == 0){
    controller::cmdThrottle = fmap(rc::rcChannels[1], -1.0, 1.0, Motor::minThr, Motor::maxThr);
    controller::cmdSteering = fmap(rc::rcChannels[0], -1.0, 1.0, minSt, maxSt);  
    controller::cmdAux = 1000;  
  }
  
  if(controller::controlMode == 1){
      controller::cmdThrottle = fmap(ros_node::rosThr, -1.0, 1.0, Motor::minThr, Motor::maxThr);
      controller::cmdSteering = fmap(ros_node::rosSteering, -1.0, 1.0, minSt, maxSt);
      controller::cmdAux = fmap(ros_node::rosAux, -1.0,  1.0, 900, 2100);
      controller::cmdAux = constrain(controller::cmdAux, 900, 2100);
  }

  Motor::servoValues[0] = (uint16_t) controller::cmdSteering;
  Motor::servoValues[1] = (uint16_t) controller::cmdThrottle;
  Motor::servoValues[2] = (uint16_t) controller::cmdAux;
  write_servos();
  if(timeNow - prevJoy >= 20){
    prevJoy = timeNow;
    joy_step();
  }
  joy_step();
  ros_node::nodeHandle.spinOnce();
  delay(5);
}
