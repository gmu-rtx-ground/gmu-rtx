#include "Arduino.h"
#include "tasks.h"
#include "util.h"

unsigned long task_last_time[10];
int f_latch = 0;

void init_task(){
  for(int i=0; i<10; i++){
    task_last_time[i] = millis();
  }
}

int failsafe(){
  if(f_latch != 0){
    servo_failsafe();
    return 3;
  }

  int f_stat = 0;
  commRate    = millis() - lastMsgTime;
  rcRate      = millis() - lastRcTime;
  controlLoop = millis() - task_last_time[failsafec_id];

  if (millis() - lastMsgTime > commLossTime){
    f_stat = 1;
    commRate = 10000.0;
  }

  if (millis() - lastRcTime > rcFailSafeTime){
    f_stat = 2;
    rcRate = 10000.0;
  }
  
  if (millis() - task_last_time[failsafec_id] > maxConLoopTime){
    f_stat = 3;
    f_latch = 1;
    controlLoop = 10000.0;
  }
  task_last_time[failsafec_id] = millis();
  
  if(f_stat > 1){
    servo_failsafe();
  } 

  if(f_stat == 1){
    // controlMode = 0;
  }

  return f_stat;
}

void pid_step(){
  if (millis() - task_last_time[pid_id] >= pidLoopTime){
    task_last_time[pid_id] = millis();

    float curRpm = ((float)(rpms[0] + rpms[1])) / (2 * maxRpm); 
    float thrScaled = cmdThrottle;// * (1.0 - ((float)(minTh - minThr))/thrRange);
    float pidOut = compute_pid(curRpm*100, thrScaled, dt_rpm/1000);
    int outThr;
    if(pidOut>0){
      outThr = (int) fmap(pidOut, 0, 100.0, 1538, maxThr);  
    }  
    if(pidOut<0){
      outThr = (int) fmap(pidOut, -100.0, 0, minThr, 1500-38);  
    } 
    if(pidOut == 0){
      outThr = 1500;
    }
    servoValues[1] = get_thr_cmd(outThr, rpms[0] + rpms[1]);
    servoValues[0] = (int) fmap(cmdSteering, -1.0, 1.0, minStr, maxStr);
    
    // Serial.print("in_thr");
    // Serial.print(cmdThrottle*100);
    // Serial.print("\t cur_rpm");
    // Serial.print(curRpm);
    // Serial.print("\t out_thr");
    // Serial.println(pidOut);
     write_servos();
  }
}

void rpm_step(){
  if (millis() - task_last_time[rpm_id] >= pidLoopTime){
    task_last_time[rpm_id] = millis();
    update_rpm();
  }
}

void status_step(){
  if (millis() - task_last_time[status_id] >= statPubTime){
    task_last_time[status_id] = millis();
    if (f_latch == 0){
      update_angle();
      update_voltage(); 
    }
    float rStatPacket[9] = {rpms[0], rpms[1], angles[0], battVoltage[0], battper[0], commRate, rcRate, controlLoop, controlMode};
    robotStatus.data_length = 9;
    robotStatus.data = rStatPacket;
    rStatPub.publish(&robotStatus);

    // Serial.print("Control Mode:");
    // Serial.print(controlMode);
    // Serial.print("\t Throttle:"); 
    // Serial.print(cmdThrottle);
    // Serial.print("\t outThrottle: ");
    // Serial.print(servoValues[1]);
    // Serial.print("\t CurSpeed:");
    // Serial.print(((float)(rpms[0] + rpms[1])) / (2 * maxRpm));
    // Serial.print("\t RPM1: ");
    // Serial.print(rpms[0]);
    // Serial.print("\t RPM2: ");
    // Serial.print(rpms[1]);
    // Serial.print("\t ST Angle: ");
    // Serial.print(angles[0]);
    // Serial.print("\t looptime: ");
    // Serial.print(controlLoop);
    // Serial.print("\t ch_0: ");
    // Serial.print(rcValue[0]);
    // Serial.print("\t ch_1: ");
    // Serial.println(rcValue[1]);
  }
}

void joy_step(){
  if(millis() - task_last_time[joy_id] >= joyPubTime){
    task_last_time[joy_id] = millis();
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
}