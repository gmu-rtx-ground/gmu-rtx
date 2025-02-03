#include "remote_rc.h"
#include "util.h"


static uint8_t PCInt_RX_Pins[PCINT_PIN_COUNT] = {PCINT_RX_BITS};
volatile uint16_t rcValue[PCINT_PIN_COUNT];
unsigned long lastRcTime = 0;
float rcChannels[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 
float rcAcc = 0;
int ctrlEn = 0;

#define RX_PIN_CHECK(pin_pos, rc_value_pos) \
  if (mask & PCInt_RX_Pins[pin_pos]) {      \                      
    if (!(pin & PCInt_RX_Pins[pin_pos])) {  \
      if (((cTime-edgeTime[pin_pos]) > 500 )&&((cTime-edgeTime[pin_pos]) < 2500 )){ \                     
        rcValue[rc_value_pos] = cTime-edgeTime[pin_pos];  \
        lastRcTime = millis(); \
      } \                            
    } else edgeTime[pin_pos] = cTime; \                           
   }



void init_rc(){
  DDRK = 0;
  for(uint8_t i = 0; i < PCINT_PIN_COUNT; i++){
      PCINT_RX_PORT |= PCInt_RX_Pins[i];
      PCINT_RX_MASK |= PCInt_RX_Pins[i];
    }
  PCICR = PCIR_PORT_BIT;
}

void update_rc(){
  for(uint8_t i=0; i < 6; i++){
    rcChannels[i] = fmap(rcValue[i], 1000, 2000, -1.0, 1.0);
  }
  
  if((rcValue[2] > 1500) && (commRate < 3000)) {
    controlMode = 1;
  }
  
  else{
    controlMode = 0;
  }
  

  if(rcValue[5]>1800){
    ctrlEn = 2;
  }
  else{
    if(rcValue[5]<1300){
      ctrlEn = 0;
    }
    else{
      ctrlEn = 1;
    }
  }


  int thrCap = (int) fmap(rcValue[3], 1900, 1100, 0, maxThrCap);
  thrCap = clip(thrCap, 0, maxThrCap);
  minThr = minTh + thrCap;
  maxThr = maxTh - thrCap;
  offsetSteering = (int) fmap(rcValue[4], 1000, 2000, -150, 150);
}


ISR(RX_PC_INTERRUPT) {
    uint8_t mask;
    uint8_t pin;
    uint16_t cTime,dTime;
    static uint16_t edgeTime[8];
    static uint8_t PCintLast;
  
    pin = RX_PCINT_PIN_PORT; // RX_PCINT_PIN_PORT indicates the state of each PIN for the arduino port dealing with Ports digital pins
   
    mask = pin^PCintLast;   // doing a ^ between the current interruption and the last one indicates wich pin changed
    cTime = micros();         // micros() return a uint32_t, but it is not usefull to keep the whole bits => we keep only 16 bits
    sei();                    // re enable other interrupts at this point, the rest of this interrupt is not so time critical and can be interrupted safely
    PCintLast = pin;          // we memorize the current state of all PINs [D0-D7]
  
    #if (PCINT_PIN_COUNT > 0)
      RX_PIN_CHECK(0,0);
    #endif
    #if (PCINT_PIN_COUNT > 1)
      RX_PIN_CHECK(1,1);
    #endif
    #if (PCINT_PIN_COUNT > 2)
      RX_PIN_CHECK(2,2);
    #endif
    #if (PCINT_PIN_COUNT > 3)
      RX_PIN_CHECK(3,3);
    #endif
    #if (PCINT_PIN_COUNT > 4)
      RX_PIN_CHECK(4,4);
    #endif
    #if (PCINT_PIN_COUNT > 5)
      RX_PIN_CHECK(5,5);
    #endif
    #if (PCINT_PIN_COUNT > 6)
      RX_PIN_CHECK(6,6);
    #endif
    #if (PCINT_PIN_COUNT > 7)
      RX_PIN_CHECK(7,7);
    #endif
}