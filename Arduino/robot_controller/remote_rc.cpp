#include "remote_rc.h"
#include "util.h"

namespace rc {
  volatile unsigned long lastRcTime = 0;
  volatile uint16_t rcValue[PCINT_PIN_COUNT];
  float rcChannels[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  volatile uint16_t rcData[RC_CHANS];
  volatile uint8_t rcFailsafeCnt = 0;
  int ctrlEn = 0;
  float rcAcc = 0;
  float rcThr;
  float rcSteering;
}

static uint8_t PCInt_RX_Pins[PCINT_PIN_COUNT] = {PCINT_RX_BITS};
volatile uint8_t GoodPulses = 0;


void init_rc(){
  DDRK = 0;
  for(uint8_t i = 0; i < PCINT_PIN_COUNT; i++){ // i think a for loop is ok for the init.
    PCINT_RX_PORT |= PCInt_RX_Pins[i];
    PCINT_RX_MASK |= PCInt_RX_Pins[i];
    rc::rcValue[i] = 1500;
    rc::rcData[i] = 1500;
  }
  PCICR = PCIR_PORT_BIT; 
}

uint16_t readRawRC(uint8_t chan) {
  uint16_t data;
  uint8_t oldSREG;
  oldSREG = SREG; cli();  // Let's disable interrupts
  data = rc::rcValue[chan];   // Let's copy the data Atomically
  SREG = oldSREG;         // Let's restore interrupt state
  return data;            // We return the value correctly copied when the IRQ's where disabled
}

void computeRC() {
  static uint16_t rcData4Values[RC_CHANS][AVERAGING_ARRAY_LENGTH - 1];
  uint16_t rcDataMean, rcDataTmp;
  static uint8_t rc4ValuesIndex = 0;
  uint8_t chan, a;
  uint8_t failsafeGoodCondition = 1;

  rc4ValuesIndex++;
  if (rc4ValuesIndex == AVERAGING_ARRAY_LENGTH - 1)
      rc4ValuesIndex = 0;

  for (chan = 0; chan < RC_CHANS; chan++)
  {
      rcDataTmp = readRawRC(chan);
      failsafeGoodCondition = rcDataTmp > FAILSAFE_DETECT_TRESHOLD || chan > 3; // update controls channel only if pulse is above FAILSAFE_DETECT_TRESHOLD
      if (failsafeGoodCondition)
      {
          rcDataMean = rcDataTmp;
          for (a = 0; a < AVERAGING_ARRAY_LENGTH - 1; a++)
              rcDataMean += rcData4Values[chan][a];
          rcDataMean = (rcDataMean + (AVERAGING_ARRAY_LENGTH / 2)) / AVERAGING_ARRAY_LENGTH;
          if (rcDataMean < (uint16_t)rc::rcData[chan] - 3)
              rc::rcData[chan] = rcDataMean + 2;
          if (rcDataMean > (uint16_t)rc::rcData[chan] + 3)
              rc::rcData[chan] = rcDataMean - 2;
          rcData4Values[chan][rc4ValuesIndex] = rcDataTmp;
      }
  }
}

void update_rc(){
  computeRC();
  for(uint8_t i=0; i < 6; i++){
    rc::rcChannels[i] = 0.5 * fmap(rc::rcData[i], 1000, 2000, -1.0, 1.0) + 0.5*rc::rcChannels[i];
  }
  
  if((rc::rcData[2] > 1500) && (controller::commRate < 3000)) {
    controller::controlMode = 1;
  }
  else{
    controller::controlMode = 0;
  }
}

#define RX_PIN_CHECK(pin_pos, rc_value_pos)                        \
if (mask & PCInt_RX_Pins[pin_pos]) {                             \
  if (!(pin & PCInt_RX_Pins[pin_pos])) {                         \
    if(edgeTime[pin_pos] > cTime){ \
      dTime = cTime + 9999 - edgeTime[pin_pos]; \
    } \
    else{\
      dTime = cTime - edgeTime[pin_pos]; \
    }\
    if (900<dTime && dTime<2200) {                               \
      rc::rcValue[rc_value_pos] = dTime;                             \
      if((rc_value_pos==0 || rc_value_pos==1 ||                   \
          rc_value_pos==2 || rc_value_pos==3)                     \
          && dTime>FAILSAFE_DETECT_TRESHOLD)                     \
            GoodPulses |= (1<<rc_value_pos);                     \
    }                                                            \
  } else edgeTime[pin_pos] = cTime;                              \
}

ISR(RX_PC_INTERRUPT) {
  uint8_t mask;
  uint8_t pin;
  uint16_t cTime,dTime;
  static uint16_t edgeTime[8];
  static uint8_t PCintLast;

  pin = RX_PCINT_PIN_PORT; // RX_PCINT_PIN_PORT indicates the state of each PIN for the arduino port dealing with Ports digital pins
 
  mask = pin ^ PCintLast;   // doing a ^ between the current interruption and the last one indicates wich pin changed
  cTime = TCNT4/2;         // micros() return a uint32_t, but it is not usefull to keep the whole bits => we keep only 16 bits
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
  
  if (GoodPulses == 0x0F) {  // If all main four channels have good pulses, clear FailSafe counter
      GoodPulses = 0;
      if(rc::rcFailsafeCnt > 20){
          rc::rcFailsafeCnt -= 20;
      } else {
          rc::rcFailsafeCnt = 0;
      }
     
  } 
  rc::lastRcTime = micros();
}