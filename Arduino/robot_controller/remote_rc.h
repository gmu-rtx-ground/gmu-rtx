#ifndef REMOTE_RC_H_
#define REMOTE_RC_H_
#endif

#include <Arduino.h>
#include "def.h"
#include "robot_controller.h"

#define maxThrCap 350

#define RC_CHANS 6
#define AVERAGING_ARRAY_LENGTH 4
#define FAILSAFE_DETECT_TRESHOLD  985

//Disabled the channel 4, 5 and 6
#define PCINT_PIN_COUNT            RC_CHANS
#define PCINT_RX_BITS              (1<<0),(1<<1),(1<<2),(0<<3),(0<<4),(0<<5)
#define PCINT_RX_PORT              PORTK
#define PCINT_RX_MASK              PCMSK2
#define PCIR_PORT_BIT              (1<<2)
#define RX_PC_INTERRUPT            PCINT2_vect
#define RX_PCINT_PIN_PORT          PINK

extern volatile unsigned long lastRcTime;
extern volatile uint16_t rcValue[PCINT_PIN_COUNT];
extern float rcChannels[6];

extern volatile uint8_t rcFailsafeCnt;
extern uint16_t rcData[RC_CHANS];

void computeRC();
void configureReceiver();
void init_rc();
void update_rc();