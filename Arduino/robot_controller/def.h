#ifndef DEF_H_
#define DEF_H_

/*******************************************************
                  Pin definitions
*******************************************************/
//LED pins
#define conLed        22
#define modeLed       23
#define batLed        24
#define rcLed         25
#define gpsLed        26
#define camLed        27
#define modeLed       28
#define lidarLed      29

//Sensor Pins
#define quadEncCnt    2
#define magEncCt      1
#define quadEncPins   {{3, 2}, {18, 19}}
#define magEncPins    {16}
#define batPinCnt     1
#define batSenPin     {A0}

//servo pin defination
#define steeringPin   4
#define escPin        5

//Interrupt Pins 
//A8 - A13 --> CH1 - CH6 
#define PCINT_PIN_COUNT            6
#define PCINT_RX_BITS              (1<<0),(1<<1),(1<<2),(1<<3),(1<<4),(1<<5)
#define PCINT_RX_PORT              PORTK
#define PCINT_RX_MASK              PCMSK2
#define PCIR_PORT_BIT              (1<<2)
#define RX_PC_INTERRUPT            PCINT2_vect
#define RX_PCINT_PIN_PORT          PINK


/*******************************************************
                  Limits definitions
*******************************************************/
// Limits for the servo range
#define minSt       1000 
#define maxSt       2000 
#define minTh       1000
#define maxTh       2000
#define maxbattV    16.8
#define minbattV    14.2


/*******************************************************
                  Timing definitions
*******************************************************/
#define brkTime         500
#define rcFailSafeTime  200
#define commLossTime    600
#define maxConLoopTime  100
#define pidLoopTime     20
#define joyPubTime      20
#define statPubTime     20


/*******************************************************
                  Parameter definitions
*******************************************************/
#define encCpr        {15, 15}
#define battscale     17.3
#define f_diff_ratio  0.3
#define tire_dia      218
#define kp            0.8
#define ki            1.5
#define kd            0.8
#define desACC        1.0
#define maxAcc        32.0
#define maxDesc       32.0
#define maxSpeed      22.0


#endif
