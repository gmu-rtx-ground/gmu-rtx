#ifndef MOTOR_H_
#define MOTOR_H_
#endif

#include <Arduino.h>
#include <stdint.h>

#define noServos 3

#ifndef sTimer
    #define sTimer 4
#endif

#if noServos > 0

    #if sTimer == 1
        #define s_DDR DDRB
        #define s_TCCRA TCCR1A
        #define s_TCCRB TCCR1B
        #define s_TCCRC TCCR1C
        #define s_WGM1 WGM11
        #define s_WGM2 WGM12
        #define s_WGM3 WGM13
        #define s_CS0 CS10
        #define s_CS1 CS11
        #define s_SC2 CS12
        #define s_ICR ICR1
        #define s_COMA1 COM1A1
        #define s_COMB1 COM1B1
        #define s_COMC1 COM1C1
        #define s_COMA0 COM1A0
        #define s_COMB0 COM1B0
        #define s_COMC0 COM1C0    
        #ifndef S1
            #define S1 OCR1A
            #define S1_PIN DDB5
        #elif
        #endif
        #ifndef S2
            #define S2 OCR1B
            #define S2_PIN DDB6
        #endif
        #ifndef S3
            #define S3 OCR1C
            #define S3_PIN DDB7
        #endif
    #endif

    #if sTimer == 3
        #define s_DDR DDRE
        #define s_TCCRA TCCR3A
        #define s_TCCRB TCCR3B
        #define s_TCCRC TCCR3C
        #define s_WGM1 WGM31
        #define s_WGM2 WGM32
        #define s_WGM3 WGM33
        #define s_CS0 CS30
        #define s_CS1 CS31
        #define s_SC2 CS32
        #define s_ICR ICR3
        #define s_COMA1 COM3A1
        #define s_COMB1 COM3B1
        #define s_COMC1 COM3C1
        #define s_COMA0 COM3A0
        #define s_COMB0 COM3B0
        #define s_COMC0 COM3C0  
        #ifndef S1
            #define S1 OCR3A
            #define S1_PIN DDE3
        #elif
        #endif
        #ifndef S2
            #define S2 OCR3B
            #define S2_PIN DDE4
        #endif
        #ifndef S3
            #define S3 OCR3C
            #define S3_PIN DDE5
        #endif

    #endif

    #if sTimer == 4
        #define s_DDR DDRH
        #define s_TCCRA TCCR4A
        #define s_TCCRB TCCR4B
        #define s_TCCRC TCCR4C
        #define s_WGM1 WGM41
        #define s_WGM2 WGM42
        #define s_WGM3 WGM43
        #define s_CS0 CS40
        #define s_CS1 CS41
        #define s_SC2 CS42
        #define s_ICR ICR4
        #define s_COMA1 COM4A1
        #define s_COMB1 COM4B1
        #define s_COMC1 COM4C1
        #define s_COMA0 COM4A0
        #define s_COMB0 COM4B0
        #define s_COMC0 COM4C0 
        #ifndef S1
            #define S1 OCR4A
            #define S1_PIN DDH3
        #endif
        #ifndef S2
            #define S2 OCR4B
            #define S2_PIN DDH4
        #endif
        #ifndef S3
            #define S3 OCR4C
            #define S3_PIN DDH5
        #endif
    #endif

    #if sTimer == 5
        #define S_DDR DDRH
        #define s_TCCRA TCCR5A
        #define s_TCCRB TCCR5B
        #define s_TCCRC TCCR5C
        #define s_WGM1 WGM51
        #define s_WGM2 WGM52
        #define s_WGM3 WGM53
        #define s_CS0 CS50
        #define s_CS1 CS51
        #define s_SC2 CS52
        #define s_ICR ICR5
        #define s_COMA1 COM5A1
        #define s_COMB1 COM5B1
        #define s_COMC1 COM5C1
        #define s_COMA0 COM5A0
        #define s_COMB0 COM5B0
        #define s_COMC0 COM5C0 
        #define S_REG DDRL
        #ifndef S1
            #define S1 OCR5A
            #define S1_PIN DDL3
        #elif
        #endif
        #ifndef S2
            #define S2 OCR5B
            #define S2_PIN DDL4
        #endif
        #ifndef S3
            #define S3 OCR5C
            #define S3_PIN DDL5
        #endif
    #endif

#endif

#if noServos > 0
    #define writeS1(value)  \
        cli();              \
        S1 = value * 2;     \
        sei();              

#endif


#if noServos > 1
    #define writeS2(value)  \
        cli();              \
        S2 = value * 2;     \
        sei();              
#endif

#if noServos > 2
    #define writeS3(value)  \
        cli();              \
        S3 = value * 2;     \
        sei();              
#endif

extern uint16_t servoValues[noServos];
void initServos();
void writeServo(uint8_t servo, uint8_t idx);
void write_servos();
void servo_failsafe();
int throttle_PID(int, int);

#ifndef ESC
    #define ESC writeS1
    #define ESC_idx 0
#endif

#ifndef Steering
    #define Steering writeS2
    #define Steering_idx 1
#endif

#ifndef Aux
    #define Aux writeS3
    #define Aux_idx 2
#endif
