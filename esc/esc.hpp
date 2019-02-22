#ifndef __esc_H__    //Inclusion safeguards
#define __esc_H__    //Definition of inclusion

/*============================================================================*/

/* Libraries */
#include "mbed.h"

/* Definitions */
#define lMotorSpeedControlPin PA_4
#define rMotorSpeedControlPin PA_5

#define lMotorDirectionPin PF_13
#define rMotorDirectionPin PE_9

#define lMotorEnablePin PE_14
#define rMotorEnablePin PE_15

#define ENABLE 1
#define DISABLE 0
#define LEFT_MOTOR_FORWARDS 1
#define LEFT_MOTOR_BACKWARDS 0
#define RIGHT_MOTOR_FORWARDS 0
#define RIGHT_MOTOR_BACKWARDS 1


/* Class for single ESC */
class ESC {
    public:
        ESC(PinName speedControlPin, PinName directionPin, PinName enablePin);
        void setDirection(char direction);
        void setEnable(char enable);
        void setSpeed(double speed);
    
    private:
        AnalogOut _speedControl;
        DigitalOut _direction;
        DigitalOut _enable;
        
};


/*============================================================================*/
#endif    