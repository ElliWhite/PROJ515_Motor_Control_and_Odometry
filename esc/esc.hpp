#ifndef __esc_H__    //Inclusion safeguards
#define __esc_H__    //Definition of inclusion

/*============================================================================*/

/* Libraries */
#include "mbed.h"

/* Definitions */
#define lMotorSpeedControlPin PA_5
#define rMotorSpeedControlPin PA_4

#define lMotorDirectionPin PF_13    // D7
#define rMotorDirectionPin PE_9     // D6

#define lMotorEnablePin PE_14                                                                                                                              
#define rMotorEnablePin PE_15

#define ENABLE 0
#define DISABLE 1
#define LEFT_MOTOR_FORWARDS 0
#define LEFT_MOTOR_BACKWARDS 1
#define RIGHT_MOTOR_FORWARDS 1
#define RIGHT_MOTOR_BACKWARDS 0

#define STATIONARY_BRAKES_ON 5
#define STATIONARY 4
#define START_BRAKING 3
#define BRAKING     2
#define START_NOT_BRAKING 1
#define NOT_BRAKING 0


/* Class for single ESC */
class ESC {
    public:
        ESC(PinName speedControlPin, PinName directionPin, PinName enablePin);
        void setDirection(char direction);
        void setEnable(char enable);
        void setSpeed(double speed);
        Timer braking_timer;            // Timer to keep track of how much time has 
                                        // passed before certain relays can be turned on
        Timer stationary_braking_timer; // Timer to keep track of how much time has 
                                        // passed before certain relays can be turned on
                                        // when the robot is meant to be stationary
        char braking_state;
    
    private:
        AnalogOut _speedControl;
        DigitalOut _direction;
        DigitalOut _enable;
        
};


/*============================================================================*/
#endif    