/*============================================================================*/

/* Libraries */
#include "esc.hpp"

/*============================= CLASS CONSTRUCTOR ============================*/

/* Class Constructor */
ESC::ESC(PinName speedControlPin, PinName directionPin, PinName enablePin) :  

    _speedControl(speedControlPin), 
    _direction(directionPin),  
    _enable(enablePin)
    {
        braking_state = NOT_BRAKING;
    }

/*================================= FUNCTIONS ================================*/

/*====================================*/
/* Function to set direction of motor */
/*====================================*/
void ESC::setDirection(char direction){
    
    _direction = direction;   
    
}

/*=====================================*/
/* Function to enable or disable motor */
/*=====================================*/
void ESC::setEnable(char enable){
    
    _enable = enable;
    
}

/*================================*/
/* Function to set speed of motor */
/*================================*/
void ESC::setSpeed(double speed){
    
    _speedControl.write(speed); 

}

/*============================================================================*/

/*============================================================================*/


