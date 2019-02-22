#ifndef __as5600_H__    //Inclusion safeguards
#define __as5600_H__    //Definition of inclusion

/*============================================================================*/

/* Libraries */
#include "mbed.h"

/* Class for AS5600 Encoder */ 
class AS5600 {
    public:
        AS5600(PinName sda, PinName scl);
        int getAngleAbsolute();
        int getAngleRelative();
        int isMagnetPresent();
        float getAngleDegrees();
        float getAngleAbsoluteDegrees();
        float getAngleMinMax(float angleMax);
        void init();
        void setZero();
        
        float degPerPip;
    
    private:
        char read(char address);
        I2C i2c;
        unsigned char devAddress;
        int relativeZeroPos;
};

/*============================================================================*/
#endif              //End of inclusion
 