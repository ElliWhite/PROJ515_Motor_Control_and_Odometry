#include "mbed.h"
#ifndef as5600_H_
#define as5600_H_
 
 
class AS5600 {
    public:
        AS5600(PinName sda, PinName scl);
        float getAngleDegrees();
        int getAngleAbsolute();
        int getAngleRelative();
        float getAngleAbsoluteDegrees();
        float getAngleMinMax(float angleMax);
        int isMagnetPresent();
        float degPerPip;
        void init();
        void setZero();
    
    private:
        char read(char address);
        I2C i2c;
        unsigned char devAddress;
        int relativeZeroPos;
};


#endif
 