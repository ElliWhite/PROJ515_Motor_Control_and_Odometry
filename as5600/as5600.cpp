/*============================================================================*/

/* Libraries */
#include "as5600.hpp"

/*============================= CLASS CONSTRUCTOR ============================*/

/* Class Constructor */
AS5600::AS5600(PinName sda, PinName scl) : i2c(sda,scl){
       
    //Device address is 7-bit. 8th-bit is read/write bit.
    //Read = 1, write = 0
    devAddress=0x36;
    i2c.frequency(100000);
    degPerPip = 0.087890625;
}

/*================================= FUNCTIONS =================================*/

/*===================================*/
/* Function to read from I2C address */
/*===================================*/
char AS5600::read(char address){
    
    char returnVal;
    //The device address is shifted by 1 as we need the read/write bit 
    //after the 7-bit address. This creates a read/write bit equal to 0.
    //Mbed's read and write also forces last bit to relevant value
    i2c.write((devAddress << 1), &address, 1);
    i2c.read((devAddress << 1), &returnVal, 1);
    return returnVal;
}

/*================================================================*/
/* Function to find if a magnet is present infront of the encoder */
/*================================================================*/
int AS5600::isMagnetPresent(){
    
    char addrStatusReg = 0x0B;              //Address of status register
    char data = 0;
    char value = 0;
    data=this->read(addrStatusReg);         //Read from status register
    value=(char) (data & 0x20) >> 5;        //AND with 0x20 to supress irrelavent bits
    //value = (char)(data & 0x38) >> 3;     //Variant for looking at MD, ML & MH values
    return value;
}

/*==================================================*/
/* Function to get the absolute angle of the magnet */
/*==================================================*/
int AS5600::getAngleAbsolute(){
    
    char addrAngleLower=0x0E;               //Address of angle register bits 0-7 (LOWER)
    char addrAngleUpper=0x0F;               //Address of angle register bits 8-11(UPPER)
    int data=0;
 
    data=this->read(addrAngleLower) << 8;   //Read from lower register and shift left by 8
    data=data+this->read(addrAngleUpper);   //Read from upper register and add to previous
                                            //value
    return data;
}

/*====================================================*/
/* Function to set the initial relative zero position */
/*====================================================*/
void AS5600::setZero(){
    
    relativeZeroPos=0;
    //get initial position of encoder by reading 100 times. Allows sensor to stabalise.
    for(char i = 0; i < 100; i++){
        relativeZeroPos=this->getAngleAbsolute();
    }
}

/*========================================================*/
/* Function to get the angle relative to initial position */
/*========================================================*/
int AS5600::getAngleRelative(){
    
    return ((this->getAngleAbsolute() + (2047 - relativeZeroPos)) % 4096) - 2047;
}

/*===============================================*/
/* Function to get the absolute angle in degrees */
/*===============================================*/
float AS5600::getAngleAbsoluteDegrees(){
    
    return (this->getAngleAbsolute() * degPerPip);
    
}

/*============================================================================*/

/*============================================================================*/