#include "as5600.h"

AS5600::AS5600(PinName sda, PinName scl) : i2c(sda,scl){
       
    //device address is 7-bit. 8th-bit is read/write bit.
    //read = 1, write = 0
    devAddress=0x36;
    i2c.frequency(400000);
    degPerPip = 0.087890625;
}

char AS5600::read(char address){
    
    char returnVal;
    //the device address is shifted by 1 as we need the read/write bit 
    //after the 7-bit address. This creates a read/write bit equal to 0.
    //Mbed's read and write also forces last bit to relevant value
    i2c.write((devAddress << 1), &address, 1);
    i2c.read((devAddress << 1), &returnVal, 1);
    return returnVal;
}

int AS5600::isMagnetPresent(){
    
    char addrStatusReg = 0x0B;
    char data = 0;
    char value = 0;
    data=this->read(addrStatusReg);
    value=(char) (data & 0x20) >> 5;
    //value = (char)(data & 0x38) >> 3;     //here for looking at MD, ML & MH values
    return value;
}

int AS5600::getAngleAbsolute(){
    
    char addrAngleLower=0x0E;
    char addrAngleUpper=0x0F;
    int data=0;
 
    data=this->read(addrAngleLower) << 8;
    data=data+this->read(addrAngleUpper);
 
    return data;
}

void AS5600::setZero(){
    
    relativeZeroPos=0;
    //get initial position of encoder by reading 100 times. Allows sensor to stabalise
    for(char i = 0; i < 100; i++){
        relativeZeroPos=this->getAngleAbsolute();
    }
}

int AS5600::getAngleRelative(){
    
    int relativePos = (this->getAngleAbsolute() - 2047) + (2047 - relativeZeroPos);
    return relativePos;
    
    //return ((this->getAngleAbsolute() + (2047 - relativeZeroPos)) % 4096) - 2047;
}

float AS5600::getAngleAbsoluteDegrees(){
    
    return (this->getAngleAbsolute() * degPerPip);
    
}
