#include "RF69Driver.h"



RF69Driver::RF69Driver(uint8_t slaveSelectPin, uint8_t interruptPin, RHGenericSPI& spi) 
    : RH_RF69(slaveSelectPin, interruptPin, spi) {};

bool RF69Driver::init()
{
    if(!RH_RF69::init()) return false;

    //Update broadcast address to RH_BROADCAST_ADDRESS
    spiWrite(0x3A, RH_BROADCAST_ADDRESS);

    //Register interrupt w/SPI
    int interruptNum = digitalPinToInterrupt(_interruptPin);
    SPI.usingInterrupt(interruptNum);

    //Change our interrupt vector to call handleInterrupt
    if(_myInterruptIndex != 0) return true;
    detachInterrupt(interruptNum);
    attachInterrupt(interruptNum, isr, RISING);

    return true;
}

void RF69Driver::handleInterrupt()
{
    //Default back to receiving so we dont risk losing messages
    RH_RF69::handleInterrupt();
    RH_RF69::setModeRx();
}

void RF69Driver::isr()
{
    reinterpret_cast<RF69Driver*>(_deviceForInterrupt[0])->handleInterrupt();
}

void RF69Driver::setThisAddress(uint8_t addr)
{
    //Keep node addr register on radio updated
    spiWrite(0x39, addr);
    RH_RF69::setThisAddress(addr);
}
