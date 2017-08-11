#ifndef __RF_69_DRIVER_INCLUDED__
#define __RF_69_DRIVER_INCLUDED__

#include <RH_RF69.h>

class RF69Driver : public RH_RF69
{
protected:
   
    //Overriding handleInterrupt so that we can set the radio to receive
    //by default rather than idle 

    void handleInterrupt();
    //Only providing one interrupt vector so no multi device setup
    static void isr();

public:

    //Inherited ctor
    RF69Driver(uint8_t slaveSelectPin = SS, uint8_t interruptPin = 2, RHGenericSPI& spi = hardware_spi);

    //Override init to set out new ISR as the interrupt vector
    bool init() override;

    //Keeps the radio NODE_ADDR register updated so we can use address filtering
    void setThisAddress(uint8_t) override;
};

#endif
