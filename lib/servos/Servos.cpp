
#include <mbed.h>
#include "Servos.h"

namespace Servos{

    namespace{
        //private variables
        PortIn *servoPort;
        
        //private functions
        
    }

    //public functions
    
    void init(Serial *ser){
        ser->printf("a\r\n");
        servoPort = new PortIn(PortName::PortA,0b1111);
        servoPort->mode(PinMode::PullNone);
        ser->printf("%d\r\n",servoPort->read());
        
    }


    int test(){
        return servoPort->read();
        // return GPIOA->IDR;
    }
    
}

