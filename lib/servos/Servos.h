#ifndef SERVOS_h
#define SERVOS_h

#include <mbed.h>


namespace Servos
{
        enum Channel { throttle=0, Aux=1, ailLeft=2, ailRight=3};

        // init the servos connections, servos are connected as follows:
        //  B3 : throttle
        //  B4 : aux
        //  B5 : ailLeft
        //  B6 : ailRight
        // (servos normally accept 3.3V signals)
        void init();
        
        
        ///sets the desired channel with a pulse width in microseconds typically 
        ///between 1000 and 2000, some servos can use timings outside this range.
        void setChannelMicros(Channel chan,unsigned int micros);

        /// set all servo pins up, and attaches interrupts to pull pins low.
        /// This will effectively send a command to the servos
        void startAllPulses();
};


#endif