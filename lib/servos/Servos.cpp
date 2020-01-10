
#include <mbed.h>
#include "Servos.h"
// Servos pins are fixed, we create the structures in advance



namespace Servos{


    namespace{
        //private variables

        // mask for all servo pins here using pins B3,B4,B5,B6
        int pinsMask = 0b1111000;
        PortOut servoPins = PortOut(PortName::PortB,pinsMask);
        Timeout timeouts[4];
        unsigned int chanMicros[4];
        mbed::Callback<void ()> callbacks[4];
        

        //private functions

        //callbacks to set each pin down
        void fall0() { servoPins = servoPins &~0b0001000;}
        void fall1() { servoPins = servoPins.read()&~0b0010000;}
        void fall2() { servoPins = servoPins.read()&~0b0100000;}
        void fall3() { servoPins = servoPins.read()&~0b1000000;}

        
    }

    //public functions
    
    
    void init(){
        for (int i=0;i<4;i++){
            chanMicros[i]=1500;
        }
        callbacks[0]=callback(&fall0);
        callbacks[1]=callback(&fall1);
        callbacks[2]=callback(&fall2);
        callbacks[3]=callback(&fall3);
    }

    void setChannelMicros(Channel chan,unsigned int micros){
        chanMicros[chan]=micros;
    }

    /// set all servo pins up, and attaches interrupts to pull pins low.
    void startAllPulses(){

        // set all servos high
        servoPins = pinsMask;

        //attach all servos to pull them down.
        for (int i=0;i<4;i++){
            timeouts[i].attach_us(callbacks[i], chanMicros[i]);
        }
    }
    
}

