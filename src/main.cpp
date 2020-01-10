#include <mbed.h>
#include "Servos.h"

Serial pc(A9, A10,9600);  // tx, rx

int main() {

    pc.printf("Hello World!\r\n");
    Servos::init();

    char buffer[50];
    bool hold = false;
    int servoVal=1500;
    while(1) {
        int pos = 0;
        while(pc.readable()){
            buffer[pos++]=pc.getc();
        }
        if(pos>0){
            if(buffer[0]=='h'){
                hold=true;
                continue;
            }

            if(buffer[0]=='d'){
                hold=false;
                continue;
            }
            int chan = atoi(buffer);
            pc.printf("%d\n",chan);
            if(chan>999 && chan<2001){
                servoVal=chan;
                Servos::setChannelMicros(Servos::Channel::throttle,servoVal);
                Servos::setChannelMicros(Servos::Channel::ailLeft,servoVal);
                Servos::setChannelMicros(Servos::Channel::ailRight,servoVal);
                Servos::setChannelMicros(Servos::Channel::Aux,servoVal);
                wait_us(19000);
                Servos::startAllPulses();

            }
        }

        wait_us(19000);

        if(hold)
            Servos::startAllPulses();

    }
}



#define DEG2RAD 0.0174532925199433f
#define RAD2DEG 57.2957795130823f