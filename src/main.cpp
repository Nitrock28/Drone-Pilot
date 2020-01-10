#include <mbed.h>
#include "Servos.h"

Serial pc(A9, A10,115200);  // tx, rx

int main() {

    pc.printf("Hello World!\n");
    Servos::init();
    pc.printf("init\n");

    char buffer[50];
    bool hold = false;
    int servoVal=1500;
    bool msgOk=false;
    int pos = 0;
    while(1) {
        // can only read 1 character at the time...

        if(pc.readable()){
            buffer[pos]=pc.getc();
            if(buffer[pos]=='.'){
                buffer[pos]='\0';
                msgOk=true;
                pos = 0;
            }
            else
                pos++;
        }
        if(msgOk){
            msgOk=false;
            if(buffer[0]=='h'){
                pc.printf("hold\n"); 
                hold=true;
                continue;
            }

            if(buffer[0]=='d'){
                pc.printf("no hold\n"); 
                hold=false;
                continue;
            }
            int chan = atoi(buffer);
            
            if(chan>500 && chan<2500){
                servoVal=chan;
                pc.printf("%d\n",servoVal);
                Servos::setChannelMicros(Servos::Channel::throttle,servoVal);
                Servos::setChannelMicros(Servos::Channel::ailLeft,servoVal);
                Servos::setChannelMicros(Servos::Channel::ailRight,servoVal);
                Servos::setChannelMicros(Servos::Channel::Aux,servoVal);
                wait_us(19000);
                Servos::startAllPulses();
            }
        }

        wait_us(5000);

        if(hold)
            Servos::startAllPulses();

    }
}



#define DEG2RAD 0.0174532925199433f
#define RAD2DEG 57.2957795130823f