#include <mbed.h>
#include "Servos.h"

Serial pc(A9, A10,9600);  // tx, rx

int main() {

    pc.printf("Hello World!\r\n");
    Servos::init(&pc);
    // PortIn servoPort = PortIn(PortName::PortA,0b1111);
    // pc.printf("b\r\n");
    // servoPort.mode(PinMode::PullNone);
    // pc.printf("c\r\n");

    while(1) {

        pc.printf("-- %d --\r\n",Servos::test());
        // pc.printf("-- %d --\r\n",servoPort.read());
        wait_ms(500);
    }
}



#define DEG2RAD 0.0174532925199433f
#define RAD2DEG 57.2957795130823f