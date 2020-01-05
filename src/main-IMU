#include <mbed.h>
#include "IMU.h"

Serial pc(A9, A10,9600);  // tx, rx
SPI spi_bus(SPI_MOSI, SPI_MISO, SPI_SCK); // mosi, miso, sclk

DigitalOut led(C13,0);
DigitalOut sel_Baro(B0,1);
DigitalOut sel_MPU(SPI_CS,1);
float data[9];
float angles[3];
uint32_t CLKH = 0, CLKL=0, old=0;

//gets the time in µs since startup, must be called at least every 30 ms. 
//Loops every 1h 11m 35s
unsigned int getTime(){
    CLKL=us_ticker_read();
    if(CLKL<old){
        CLKH+=0x00010000;
    }
    old=CLKL;
    return(CLKH|CLKL);
}

int main() {
    pc.printf("Hello World!\r\n");
    IMU::init(&spi_bus,&sel_MPU,&sel_Baro);
    IMU::calibrateSensor(&led);
    uint32_t Now = getTime();
    uint32_t lastTime = Now;
    uint32_t count = 0;

    while(1) {
        Now = getTime();
        if((Now-lastTime)>5000U){
            led=0;
            IMU::getBMPData(data);
            //IMU::printsyncData(data,&pc);
            // // we must pass -ax, ay, az, gx, -gy, -gz, my, -mx, mz because sensor is not aligned
            IMU::MadgwickAHRSupdate(-data[0],data[1],data[2],data[3],-data[4],-data[5],data[7],-data[6],data[8],Now);
            led=1;
            lastTime = Now;
            count++;
            if(count>198){
                IMU::printsyncData(data,&pc);
                IMU::getEulerAngles(angles);
                pc.printf("Roll Pitch Yaw : %f | %f | %f\r\n",angles[0],angles[1],angles[2]);
                count=0;
            }
        }

    }
}




#define DEG2RAD 0.0174532925199433f
#define RAD2DEG 57.2957795130823f