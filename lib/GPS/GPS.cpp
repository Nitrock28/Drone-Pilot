#include <mbed.h>
#include "GPS.h"

#define DEG2RAD (float)0.0174532925199433
#define RAD2DEG (float)57.2957795130823
#define ERR32 0x7FFFFFFF

GPS::GPS(Serial& gpsBus): _gps(gpsBus) {}

void GPS::init(){
    TargetIndex=0;
    velBuf[36] = 0;
    posBuf[28] = 0;
    _gps.baud(9600);
    _gps.format(8,mbed::SerialBase::None, 1);//8 bits par send, no parity check, 1 stop bit.

    // delete all NMEA messages
    _gps.printf("$PUBX,40,GBS,0,0,0,0,0,0*4D\r\n");
    _gps.printf("$PUBX,40,VTG,0,0,0,0,0,0*5E\r\n");
    _gps.printf("$PUBX,40,GLL,0,0,0,0,0,0*5C\r\n");
    _gps.printf("$PUBX,40,GSA,0,0,0,0,0,0*4E\r\n");
    _gps.printf("$PUBX,40,GSV,0,0,0,0,0,0*59\r\n");
    _gps.printf("$PUBX,40,GGA,0,0,0,0,0,0*5A\r\n");
    _gps.printf("$PUBX,40,RMC,0,0,0,0,0,0*47\r\n");

    //setup 1Hz UBX binary messages for position and speed.
    uint8_t setup1[]={0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x13, 0xBE};
    for (int i = 0; i<16;i++){
        _gps.putc(setup1[i]);
    }
    wait(0.001);
    uint8_t setup2[]={0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x12, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x23, 0x2E};
    for (int i = 0; i<16;i++){
        _gps.putc(setup2[i]);
    }
    msgStep=0;
    seenFirst=0;
    _gps.attach(callback(this, &GPS::RXCallback),mbed::SerialBase::RxIrq);


}

void GPS::RXCallback(void){
    uint8_t byteIn = _gps.getc();
    if (byteIn==0xB5){
        seenFirst = true;
        if(msgStep ==0)
            return;
        //this byte may just be part of a message, continue analysis
    }
    else if (seenFirst && byteIn==0x62){
        // we have seen the 2 start bytes, read messageType
        msgStep = 1;
        seenFirst = false;
        return;// we are ready to receive next, end here
    }
    else{
        // remove the first byte flag
        seenFirst = false;
    }

    if (msgStep==5){
        //message reading
        CHKA+=byteIn;
        CHKB+=CHKA;
        buffer[MSGIndex]=byteIn;
        MSGIndex++;
        if(MSGIndex>=MSGLength)
            msgStep=6;
        return;
    }
    if (msgStep==1){
        msgStep = 2;
        CHKA=byteIn;
        CHKB=CHKA;
        messageType = ((uint16_t)byteIn)<<8;
        return;
    }
    if (msgStep==2){
        msgStep = 3;
        CHKA+=byteIn;
        CHKB+=CHKA;
        messageType |= (uint16_t)byteIn;
        return;
    }
    if (msgStep==3){
        msgStep = 4;
        CHKA+=byteIn;
        CHKB+=CHKA;
        MSGLength = (uint16_t)byteIn;
        return;
    }
    if (msgStep==4){
        msgStep = 5;
        CHKA+=byteIn;
        CHKB+=CHKA;
        MSGLength |= ((uint16_t)byteIn)<<8;
        MSGIndex=0;
        if(MSGLength >40)
            msgStep=0;//skip message
        return;
    }
    if (msgStep==6){
        //first checksum byte
        msgStep = 7;
        if(CHKA!=byteIn)
            msgStep=0;//skip message
        return;
    }
    if (msgStep==7){
        //second checksum byte
        msgStep=0;//End of message
        if(CHKB==byteIn){

            if(messageType == 0x0102){
                //Position message copy to working table
                for(int i = 0;i<28;i++){
                    posBuf[i]=buffer[i];
                }
                posBuf[28]+=1;//signal new write to processing code.
                return;
            }
            if(messageType == 0x0112){
                //speed message copy to working table
                for(int i = 0;i<36;i++){
                    velBuf[i]=buffer[i];
                }
                velBuf[36]+=1;//signal new write to processing code.
                return;
            }
        }
    }
}

bool GPS::dataAvailable(){
    //data is available if the two buffers are full.
    return (velBuf[36]!=0 && posBuf[28]!=0);
}
//  returns a 16bit signed table with following values filled in :
//      current horizontal speed in cm/s unsigned
//      current vertical downward speed in cm/second signed
//      current heading in e-5 degrees from true north
//      required heading in e-5 degrees from true north (target)
//      Distance to target in mm
//      current altitude above sea level in mm
//      Target altitude above sea level in mm
// data is sent regardless of the current GPS precision
void GPS::getData(int32_t* outpuTable){
    
    int32_t curLat =*((int32_t*)(posBuf+8));// bytes 8 to 11 in e-7 degrees
    int32_t curLon =*((int32_t*)(posBuf+4));// bytes 4 to 7
    float dlat = (float)(targets[TargetIndex][0]-curLat);
    float dlon = (float)(targets[TargetIndex][1]-curLon);
    // compute planisphere heading and distance,(no curvature of earth used, good for short distance not too close from poles)
    float dlonscaled = dlon*cos(((float)curLat)*DEG2RAD*1.0e-7);// using scaling at origin
    float angleF = atan2f(dlonscaled,dlat);// in radians
    float distF = sqrt(dlonscaled*dlonscaled+dlat*dlat)*DEG2RAD*6.372795e2; // scale by earth radius and degree-7 conversion -> mm
    
    
    outpuTable[0]=*((int32_t*)(velBuf+20));//hspeed 20 to 24
    outpuTable[1]=*((int32_t*)(velBuf+12));//downspeed 12 to 24
    outpuTable[2]=*((int32_t*)(velBuf+24)); // heading bytes 24 to 27
    outpuTable[3]=(int32_t)(angleF*1e5*RAD2DEG);// target direction from north in 1e-5 degrees
    if(outpuTable[3]<0)
        outpuTable[3]+=36000000; //assert positive angle as from GPS
    outpuTable[4]=(int32_t)distF;
    outpuTable[5]=*((int32_t*)(posBuf+16));// altitude msl : bytes 16 to 19
    outpuTable[6]=targets[TargetIndex][2];
    posBuf[28]=0;
    velBuf[36]=0;
}


void GPS::RegisterTarget(int32_t* inputTable){
    targets[0][0]=inputTable[0];//lattitude in 1e-7 deg
    targets[0][1]=inputTable[1];//longitue in 1e-7 deg
    targets[0][2]=inputTable[2];//altitude above sea level in mm
    return;
}

// void getCourseAndDistance(double long1, double long2, double lat1, double lat2)
//     double delta = radians(long2-long1);
//     double sdlong = sin(delta);
//     double cdlong = cos(delta);
//     lat1 = radians(lat1);
//     lat2 = radians(lat2);
//     double slat1 = sin(lat1);
//     double clat1 = cos(lat1);
//     double slat2 = sin(lat2);
//     double clat2 = cos(lat2);
//     delta = (clat2 * slat1) - (slat2 * clat1 * cdlong);
//     delta = sq(delta);
//     delta += sq(clat1 * sdlong);
//     delta = sqrt(delta);
//     double denom = (slat2 * slat1) + (clat2 * clat1 * cdlong);
//     delta = atan2(delta, denom);
//     dist =  delta * 6372795;
//     // returns course in degrees (North=0, West=270) from position 1 to position 2,
//     // both specified as signed decimal-degrees latitude and longitude.
//     // Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
//     // Courtesy of Maarten Lamers
//     double dlon = radians(long2-long1);
//     lat1 = radians(lat1);
//     lat2 = radians(lat2);
//     double a1 = sin(dlon) * cos(lat2);
//     double a2 = sin(lat1) * cos(lat2) * cos(dlon);
//     a2 = cos(lat1) * sin(lat2) - a2;
//     a2 = atan2(a1, a2);
//     if (a2 < 0.0)
//     {
//         a2 += TWO_PI;
//     }
//     course =  degrees(a2);
// }