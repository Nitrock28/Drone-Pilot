
#include <mbed.h>
#include "GPS.h"

namespace GPS{

    namespace{
        volatile bool seenFirst;
        volatile unsigned int msgStep;
        volatile uint8_t CHKA;
        volatile uint8_t CHKB;
        volatile unsigned int MSGLength;
        volatile unsigned int MSGIndex;
        volatile uint16_t messageType;
        Serial* _ser;
        volatile uint8_t buffer[40]; //40 bytes max
        volatile uint8_t posBuf[29]; 
        volatile uint8_t velBuf[37];

        int32_t targets[MAX_TARGET_NUM][4];
        unsigned int TargetIndex;
        unsigned int TotalTargets;
        int32_t neededHeading;
        int32_t legDist;
        int32_t slopeInv;
        uint32_t lastTimedata;
        

        void RXCallback(void){
            uint8_t byteIn = _ser->getc();
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
                MSGLength = byteIn;
                return;
            }
            if (msgStep==4){
                msgStep = 5;
                CHKA+=byteIn;
                CHKB+=CHKA;
                MSGLength |= ((unsigned int)byteIn)<<8;
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

        // compute planisphere heading and distance,(no curvature of earth used, good for short distance not too close from poles)
        // from an origin and a target on earth. the result is output in the ditHead table in mm and  1e-5 degrees [-18000000;18000000]
        void distAndHeading(int32_t currentLat,int32_t currentLon,int32_t targetLat,int32_t targetLon, int32_t* distHead){
            float dlat = (float)(targetLat-currentLat);
            float dlon = (float)(targetLon-currentLon);
            
            float meanLat = ((float)(currentLat+targetLat))*0.5f;
            float dlonscaled = dlon*cos(meanLat*DEG2RAD*1.0e-7f);// using scaling at origin
            float angleF = atan2f(dlonscaled,dlat)*1e5*RAD2DEG;// in 1e-5 degrees [-18000000;18000000]
            float distF = sqrt(dlonscaled*dlonscaled+dlat*dlat)*DEG2RAD*6.372795e2f; // scale by earth radius and degree-7 conversion -> mm

            distHead[0]=(int32_t)distF;
            distHead[1]=(int32_t)angleF;
        }
    }

    void init(Serial* gpsBus){
        _ser = gpsBus;
        TargetIndex=1;
        TotalTargets=1;

        for(int i = 0;i<MAX_TARGET_NUM;i++){
            targets[i][0]=ERR32; // set as not registered.
        }
        velBuf[36] = 0;
        posBuf[28] = 0;
        _ser->baud(9600);
        _ser->format(8,mbed::SerialBase::None, 1);//8 bits par send, no parity check, 1 stop bit.

        // delete all NMEA messages
        _ser->printf("$PUBX,40,GBS,0,0,0,0,0,0*4D\r\n");
        _ser->printf("$PUBX,40,VTG,0,0,0,0,0,0*5E\r\n");
        _ser->printf("$PUBX,40,GLL,0,0,0,0,0,0*5C\r\n");
        _ser->printf("$PUBX,40,GSA,0,0,0,0,0,0*4E\r\n");
        _ser->printf("$PUBX,40,GSV,0,0,0,0,0,0*59\r\n");
        _ser->printf("$PUBX,40,GGA,0,0,0,0,0,0*5A\r\n");
        _ser->printf("$PUBX,40,RMC,0,0,0,0,0,0*47\r\n");

        //setup 1Hz UBX binary messages for position and speed.
        uint8_t setup1[]={0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x13, 0xBE};
        for (int i = 0; i<16;i++){
            _ser->putc(setup1[i]);
        }
        wait(0.001);
        uint8_t setup2[]={0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x12, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x23, 0x2E};
        for (int i = 0; i<16;i++){
            _ser->putc(setup2[i]);
        }
        msgStep=0;
        seenFirst=0;
        _ser->attach(callback(&RXCallback),mbed::SerialBase::RxIrq);
    }

    

    bool dataAvailable(){
        //data is available if the two buffers are full.
        return (velBuf[36]!=0 && posBuf[28]!=0);
    }
    //  returns a int table with following values filled in :
    //      current horizontal speed in cm/s
    //      current vertical downward speed in cm/second signed
    //      current heading in e-5 degrees from true north
    //      required heading in e-5 degrees from true north (target)
    //      Distance to target in mm
    //      current altitude above sea level in mm
    //      Target altitude above sea level in mm
    // data is sent regardless of the current GPS precision
    void getData(int32_t* outpuTable){
        
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
        outpuTable[4]=(int32_t)distF; // distance from target in mm
        outpuTable[5]=*((int32_t*)(posBuf+16));// altitude msl : bytes 16 to 19 in mm
        outpuTable[6]=targets[TargetIndex][2];
        posBuf[28]=0;
        velBuf[36]=0;
    }

    
    void RegisterHome(int32_t* inputTable){
        targets[0][0]=inputTable[0];//lattitude in 1e-7 deg
        targets[0][1]=inputTable[1];//longitue in 1e-7 deg
        targets[0][2]=inputTable[2];//altitude above sea level in mm
        targets[0][3]=0;// keep at home once RTH is activated.

        int32_t distHeading[2];
        distAndHeading(targets[0][0],targets[0][1],targets[1][0],targets[1][1],distHeading);
        neededHeading = distHeading[1];
    }

    // Add a target to the list in given position.
    // targetNum must be in 1-(MAX_TARGET_NUM-1) or negative for default add at the end of the list.
    // nextTarget : -1 -> next target in the list(or home if end of list). 0-(MAX_TARGET_NUM-1) selected target. -2 : landing at target.
    // the table must contain : 
    //      lattitude in 1e-7 degrees
    //      longitude in 1e-7 degrees
    //      altitude msl of target in mm.
    void RegisterTarget(int targetNum, int nextTarget, int32_t* inputTable){
        if (targetNum<0)
            targetNum=TotalTargets;

        if (targetNum<MAX_TARGET_NUM){
            targets[targetNum][0]=inputTable[0];//lattitude in 1e-7 deg
            targets[targetNum][1]=inputTable[1];//longitue in 1e-7 deg
            targets[targetNum][2]=inputTable[2];//altitude above sea level in mm
            targets[targetNum][3] = nextTarget;//next target : -1 -> next target in the list. 0-(MAX_TARGET_NUM-1) selected target. -2 : landing at target.
            if(targetNum>=TotalTargets)
                TotalTargets=targetNum+1;
        }
    }


    // 
    void activateNextTarget(){
        unsigned int oldTarget = TargetIndex;
        if(TargetIndex!=0){
            int32_t next = targets[TargetIndex][3];
            if(next==-1){
                TargetIndex++;
                while (TargetIndex<MAX_TARGET_NUM && targets[TargetIndex][0]== ERR32);
                    TargetIndex++;
                if (TargetIndex==MAX_TARGET_NUM)
                    TargetIndex=0; // RTH, end of targets reached.
            }
            else if(next>=0 && next<MAX_TARGET_NUM){
                TargetIndex = next;
                if(targets[TargetIndex][0]== ERR32)
                    TargetIndex=0; // RTH if next target not set up
            }
            // all other cases (landing) keep the current target as is.
        }
        if (oldTarget != TargetIndex && TargetIndex!= 0){
            int32_t distHeading[2];
            distAndHeading(targets[oldTarget][0],targets[oldTarget][1],targets[TargetIndex][0],targets[TargetIndex][1],distHeading);
            neededHeading = distHeading[1];
            if(targets[TargetIndex][2]==targets[oldTarget][2])
                slopeInv= 0;
            else
                legDist = distHeading[0];
                slopeInv= distHeading[0]/(targets[TargetIndex][2]-targets[oldTarget][2]);
        }
    }



    // external call to obtain relevant data of next target.
    // data is in the following order:
    //      current lattitude in e-7 degrees.
    //      current longitude in e-7 degrees.
    //      current msl altitude in mm.
    //      current horizontal speed in cm/s
    //      current vertical downward speed in cm/second signed
    //      current heading in 1e-5 degrees, [0-36000000](from GPS)
    //      Distance to target in mm
    //      current course error in e-5 degrees, difference between course and required course. 0 means direction is ok, positive means we need to turn left (course too far right)
    //      current altitude error in mm, positive means we need to climb, negative means to go down
    //      mode : -1- Failsafe, no GPS available keep craft in stable condition until fix is back.
    //             0 - standard mode, position and altitudes are tracked, the path between points is followed as closely as possible.
    //             1 - RTH, only the destination set as home is used, no further target will be used after reaching home.  
    //             2 - landing mode. the target is tracked, the altitude error is always 0, 
    void getNav(uint32_t timeNow, int32_t* data){
        if (dataAvailable()){
            
            // uint32_t hAcc =*((uint32_t*)(posBuf+20));//horizontal accuracy estimate mm
            // uint32_t vAcc =*((uint32_t*)(posBuf+24));//vertical accuracy estimate mm
            // uint32_t sAcc =*((uint32_t*)(velBuf+28));//speed accuracy estimate cm/s
            // uint32_t cAcc =*((uint32_t*)(velBuf+32));//course accuracy estimate 1e-5 deg


            data[0] =*((int32_t*)(posBuf+8));// latitude : bytes 8 to 11 in e-7 degrees
            data[1] =*((int32_t*)(posBuf+4));// longitude : bytes 4 to 7 in e-7 degrees
            data[2] =*((int32_t*)(posBuf+16));// altitude msl : bytes 16 to 19 in mm
            data[3] =*((int32_t*)(velBuf+20));// horizontal: speed bytes 20 to 24 in cm/s
            data[4] =*((int32_t*)(velBuf+12));// downspeed: bytes 12 to 24 in cm/s
            data[5] =*((int32_t*)(velBuf+24)); // heading : bytes 24 to 27 in 1e-5 degrees, [0-36000000]

            //terminate read.
            posBuf[28]=0;
            velBuf[36]=0;

            int32_t distHeading[2];
            distAndHeading(data[0],data[1],targets[TargetIndex][0],targets[TargetIndex][1],distHeading);
            data[6] = distHeading[0];

            if(TargetIndex==0){
                //RTH mode, aim at target.
                data[7]=data[5]-distHeading[1];
                data[9]=1;
            }
            else{
                
                int32_t drift = neededHeading-distHeading[1];
                //TODO replace by mod operation?
                while(drift>17999999)
                    drift-=36000000;
                while(drift<-18000000)
                    drift+=36000000;
                
                if(drift>-6000000 && drift<6000000){
                    //algo : 
                    // wantedDir = targetDir-drift -> to have the wanted direction such that we cut the remaining segment in two (maybe do a bit more than that to comm quicker to the path?)
                    // error= current -wantedDir;
                    data[7]=data[5]+neededHeading-2*distHeading[1];
                }
                else{
                    // drifted too much from trajectory, simple target tracking.
                    data[7]=data[5]-distHeading[1];
                }
                if(targets[TargetIndex][3]==-2)
                    data[9]=2;
                else
                    data[9]=0;

            }

            //TODO replace by mod operation?
            while(data[7]>17999999)
                data[7]-=36000000;
            while(data[7]<-18000000)
                data[7]+=36000000;

            if(distHeading[0]>legDist){
                distHeading[0] = legDist;
            }
            data[8] = (targets[TargetIndex][2]-distHeading[0]/slopeInv)-data[2];
            lastTimedata = timeNow;
        }
        else{
            if((timeNow-lastTimedata)>MAX_MICROS_NO_DATA)
                data[9]=-1; // failsafe mode
        }
        // in case no data is available, do not change anything in the table, this allows this function to be called asynchronously
    }

    
    
}

