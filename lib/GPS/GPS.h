#ifndef GPS_h
#define GPS_h

#include <mbed.h>


class GPS
{
    private:
        volatile bool seenFirst;
        volatile uint8_t msgStep;
        volatile uint8_t CHKA;
        volatile uint8_t CHKB;
        volatile uint16_t MSGLength;
        volatile uint16_t MSGIndex;
        volatile uint16_t messageType;
        Serial& _gps;
        volatile uint8_t buffer[40]; //40 bytes max
        volatile uint8_t posBuf[29]; 
        volatile uint8_t velBuf[37];

        int32_t targets[20][3];
        uint8_t TargetIndex;
        void RXCallback(void);
        int16_t atan2(int32_t Dlat, int32_t Dlon);
    public:
        GPS(Serial& gpsBus);
        void init();

        bool dataAvailable(void);
        void getData(int32_t* outputTable);
        void RegisterTarget(int32_t* inputTable);
};

#endif