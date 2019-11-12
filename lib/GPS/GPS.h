#ifndef GPS_h
#define GPS_h

#include <mbed.h>


namespace GPS
{
    // private:
    //     volatile bool seenFirst;
    //     volatile uint8_t msgStep;
    //     volatile uint8_t CHKA;
    //     volatile uint8_t CHKB;
    //     volatile uint16_t MSGLength;
    //     volatile uint16_t MSGIndex;
    //     volatile uint16_t messageType;
    //     Serial& _gps;
    //     volatile uint8_t buffer[40]; //40 bytes max
    //     volatile uint8_t posBuf[29]; 
    //     volatile uint8_t velBuf[37];

    //     int32_t targets[20][3];
    //     uint8_t TargetIndex;
    //     void RXCallback(void);
    //     int16_t atan2(int32_t Dlat, int32_t Dlon);
    // public:
        void init(Serial& gpsBus);

        bool dataAvailable(void);
        void getData(int32_t* outputTable);
        void RegisterTarget(int32_t* inputTable);
};

#endif


#define DEG2RAD (float)0.0174532925199433
#define RAD2DEG (float)57.2957795130823
#define ERR32 0x7FFFFFFF
#define MAX_TARGET_NUM 50
#define MAX_MICROS_NO_DATA 3000000U