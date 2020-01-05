#ifndef GPS_h
#define GPS_h

#include <mbed.h>


namespace GPS
{
        void init(Serial* gpsBus);

        bool dataAvailable(void);
        void getData(int32_t* outputTable);
        void RegisterHome(int32_t* inputTable);
        void RegisterTarget(int targetNum, int nextTarget, int32_t* inputTable);
        void activateNextTarget();
        void getNav(uint32_t timeNow, int32_t* data);
};

#endif


#define DEG2RAD (float)0.0174532925199433
#define RAD2DEG (float)57.2957795130823
#define ERR32 0x7FFFFFFF
#define MAX_TARGET_NUM 50
#define MAX_MICROS_NO_DATA 3000000U