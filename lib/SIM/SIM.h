#ifndef GPS_h
#define GPS_h

#include <mbed.h>


namespace SIM
{
    void init(Serial& simBus);

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