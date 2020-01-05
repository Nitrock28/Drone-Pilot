#ifndef SERVOS_h
#define SERVOS_h

#include <mbed.h>


namespace Servos
{
        void init(Serial *ser);
        int test();
};


#endif