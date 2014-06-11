#ifndef COMMON_H
#define	COMMON_H

#include <plib.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#define uchar unsigned char
#define ushort unsigned short int

#define PB_FREQ (80000000L)
#define I2C1_FREQ (400000)
#define I2C1_BAUD ((PB_FREQ/2/I2C1_FREQ) - 2)

// number of times to try writing config
#define IMU_TRYCONFIG 3

#define HELICOPTER_1

#if defined (HELICOPTER_2)
    #define LV_EZ0
#endif

static inline void wait(unsigned int count){
    int i;
    for (i = 0; i < count; i++);
}

static inline float COM_round(float in){
    return floor(in + 0.5);
}

#endif	/* COMMON_H */
