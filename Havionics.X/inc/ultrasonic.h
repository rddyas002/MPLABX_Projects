#ifndef ULTRASONIC_H
#define	ULTRASONIC_H

#ifdef ULTRASONIC_H_IMPORT
    #define ULTRASONIC_EXTERN
#else
	#define ULTRASONIC_EXTERN extern
#endif

#include "../inc/common.h"

#define ULTRASONIC_DMA_RX_CHANNEL   DMA_CHANNEL1
#define ULTRASONIC_BAUD             9600
#define ULTRASONIC_BUFFER_SIZE      10

ULTRASONIC_EXTERN void ULTRASONIC_setup(void);
ULTRASONIC_EXTERN unsigned short int ULTRASONIC_getData(void);
ULTRASONIC_EXTERN volatile float ULTRASONIC_getHeave(void);
ULTRASONIC_EXTERN bool ULTRASONIC_isUpdated(void);
ULTRASONIC_EXTERN bool ULTRASONIC_setUpdatedFalse(void);

#endif	/* ULTRASONIC_H */