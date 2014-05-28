#ifndef SPEKTRUMRX_H
#define	SPEKTRUMRX_H

#ifdef SPEKTRUMRX_H_IMPORT
    #define SPEKTRUMRX_EXTERN
#else
    #define SPEKTRUMRX_EXTERN extern
#endif

#include <plib.h>
#include <stdbool.h>
#include <string.h>
#include "HardwareProfile.h"
#include "GenericTypeDefs.h"
#include "../inc/common.h"

// UART 5
#define SPEKTRUM_UART               UART3
#define SPEKTRUMRX_UART_RX_TRIS     TRISGbits.TRISG7
#define SPEKTRUM_BAUDRATE           115200
#define SPEKTRUM_INT_PRIORITY       INT_PRIORITY_LEVEL_6
#define SPEKTRUM_INT_SUBPRIORITY    INT_SUB_PRIORITY_LEVEL_3

#define SPEKTRUM_TIMEOUT (500)  // 1000ms

#define SPEKTRUM_CH0 (0b00000000)
#define SPEKTRUM_CH1 (0b00000100)
#define SPEKTRUM_CH2 (0b00001000)
#define SPEKTRUM_CH3 (0b00001100)
#define SPEKTRUM_CH4 (0b00010000)
#define SPEKTRUM_CH5 (0b00010100)
#define SPEKTRUM_CH6 (0b00011000)

#define SPEKTRUM_RIGHT_NOM  (1522)
#define SPEKTRUM_LEFT_NOM   (1484)
#define SPEKTRUM_REAR_NOM   (1484)
#define SPEKTRUM_RUDDER_NOM (1266)
#define SPEKTRUM_ESC_NOM    (940)
#define SPEKTRUM_GAIN_NOM   (1405)

// Mapping of SPEKTRUM input to actual pulse width
#define SPEKTRUM_m0 (1.1689)
#define SPEKTRUM_c0 (912.6475)
#define SPEKTRUM2PULSEWIDTH(x) ((UINT16) (SPEKTRUM_m0*(float)x + SPEKTRUM_c0))

typedef struct{
    char buffer[20];
    char index;
    bool packet_complete;
    UINT16 ms_timer;
}SPEKTRUM_data_struct;

typedef struct{
    short int esc;
    short int servo_right;
    short int servo_rear;
    short int servo_rudder;
    short int gain;
    short int servo_left;
}SPEKTRUM_channel_struct;

SPEKTRUMRX_EXTERN void SPEKTRUMRX_initialize(void);
SPEKTRUMRX_EXTERN void SPEKTRUM_decodePacket(void);
SPEKTRUMRX_EXTERN bool SPEKTRUM_isPacketComplete(void);
SPEKTRUMRX_EXTERN void SPEKTRUM_setPacketComplete(bool val);
SPEKTRUMRX_EXTERN bool SPEKTRUM_isAutoMode(void);
SPEKTRUMRX_EXTERN bool SPEKTRUM_timeoutInc(void);
SPEKTRUMRX_EXTERN void SPEKTRUM_setAutoMode(bool val);
SPEKTRUMRX_EXTERN int SPEKTRUM_getTimeout(void);
SPEKTRUMRX_EXTERN UINT16 SPEKTRUM_getChannel0(void);
SPEKTRUMRX_EXTERN UINT16 SPEKTRUM_getChannel1(void);
SPEKTRUMRX_EXTERN UINT16 SPEKTRUM_getChannel2(void);
SPEKTRUMRX_EXTERN UINT16 SPEKTRUM_getChannel3(void);
SPEKTRUMRX_EXTERN UINT16 SPEKTRUM_getChannel4(void);
SPEKTRUMRX_EXTERN UINT16 SPEKTRUM_getChannel5(void);
SPEKTRUMRX_EXTERN UINT16 SPEKTRUM_getChannel6(void);
SPEKTRUMRX_EXTERN UINT16 SPEKTRUM_getESC(void);
SPEKTRUMRX_EXTERN UINT16 SPEKTRUM_getRIGHT(void);
SPEKTRUMRX_EXTERN UINT16 SPEKTRUM_getREAR(void);
SPEKTRUMRX_EXTERN UINT16 SPEKTRUM_getRUDDER(void);
SPEKTRUMRX_EXTERN UINT16 SPEKTRUM_getGAIN(void);
SPEKTRUMRX_EXTERN UINT16 SPEKTRUM_getLEFT(void);
SPEKTRUMRX_EXTERN UINT16 SPEKTRUM_findYawNominal(void);
SPEKTRUMRX_EXTERN UINT16 SPEKTRUM_getYawNominal(void);
#endif	/* SPEKTRUMRX_H */
