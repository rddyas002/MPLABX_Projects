
#ifndef RN131_H
#define	RN131_H

#include <plib.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include "HardwareProfile.h"
#include "GenericTypeDefs.h"

#ifdef RN131_H_IMPORT
	#define RN131_EXTERN
#else
	#define RN131_EXTERN extern
#endif

#define RN131_UART_PIN_RX       TRISFbits.TRISF2
#define RN131_UART_PIN_TX       TRISFbits.TRISF8
#define RN131_UART              UART1
#define RN131_DisableIntUARTTX  DisableIntU1TX
#define RN131_BAUD_RATE         (500000)
#define RN131_BUFFER_SIZE       256
#define RN131_DATA_PERIOD       (20)    // millisecond
#define RN131_DATA_PERIOD_DELTA (50000)

#define RN131_DMA_TX_PRIORITY DMA_CHN_PRI3
#define RN131_DMA_RX_PRIORITY DMA_CHN_PRI1
#define RN131_DMA_TX_CHANNEL DMA_CHANNEL3
#define RN131_DMA_RX_CHANNEL DMA_CHANNEL4

#define RN131_ENTER_COMMAND "$$$"
#define RN131_EXIT "exit\r"
#define RN131_SHOW_CONNECTION "show connection\r"
#define RN131_SSID_REDDI "set wlan ssid REDDI_Y\r"
#define RN131_PASS_REDDI "set wlan phr 3268df8c168c2534\r"
#define RN131_SSID_YashDroid "YashDroid\r"
#define RN131_PASS_YashDroid "reddi206502029\r"
#define RN131_SAVE "save\r"
#define RN131_REBOOT "reboot\r"
#define RN131_EVERYTHING "get everything\r"

typedef struct{
    char tx_buffer[RN131_BUFFER_SIZE];
    BYTE tx_index;
    BYTE tx_msglen;
    char rx_buffer[RN131_BUFFER_SIZE];
    char rx_buffered[RN131_BUFFER_SIZE];
    BYTE rx_msglen;
    BYTE rxd_msglen;
    BYTE rx_index;
    bool DMA_transmit_active;

    // wifi settings
    bool DMA_MODE;
}RN131_data_struct;

RN131_EXTERN void RN131_setupDMA(void);
RN131_EXTERN bool RN131_dataAvailable(void);
RN131_EXTERN void RN131_decodeRxPacket(void);
RN131_EXTERN void RN131_setDataAvailable(bool val);
RN131_EXTERN char * RN131_getRxDataPointer(void);
RN131_EXTERN BYTE RN131_getRxDataSize(void);
RN131_EXTERN void RN131_setRxIndexToZero(void);
RN131_EXTERN void RN131_DMATransmit(void);
RN131_EXTERN void RN131_addToBuffer(char * data, BYTE len);
RN131_EXTERN void RN131_write2wifi(const char * data);
RN131_EXTERN void RN131_writeBuffer(char * data, BYTE len);
RN131_EXTERN void RN131_setRxMsgLenToZero(void);
RN131_EXTERN void RN131_SendDataPacket(unsigned short int data[], unsigned char data_length);
RN131_EXTERN void RN131_logData(void);
RN131_EXTERN void RN131_logSendTime(void);
RN131_EXTERN UINT32 RN131_getSendTime(void);
RN131_EXTERN UINT32 RN131_getLastRxTime(void);
RN131_EXTERN bool RN131_dataMatch(void);
RN131_EXTERN void RN131_logDataWireless(float a1, float a2, float a3);

// RN131 specific commands
RN131_EXTERN void RN131_enterCmdMode(void);
RN131_EXTERN void RN131_exitCmdMode(void);
RN131_EXTERN void RN131_showConnection(void);
RN131_EXTERN void RN131_setSSID(void);
RN131_EXTERN void RN131_setPhrase(void);
RN131_EXTERN void RN131_save(void);
RN131_EXTERN void RN131_reboot(void);
RN131_EXTERN void RN131_getEverything(void);
#endif	/* RN131_H */

