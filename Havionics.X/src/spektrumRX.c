#define SPEKTRUMRX_IMPORT

#include "../inc/spektrumRX.h"
#include "../inc/HardwareProfile.h"
#include "../inc/io.h"

volatile SPEKTRUM_data_struct SPEKTRUM_rawData;
volatile SPEKTRUM_channel_struct SPEKTRUM_channelData;

volatile bool SPEKTRUM_autoMode = false;

volatile int SPEKTRUM_timeout_ms_var = 0;

UINT16 SPEKTRUM_channel0 = 0x00;
UINT16 SPEKTRUM_channel1 = 0x00;
UINT16 SPEKTRUM_channel2 = 0x00;
UINT16 SPEKTRUM_channel3 = 0x00;
UINT16 SPEKTRUM_channel4 = 0x00;
UINT16 SPEKTRUM_channel5 = 0x00;
UINT16 SPEKTRUM_channel6 = 0x00;

UINT16 SPEKTRUM_ESC = 935;
UINT16 SPEKTRUM_RIGHT = 1500;
UINT16 SPEKTRUM_REAR = 1500;
UINT16 SPEKTRUM_RUDDER = 1500;
UINT16 SPEKTRUM_GAIN = 1500;
UINT16 SPEKTRUM_LEFT = 1500;

UINT16 SPEKTRUM_RUDDER_NOMINAL = 0;

void SPEKTRUMRX_initialize(void){

    memset((SPEKTRUM_data_struct *) &SPEKTRUM_rawData, 0, sizeof(SPEKTRUM_data_struct));
    memset((SPEKTRUM_channel_struct *) &SPEKTRUM_channelData, 0, sizeof(SPEKTRUM_channel_struct));

    SPEKTRUMRX_UART_RX_TRIS = INPUT_PIN;

    UARTConfigure(SPEKTRUM_UART, UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(SPEKTRUM_UART, UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(SPEKTRUM_UART, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(SPEKTRUM_UART, GetPeripheralClock(), SPEKTRUM_BAUDRATE);
    UARTEnable(SPEKTRUM_UART, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX));

    INTClearFlag(INT_SOURCE_UART_RX(SPEKTRUM_UART));
    INTClearFlag(INT_SOURCE_UART_TX(SPEKTRUM_UART));
    INTEnable(INT_SOURCE_UART_RX(SPEKTRUM_UART), INT_ENABLED);
    INTEnable(INT_SOURCE_UART_TX(SPEKTRUM_UART), INT_DISABLED);
    INTSetVectorPriority(INT_VECTOR_UART(SPEKTRUM_UART), SPEKTRUM_INT_PRIORITY);
    INTSetVectorSubPriority(INT_VECTOR_UART(SPEKTRUM_UART), SPEKTRUM_INT_SUBPRIORITY);

    SPEKTRUM_findYawNominal();
}

UINT16 SPEKTRUM_findYawNominal(void){
    int i = 0;
    UINT32 SPEKTRUM_RUDDER_SUM = 0;
    // wait for 10 complete packets

    for (i = 0; i < 20; i++){
        while(!SPEKTRUM_isPacketComplete());
        SPEKTRUM_decodePacket();
        SPEKTRUM_RUDDER_SUM += SPEKTRUM_RUDDER;
        // Indicate packet complete to prevent redundant decoding
        SPEKTRUM_setPacketComplete(false);
    }

    SPEKTRUM_RUDDER_NOMINAL = (UINT16)(SPEKTRUM_RUDDER_SUM/20);
    return SPEKTRUM_RUDDER_NOMINAL;
}

UINT16 SPEKTRUM_getYawNominal(void){
    return SPEKTRUM_RUDDER_NOMINAL;
}

void SPEKTRUM_decodePacket(void){
    unsigned short int spektrum_word;

    // rough error checking
    if (SPEKTRUM_rawData.buffer[12] == 0x18){
        if ((SPEKTRUM_rawData.buffer[0] & 0xFC) == SPEKTRUM_CH1)
        {
            spektrum_word = ((unsigned short int)SPEKTRUM_rawData.buffer[0] << 8) & 0x03FF;
            SPEKTRUM_channel1 = (UINT16) (spektrum_word | (uchar)SPEKTRUM_rawData.buffer[1]);
            SPEKTRUM_RIGHT = SPEKTRUM2PULSEWIDTH(SPEKTRUM_channel1);
        }
        if ((SPEKTRUM_rawData.buffer[2] & 0xFC) == SPEKTRUM_CH5)
        {
            spektrum_word = ((unsigned short int)SPEKTRUM_rawData.buffer[2] << 8) & 0x03FF;
            SPEKTRUM_channel5 = (UINT16) (spektrum_word | (uchar)SPEKTRUM_rawData.buffer[3]);
            SPEKTRUM_LEFT = SPEKTRUM2PULSEWIDTH(SPEKTRUM_channel5);
        }
        if ((SPEKTRUM_rawData.buffer[4] & 0xFC) == SPEKTRUM_CH2)
        {
            spektrum_word = ((unsigned short int)SPEKTRUM_rawData.buffer[4] << 8) & 0x03FF;
            SPEKTRUM_channel2 = (UINT16) (spektrum_word | (uchar)SPEKTRUM_rawData.buffer[5]);
            SPEKTRUM_REAR = SPEKTRUM2PULSEWIDTH(SPEKTRUM_channel2);
        }
        if ((SPEKTRUM_rawData.buffer[6] & 0xFC) == SPEKTRUM_CH3)
        {
            spektrum_word = ((unsigned short int)SPEKTRUM_rawData.buffer[6] << 8) & 0x03FF;
            SPEKTRUM_channel3 = (UINT16) (spektrum_word | (uchar)SPEKTRUM_rawData.buffer[7]);
            SPEKTRUM_RUDDER = SPEKTRUM2PULSEWIDTH(SPEKTRUM_channel3);
        }
        if ((SPEKTRUM_rawData.buffer[8] & 0xFC) == SPEKTRUM_CH0)
        {
            spektrum_word = ((unsigned short int)SPEKTRUM_rawData.buffer[8] << 8) & 0x03FF;
            SPEKTRUM_channel0 = (UINT16) (spektrum_word | (uchar)SPEKTRUM_rawData.buffer[9]);
            SPEKTRUM_ESC = SPEKTRUM2PULSEWIDTH(SPEKTRUM_channel0);
        }
        if ((SPEKTRUM_rawData.buffer[10] & 0xFC) == SPEKTRUM_CH4)
        {
            spektrum_word = ((unsigned short int)SPEKTRUM_rawData.buffer[10] << 8) & 0x03FF;
            SPEKTRUM_channel4 = (UINT16) (spektrum_word | (uchar)SPEKTRUM_rawData.buffer[11]);
            SPEKTRUM_GAIN = SPEKTRUM2PULSEWIDTH(SPEKTRUM_channel4);

            if (SPEKTRUM_channel4 > 288)
                SPEKTRUM_autoMode = false;
            else{
                SPEKTRUM_autoMode = true;
            }
        }

        // clear timer to indicate packet received
        SPEKTRUM_timeout_ms_var = 0;

    //    spektrum_word = ((unsigned short int)SPEKTRUM_rawData.buffer[12] << 8);
    //    SPEKTRUM_channel7 = (UINT16) (spektrum_word | (uchar)SPEKTRUM_rawData.buffer[13]);
    }
}

// Indicates whether communication issue with SPEKTRUM module
bool SPEKTRUM_timeoutInc(void){
    if (SPEKTRUM_timeout_ms_var++ > SPEKTRUM_TIMEOUT)
        return true;
    else
        return false;
}

int SPEKTRUM_getTimeout(void){
    return SPEKTRUM_timeout_ms_var;
}

UINT16 SPEKTRUM_getChannel0(void){
    return SPEKTRUM_channel0;
}

UINT16 SPEKTRUM_getChannel1(void){
    return SPEKTRUM_channel1;
}

UINT16 SPEKTRUM_getChannel2(void){
    return SPEKTRUM_channel2;
}

UINT16 SPEKTRUM_getChannel3(void){
    return SPEKTRUM_channel3;
}

UINT16 SPEKTRUM_getChannel4(void){
    return SPEKTRUM_channel4;
}

UINT16 SPEKTRUM_getChannel5(void){
    return SPEKTRUM_channel5;
}

UINT16 SPEKTRUM_getChannel6(void){
    return SPEKTRUM_channel6;
}

UINT16 SPEKTRUM_getESC(void){
    return SPEKTRUM_ESC;
}

UINT16 SPEKTRUM_getRIGHT(void){
    return SPEKTRUM_RIGHT;
}

UINT16 SPEKTRUM_getREAR(void){
    return SPEKTRUM_REAR;
}

UINT16 SPEKTRUM_getRUDDER(void){
    return SPEKTRUM_RUDDER;
}

UINT16 SPEKTRUM_getGAIN(void){
    return SPEKTRUM_GAIN;
}

UINT16 SPEKTRUM_getLEFT(void){
    return SPEKTRUM_LEFT;
}

bool SPEKTRUM_isPacketComplete(void){
    return SPEKTRUM_rawData.packet_complete;
}

void SPEKTRUM_setPacketComplete(bool val){
    SPEKTRUM_rawData.packet_complete = val;
}

bool SPEKTRUM_isAutoMode(void){
    return SPEKTRUM_autoMode;
}

void SPEKTRUM_setAutoMode(bool val){
    SPEKTRUM_autoMode = val;
}

/* 
    Interrupt service routine for the Spektrum Module
*/
void __ISR(_UART_3_VECTOR, ipl6) UART_SPEKTRUM( void)
{
    static char prev_byte = 0, current_byte = 0;

    if (INTGetFlag(INT_SOURCE_UART_RX(SPEKTRUM_UART)))
    {
    	while(U3STAbits.URXDA)
    	{
            current_byte = UARTGetDataByte(SPEKTRUM_UART);

            if ((current_byte == 0x01) && (prev_byte == 0x03)){
                SPEKTRUM_rawData.index = 0;
                SPEKTRUM_rawData.packet_complete = false;
            }
            else
                SPEKTRUM_rawData.buffer[SPEKTRUM_rawData.index++] = current_byte;

            // If complete packet has been received, form words and register data
            if (SPEKTRUM_rawData.index == 14)
            {
                IO_DEBUG_LAT_HIGH();
                SPEKTRUM_decodePacket();
                IO_DEBUG_LAT_LOW();
                SPEKTRUM_rawData.packet_complete = true;
            }

            prev_byte = current_byte;
    	}
	// Clear interrupt flag
	INTClearFlag(INT_SOURCE_UART_RX(SPEKTRUM_UART));
    }
}

#undef SPEKTRUMRX_IMPORT
