#define RN131_H_IMPORT

#include "../inc/rn131.h"
#include "../inc/io.h"

RN131_data_struct RN131_data;
RN131_states_struct RN131_states;

DmaChannel RN131_DMA_TX_CHN = RN131_DMA_TX_CHANNEL;
DmaChannel RN131_DMA_RX_CHN = RN131_DMA_RX_CHANNEL;

void RN131_setupUART(void);
void RN131_setupDMA_TX(void);
void RN131_setupDMA_RX(void);

void RN131_setupDMA(void){
    memset(&RN131_data, 0, sizeof(RN131_data_struct));
    memset(&RN131_states, 0, sizeof(RN131_states_struct));
    RN131_data.DMA_MODE = true;

    RN131_setupUART();

    UARTEnable(RN131_UART, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));
    INTClearFlag(INT_SOURCE_UART_TX(RN131_UART));
    INTClearFlag(INT_SOURCE_UART_RX(RN131_UART));

    RN131_setupDMA_TX();
    RN131_setupDMA_RX();

    // Enable the chn
    DmaChnEnable(RN131_DMA_RX_CHN);
    // Enable DMA interrupt
    INTEnable(INT_SOURCE_DMA(RN131_DMA_RX_CHN), INT_ENABLED);
}

void RN131_setupUART(void){
    // Set UART IO direction
    RN131_UART_PIN_RX = 1;
    RN131_UART_PIN_TX = 0;

    // Configure UART1 to use flow control
    //UARTConfigure(RN131_UART, UART_ENABLE_PINS_CTS_RTS | UART_RTS_WHEN_RX_NOT_FULL | UART_ENABLE_HIGH_SPEED);
    UARTConfigure(RN131_UART, UART_ENABLE_HIGH_SPEED);
    // Generate an interrupt when TX buffer is empty or when there is data to pick up from the rx buffer
    UARTSetFifoMode(RN131_UART, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    // Set line parameters
    UARTSetLineControl(RN131_UART, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    // Set data rate
    UARTSetDataRate(RN131_UART, GetPeripheralClock(), RN131_BAUD_RATE);
}

void RN131_setupDMA_TX(void){
    // setup DMA channel for transmitting data
    DmaChnOpen(RN131_DMA_TX_CHN, RN131_DMA_TX_PRIORITY, DMA_OPEN_DEFAULT);
    // Set the events: set UART1 tx interrupt to begin transfer
    // Stop transfer when block complete: block complete when patternMatch
    DmaChnSetEventControl(RN131_DMA_TX_CHN, DMA_EV_START_IRQ_EN|DMA_EV_START_IRQ(_UART1_TX_IRQ));
    // Set the transfer source and dest addresses, source and dest sizes and the cell size
    DmaChnSetTxfer(RN131_DMA_TX_CHN, (void*)&(RN131_data.tx_buffer[0]), (void*)&U1TXREG, RN131_data.tx_msglen, 1, 1);
    // Enable the transfer done interrupt
    DmaChnSetEvEnableFlags(RN131_DMA_TX_CHN, DMA_EV_BLOCK_DONE);
    // Set DMA3 interrupt priority
    INTSetVectorPriority(INT_VECTOR_DMA(RN131_DMA_TX_CHN), INT_PRIORITY_LEVEL_5);
    // Set DMA3 interrupt sub-priority
    INTSetVectorSubPriority(INT_VECTOR_DMA(RN131_DMA_TX_CHN), INT_SUB_PRIORITY_LEVEL_3);
}

void RN131_setupDMA_RX(void){
    // Use a DMA channel for picking up data from wifi
    DmaChnOpen(RN131_DMA_RX_CHN, RN131_DMA_RX_PRIORITY, DMA_OPEN_AUTO);
    DmaChnSetMatchPattern(RN131_DMA_RX_CHN, '\n');	// interrupt when \n is encountered
    // Set the events: set UART1 rx interrupt to begin transfer
    // Stop transfer when block complete: block complete when patternMatch
    DmaChnSetEventControl(RN131_DMA_RX_CHN, DMA_EV_START_IRQ_EN|DMA_EV_MATCH_EN|DMA_EV_START_IRQ(_UART1_RX_IRQ));
    // Set the transfer source and dest addresses, source and dest sizes and the cell size
    DmaChnSetTxfer(RN131_DMA_RX_CHN, (void*)&U1RXREG, (void*)&(RN131_data.rx_buffer[0]), 1, RN131_BUFFER_SIZE, 1);
    // Enable the transfer done interrupt
    DmaChnSetEvEnableFlags(RN131_DMA_RX_CHN, DMA_EV_BLOCK_DONE);
    // Set DMA4 interrupt priority
    INTSetVectorPriority(INT_VECTOR_DMA(RN131_DMA_RX_CHN), INT_PRIORITY_LEVEL_5);
    // Set DMA4 interrupt sub-priority
    INTSetVectorSubPriority(INT_VECTOR_DMA(RN131_DMA_RX_CHN), INT_SUB_PRIORITY_LEVEL_0);
}

/* Return & set function */
char * RN131_getRxDataPointer(void){
    return &RN131_data.rx_buffered[0];
}

BYTE RN131_getRxDataSize(void){
    return RN131_data.rx_msglen;
}

void RN131_setRxIndexToZero(void){
    RN131_data.rx_index = 0;
}

void RN131_setRxMsgLenToZero(void){
    RN131_data.rx_msglen = 0;
}

void RN131_DMATransmit(void){
    if (!RN131_data.DMA_transmit_active){
        mU1TXClearIntFlag();
        RN131_data.DMA_transmit_active = true;
        DmaChnSetTxfer(RN131_DMA_TX_CHN, (void*)&(RN131_data.tx_buffer[0]), (void*)&U1TXREG, RN131_data.tx_msglen, 1, 1);
        // Make sure normal tx interrupts are disabled
        DisableIntU1TX;
        // Enable DMA interrupt
        INTEnable(INT_SOURCE_DMA(RN131_DMA_TX_CHN), INT_ENABLED);
        // Enable the channel
        DmaChnEnable(RN131_DMA_TX_CHN);
    }
}

void RN131_write2wifi(const char * data){
    BYTE len = strlen(data);
    memcpy(&RN131_data.tx_buffer[0],data,len);
    RN131_data.tx_msglen = len;
    RN131_DMATransmit();
}

void RN131_logDataWireless(float a1, float a2, float a3){
    char buffer[128];
    sprintf(&buffer[0],"%5.3f,%5.3f,%5.3f\r\n", a1, a2, a3);
    RN131_write2wifi(&buffer[0]);
}

void RN131_writeBuffer(char * data, BYTE len){
    memcpy(&RN131_data.tx_buffer[0],data,len);
    RN131_data.tx_msglen = len;
    RN131_DMATransmit();
}

void RN131_addToBuffer(char * data, BYTE len){
    memcpy(&(RN131_data.tx_buffer[RN131_data.tx_index]),data,len);
    RN131_data.tx_msglen += len;
    if (!RN131_data.DMA_transmit_active){
        RN131_data.DMA_transmit_active = true;
        // if tx buffer is empty, uart_tx will interrupt
        EnableIntU1TX;
    }
}

void RN131_SendDataPacket(unsigned short int data[], unsigned char data_length){
    unsigned char MSB, LSB, i;
    unsigned char checksum = 0;
    
    if (!RN131_data.DMA_transmit_active){
        // expected data content
        //data[1:3] gyroscope
        //data[4:6] accelerometer
        //data[7:8] ultrasonic, voltage
        //data[9:12] servo_left, servo_right, servo_back, servo_tail_ref
        //data[13] ESC input
        //data[14] speed measurement

        RN131_data.tx_index = 0;
        RN131_data.tx_msglen = 0;

        // start sequence
        RN131_data.tx_buffer[RN131_data.tx_msglen++] = '*';
        RN131_data.tx_buffer[RN131_data.tx_msglen++] = '#';

        for (i = 0; i < data_length; i++){
            LSB = (data[i] & 0xFF);
            MSB = (data[i] >> 8);
            checksum ^= LSB;
            checksum ^= MSB;
            RN131_data.tx_buffer[RN131_data.tx_msglen++] = MSB;
            RN131_data.tx_buffer[RN131_data.tx_msglen++] = LSB;
        }
        RN131_data.tx_buffer[RN131_data.tx_msglen++] = checksum;

        // end sequence
        RN131_data.tx_buffer[RN131_data.tx_msglen++] = '@';
        RN131_data.tx_buffer[RN131_data.tx_msglen++] = '!';

        RN131_DMATransmit();
        RN131_data.send_time = IO_get_time_ms();
    }
}

void RN131_decodeData(void){
    int i = 0;
    UINT16 checksum_calc = 0x00;
    char checksum_lsb = 0, checksum_msb = 0;
    UINT16 checksum_sent = 0x0000;
    char data_bin[64] = {0};
    char data_bin_index = 0;

    if ((RN131_data.rxd_msglen != 0) &&
            (RN131_data.rx_buffered[0] == '*') &&
            (RN131_data.rx_buffered[1] == '#') &&
            (RN131_data.rx_buffered[66] == '\r')){
        // Compute checksum
        for (i = 0; i < 61; i++){
            checksum_calc ^= (UINT16)((unsigned char)RN131_data.rx_buffered[i] << 8) | (RN131_data.rx_buffered[i+1]);
        }
        
        // isolate received checksum
        RN131_ASCIIHex2Byte(&RN131_data.rx_buffered[62], &checksum_lsb);
        RN131_ASCIIHex2Byte(&RN131_data.rx_buffered[64], &checksum_msb);
        checksum_sent = (UINT16)(((UCHAR)checksum_msb << 8) | checksum_lsb);
        if (checksum_calc == checksum_sent){
            // convert ASCII hex data to binary
            for (i = 2; i < 61; i+=2){
                RN131_ASCIIHex2Byte(&RN131_data.rx_buffered[i], &data_bin[data_bin_index++]);
            }

            if (data_bin_index == 30)
                RN131_decodeBinary(&data_bin[0]);
                IO_DEBUG_TOGGLE();
        }

    }
}

void RN131_decodeBinary(const char * rx_data){
    memcpy(&RN131_states.translation_x, (rx_data), 2);
    memcpy(&RN131_states.translation_y, (rx_data + 2), 2);
    memcpy(&RN131_states.translation_z, (rx_data + 4), 2);

    memcpy(&RN131_states.velocity_x, (rx_data + 6), 2);
    memcpy(&RN131_states.velocity_y, (rx_data + 8), 2);
    memcpy(&RN131_states.velocity_z, (rx_data + 10), 2);

    memcpy(&RN131_states.quaternion_0, (rx_data + 12), 4);
    memcpy(&RN131_states.quaternion_1, (rx_data + 16), 4);
    memcpy(&RN131_states.quaternion_2, (rx_data + 20), 4);
    memcpy(&RN131_states.quaternion_3, (rx_data + 24), 4);

    memcpy(&RN131_states.sequence, (rx_data + 28), 2);

    // We log time only if data is valid
    RN131_states.timestamp = IO_get_time_ms();
}

bool RN131_timeoutInc(void){
    if (RN131_data.timeout_rx++ > RN131_RX_TIMEOUT)
        return true;
    else
        return false;
}

bool RN131_getTimeout(void){
    if (RN131_data.timeout_rx > RN131_RX_TIMEOUT)
        return true;
    else
        return false;
}

void RN131_clearTimeout(void){
    RN131_data.timeout_rx = 0;
}

void RN131_logData(void){
    static UINT16 last_seq_logged = 0x0000;
    char rn131_buffer[150];

    if (last_seq_logged >=  RN131_states.sequence)
        return;

    int rn_sd = sprintf(&rn131_buffer[0],"%d,%d,%d,%d,%d,%d,%.4f,%.4f,%.4f,%.4f,%d,%lu\r\n",
            RN131_states.translation_x, RN131_states.translation_y, RN131_states.translation_z,
            RN131_states.velocity_x, RN131_states.velocity_y, RN131_states.velocity_z,
            RN131_states.quaternion_0, RN131_states.quaternion_1, RN131_states.quaternion_2, RN131_states.quaternion_3,
            RN131_states.sequence,
            RN131_states.timestamp);

    IO_logMessage(&rn131_buffer[0], rn_sd);
    
    last_seq_logged = RN131_states.sequence;
}

bool RN131_ASCIIHex2Nibble(const char * hex, UCHAR * nibble){
    switch (*hex){
        case '0':
            *nibble = 0;
            break;
        case '1':
            *nibble = 1;
            break;
        case '2':
            *nibble = 2;
            break;
        case '3':
            *nibble = 3;
            break;
        case '4':
            *nibble = 4;
            break;
        case '5':
            *nibble = 5;
            break;
        case '6':
            *nibble = 6;
            break;
        case '7':
            *nibble = 7;
            break;
        case '8':
            *nibble = 8;
            break;
        case '9':
            *nibble = 9;
            break;
        case 'A':
        case 'a':
            *nibble = 10;
            break;
        case 'B':
        case 'b':
            *nibble = 11;
            break;
        case 'C':
        case 'c':
            *nibble = 12;
            break;
        case 'D':
        case 'd':
            *nibble = 13;
            break;
        case 'E':
        case 'e':
            *nibble = 14;
            break;
        case 'F':
        case 'f':
            *nibble = 15;
            break;
        default:
            return false;
            break;
    }
    return true;
}

bool RN131_ASCIIHex2Byte(const char * hex, UCHAR * byte){
    UCHAR msn = 0x00, lsn = 0x00;

    if (!RN131_ASCIIHex2Nibble(hex, &msn))
        return false;
    if (!RN131_ASCIIHex2Nibble(hex+1, &lsn))
        return false;

    *byte = ((UCHAR)msn << 4) | lsn;
    return true;
}

float RN131_get_tx(void){
    return (float)RN131_states.translation_x/1000;
}

float RN131_get_ty(void){
    return (float)RN131_states.translation_y/1000;
}

float RN131_get_tz(void){
    return (float)RN131_states.translation_z/1000;
}

short int RN131_getTx(void){
    return RN131_states.translation_x;
}

short int RN131_getTy(void){
    return RN131_states.translation_y;
}

short int RN131_getTz(void){
    return RN131_states.translation_z;
}

float RN131_getQuaternion_q0(void){
    return RN131_states.quaternion_0;
}

float RN131_getQuaternion_q1(void){
    return RN131_states.quaternion_1;
}

float RN131_getQuaternion_q2(void){
    return RN131_states.quaternion_2;
}

float RN131_getQuaternion_q3(void){
    return RN131_states.quaternion_3;
}

/**************************************************************/
/* RN131 specific commands */
void RN131_enterCmdMode(void){
    RN131_write2wifi(RN131_ENTER_COMMAND);
    // wait half a second
    IO_delayms(500);
}

void RN131_showConnection(void){
    RN131_write2wifi(RN131_SHOW_CONNECTION);
    IO_delayms(100);
}

void RN131_exitCmdMode(void){
    RN131_write2wifi(RN131_EXIT);
    IO_delayms(100);
}

void RN131_setSSID(void){
    RN131_write2wifi(RN131_SSID_REDDI);
    IO_delayms(100);
}

void RN131_setPhrase(void){
    RN131_write2wifi(RN131_PASS_REDDI);
    IO_delayms(100);
}

void RN131_save(void){
    RN131_write2wifi(RN131_SAVE);
    IO_delayms(100);
}

void RN131_reboot(void){
    RN131_write2wifi(RN131_REBOOT);
    IO_delayms(100);
}

void RN131_getEverything(void){
    RN131_write2wifi(RN131_EVERYTHING);
    IO_delayms(100);
}

int RN131_strlen(char data[], int max_len, char match){
    int i = 0;

    for (i = 0; i < max_len; i++){
        if (data[i] == match)
            return (i + 1);
    }

    return -1;
}

/**************************************************************/

// Handler for wifi data tx using DMA channel
void __ISR(_DMA3_VECTOR, ipl5) wifiTxDmaHandler(void)
{
    int	evFlags;
    // Clear interrupt flag
    INTClearFlag(INT_SOURCE_DMA(RN131_DMA_TX_CHN));

    // Get event flags
    evFlags = DmaChnGetEvFlags(RN131_DMA_TX_CHN);

    // If interrupt due to a block event (termination) then proceed
    if(evFlags & DMA_EV_BLOCK_DONE)
    {
        DmaChnClrEvFlags(RN131_DMA_TX_CHN, DMA_EV_BLOCK_DONE);
	// Disable DMA channel
        INTEnable(INT_SOURCE_DMA(RN131_DMA_TX_CHN), INT_DISABLED);
	DmaChnDisable(RN131_DMA_TX_CHN);
        RN131_data.DMA_transmit_active = false;
    }
}

// Handler for wifi data rx using DMA channel
void __ISR(_DMA_4_VECTOR, ipl5) wifiRxDmaHandler(void)
{
    // This routine runs when a complete block has been received from the wifi module
    // The termination byte is '\n'
    int evFlags;

    // Clear interrupt flag
    INTClearFlag(INT_SOURCE_DMA(RN131_DMA_RX_CHN));
    // Get event flags
    evFlags = DmaChnGetEvFlags(RN131_DMA_RX_CHN);

    // If interrupt due to a block event (termination) then proceed
    if(evFlags & DMA_EV_BLOCK_DONE){
        // expect 68 bytes
        RN131_data.rx_msglen = strlen(&RN131_data.rx_buffer[0]);
        RN131_data.rx_buffer[RN131_data.rx_msglen] = 0;

        memcpy(&RN131_data.rx_buffered[0],&RN131_data.rx_buffer[0], RN131_data.rx_msglen);
        RN131_data.rxd_msglen = RN131_data.rx_msglen;

        RN131_data.receive_time = IO_get_time_ms();
        // Clear timeout since data has arrived
        RN131_data.timeout_rx = 0;
                
        RN131_decodeData();
        
	DmaChnClrEvFlags(RN131_DMA_RX_CHN, DMA_EV_BLOCK_DONE);
        // Clear UART1Rx interrupt flag
	INTClearFlag(INT_SOURCE_UART_RX(RN131_UART));
    }
}