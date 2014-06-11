#define RN131_H_IMPORT

#include "../inc/rn131.h"
#include "../inc/io.h"

RN131_data_struct RN131_data;
bool wifi_data_available = false;

DmaChannel RN131_DMA_TX_CHN = RN131_DMA_TX_CHANNEL;
DmaChannel RN131_DMA_RX_CHN = RN131_DMA_RX_CHANNEL;

int RN131_tx = 0, RN131_ty = 0, RN131_tz = 0;
int RN131_vx = 0, RN131_vy = 0, RN131_vz = 0;
float RN131_roll = 0, RN131_pitch = 0, RN131_yaw = 0;
float RN131_quaternion[4] = {0,1,0,0};
int RN131_bias_gx = 0, RN131_bias_gy = 0, RN131_bias_gz = 0;
int RN131_bias_ax = 0, RN131_bias_ay = 0, RN131_bias_az = 0;
char RN131_checksum_rx[3] = {0};
char RN131_checksum_tx[3] = {0};
bool RN131_data_match = false;

UINT32 RN131_send_time_ms = 0;
UINT32 RN131_last_rx_time_ms = 0;
UINT32 RN131_latency = 0;
UINT32 RN131_data_timestamp = 0;
volatile UINT32 RN131_timeout_ms_var = 0;

void RN131_setupUART(void);
void RN131_setupDMA_TX(void);
void RN131_setupDMA_RX(void);

void RN131_setupDMA(void){
    wifi_data_available = false;
    memset(&RN131_data,0,sizeof(RN131_data_struct));
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
    INTSetVectorSubPriority(INT_VECTOR_DMA(RN131_DMA_TX_CHN), INT_SUB_PRIORITY_LEVEL_0);
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
bool RN131_dataAvailable(void){
    return wifi_data_available;
}

void RN131_setDataAvailable(bool val){
    wifi_data_available = val;
}

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
        sprintf(&RN131_checksum_tx[0], "%.2X", (unsigned char)checksum);

        // end sequence
        RN131_data.tx_buffer[RN131_data.tx_msglen++] = '@';
        RN131_data.tx_buffer[RN131_data.tx_msglen++] = '!';

        RN131_DMATransmit();
        RN131_logSendTime();
    }
}

void RN131_logSendTime(void){
    RN131_send_time_ms = IO_get_time_ms();
}

UINT32 RN131_getSendTime(void){
    return RN131_send_time_ms;
}

UINT32 RN131_getLastRxTime(void){
    return RN131_last_rx_time_ms;
}

void RN131_decodeRxPacket(void){
    char * temp_char;

    if ((RN131_data.rxd_msglen != 0) &&
            (RN131_data.rx_buffered[0] == '*') &&
            (RN131_data.rx_buffered[1] == '#') &&
            (RN131_data.rx_buffered[32] == '\r')){

        temp_char = strtok(&RN131_data.rx_buffered[2],",");
        RN131_quaternion[0] = atof(temp_char);

        temp_char = strtok(NULL,",");
        RN131_quaternion[1] = atof(temp_char);

        temp_char = strtok(NULL,",");
        RN131_quaternion[2] = atof(temp_char);

        temp_char = strtok(NULL,",");
        RN131_quaternion[3] = atof(temp_char);

//        temp_char = strtok(&RN131_data.rx_buffered[2],",");
//        RN131_tx = atoi(temp_char);
//
//        temp_char = strtok(NULL,",");
//        RN131_ty = atoi(temp_char);
//
//        temp_char = strtok(NULL,",");
//        RN131_tz = atoi(temp_char);

//        temp_char = strtok(NULL,",");
//        RN131_vx = atoi(temp_char);
//
//        temp_char = strtok(NULL,",");
//        RN131_vy = atoi(temp_char);
//
//        temp_char = strtok(NULL,",");
//        RN131_vz = atoi(temp_char);

//        temp_char = strtok(NULL,",");
//        RN131_roll = atof(temp_char);
//
//        temp_char = strtok(NULL,",");
//        RN131_pitch = atof(temp_char);
//
//        temp_char = strtok(NULL,",");
//        RN131_yaw = atof(temp_char);

//        temp_char = strtok(NULL,",");
//        RN131_bias_gx = atoi(temp_char);
//
//        temp_char = strtok(NULL,",");
//        RN131_bias_gy = atoi(temp_char);
//
//        temp_char = strtok(NULL,",");
//        RN131_bias_gz = atoi(temp_char);
//
//        temp_char = strtok(NULL,",");
//        RN131_bias_ax = atoi(temp_char);
//
//        temp_char = strtok(NULL,",");
//        RN131_bias_ay = atoi(temp_char);
//
//        temp_char = strtok(NULL,",");
//        RN131_bias_az = atoi(temp_char);

        temp_char = strtok(NULL,"\r");
        memcpy(&RN131_checksum_rx[0], temp_char, 2);

        if (strcmp(&RN131_checksum_rx[0],"01") == 0){
            RN131_clearTimeout();
        }

        RN131_data_timestamp = IO_get_time_ms();

//        RN131_logData();

        if (strcmp(&RN131_checksum_rx[0], &RN131_checksum_tx[0]) == 0){
            RN131_data_match = true;
            // if received synch data, send data again
        }
        else
            RN131_data_match = false;
    }
}

bool RN131_dataMatch(void){
    return RN131_data_match;
}

bool RN131_timeoutInc(void){
    if (RN131_timeout_ms_var++ > RN131_RX_TIMEOUT)
        return true;
    else
        return false;
}

bool RN131_getTimeout(void){
    if (RN131_timeout_ms_var > RN131_RX_TIMEOUT)
        return true;
    else
        return false;
}

void RN131_clearTimeout(void){
    RN131_timeout_ms_var = 0;
}

void RN131_logData(void){
    char rn131_buffer[150];
//    int rn_sd = sprintf(&rn131_buffer[0],"%d,%d,%d,%d,%d,%d,%.1f,%.1f,%.1f,%d,%d,%d,%d,%d,%d,%lu\r\n",
//        RN131_tx, RN131_ty, RN131_tz,
//        RN131_vx, RN131_vx, RN131_vx,
//        RN131_roll, RN131_pitch, RN131_yaw,
//        RN131_bias_gx, RN131_bias_gy, RN131_bias_gz,
//        RN131_bias_ax, RN131_bias_ay, RN131_bias_az,
//        IO_get_time_ms());

    int rn_sd = sprintf(&rn131_buffer[0],"%.4f,%.4f,%.4f,%.4f,%lu\r\n",
        RN131_quaternion[0], RN131_quaternion[1],
        RN131_quaternion[2], RN131_quaternion[3],
        RN131_data_timestamp);

    IO_logMessage(&rn131_buffer[0], rn_sd);
}

int RN131_get_tz(void){
    return RN131_tz;
}

float * RN131_getQuaternion(void){
    return &RN131_quaternion[0];
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
    int i = 0;
    int evFlags;

    // Clear interrupt flag
    INTClearFlag(INT_SOURCE_DMA(RN131_DMA_RX_CHN));
    // Get event flags
    evFlags = DmaChnGetEvFlags(RN131_DMA_RX_CHN);

    // If interrupt due to a block event (termination) then proceed
    if(evFlags & DMA_EV_BLOCK_DONE)
    {
        RN131_data.rx_msglen = strlen(&RN131_data.rx_buffer[0]);
        RN131_data.rx_buffer[RN131_data.rx_msglen] = 0; // null character at end of packet

        // copy data from primary buffer to secondary buffer where operations can take place
        memcpy(&RN131_data.rx_buffered[0], &RN131_data.rx_buffer[0], RN131_data.rx_msglen);
        RN131_data.rxd_msglen = RN131_data.rx_msglen;

        RN131_last_rx_time_ms = IO_get_time_ms();
        RN131_latency = RN131_last_rx_time_ms - RN131_getSendTime();
        RN131_decodeRxPacket();

        wifi_data_available = true;
        
	DmaChnClrEvFlags(RN131_DMA_RX_CHN, DMA_EV_BLOCK_DONE);
        // Clear UART1Rx interrupt flag
	INTClearFlag(INT_SOURCE_UART_RX(RN131_UART));
    }
}