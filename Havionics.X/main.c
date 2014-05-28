/*
 * Firmware for Aerial Avionics Boad
 * Yashren Reddi
 * Department of Electrical Engineering
 * University of Cape Town
 * June 2013
 */

#include <plib.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "./USB/usb.h"
#include "./USB/usb_function_cdc.h"

#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "usb_config.h"
#include "HardwareProfile.h"

#include "usb_callback.h"
#include "rn131.h"
#include "io.h"
#include "imu.h"
#include "ultrasonic.h"
#include "pwm.h"
#include "spektrumRX.h"
#include "MDD File System/FSIO.h"

/* Processor configuration bits */
#pragma config UPLLEN   = ON                    // USB PLL enabled
#pragma config UPLLIDIV = DIV_2                 // USB PLL input divider
#pragma config FPLLMUL = MUL_20			// PLL multiplier
#pragma config FPLLIDIV = DIV_2			// PLL input divider
#pragma config FPLLODIV = DIV_1			// PLL output divider
#pragma config FWDTEN = OFF 			// Watchdog timer enable bit, done later in software
#pragma config WDTPS = PS1024			// Set watchdog postscaler to bit after 512ms
#pragma config POSCMOD = HS			// High speed oscillator mode
#pragma config FNOSC = PRIPLL			// Use primary oscillator with PLL
#pragma config FPBDIV = DIV_1			// Set peripheral clock divisor to 1
#pragma config ICESEL = ICS_PGx2		// Use PGD and PGC pair 2 for programming
#pragma config BWP = OFF				// Boot flash is not writable during execution
#pragma config DEBUG = OFF

unsigned short int data_pack[17] = {0};

int main(int argc, char** argv) {
    // Disable JTAG to enable corresponding IO pin functionality
    mJTAGPortEnable(0);

    // Configure MCU for maximum performance
    SYSTEMConfigWaitStatesAndPB(GetSystemClock());	// kill wait states
    CheKseg0CacheOn();                                  // Enable cache
    mCheConfigure(CHECON | 0x30);			// Enable pre-fetch module
    mBMXDisableDRMWaitState();                          // Disable RAM wait states

    #if defined(USE_USB)
        InitializeUSB();
    #endif

    IO_setup();
    RN131_setupDMA();
   
    INTEnableSystemMultiVectoredInt();
    INTEnableInterrupts();

    IO_delayms(3000);

    ITG3200_setup();
    ADXL345_setup();
    ULTRASONIC_setup();
    PWM_initialize();
    SPEKTRUMRX_initialize();

    #if defined(SETUP_RADIO_DETECT)
        IO_changeNotificationSetup();
    #endif

    // initialize SD file system
    IO_initialize_FS();

    IO_LED_init();

    float dt = 20e-3;
    UINT32 prev_time = 0;

    while(1){
        #if defined(USE_USB)
            if(USB_BUS_SENSE && (USBGetDeviceState() == DETACHED_STATE))
            {
                USBDeviceAttach();
            }            
            ProcessIO();
            if (count++ > 1000000){
                count = 0;
                
                if (USBUSARTIsTxTrfReady() && RN131_dataAvailable()){
                    putUSBUSART(RN131_getRxDataPointer(), RN131_getRxDataSize());
                    CDCTxService();
                    RN131_setDataAvailable(false);
                    RN131_setRxMsgLenToZero();
                    count = 0;
                }

//                int usblen = sprintf(usb_buffer_temp,"%d,%d,%d,%d,%d,%d,0x%4X\r\n",
//                        SPEKTRUM_getChannel1(),
//                        SPEKTRUM_getChannel2(),
//                        SPEKTRUM_getChannel3(),
//                        SPEKTRUM_getChannel4(),
//                        SPEKTRUM_getChannel5(),
//                        SPEKTRUM_getChannel6(),
//                        SPEKTRUM_getChannel7());
//                putUSBUSART(&usb_buffer_temp[0], usblen);
            }

        #endif

//        if (IO_get_change_LED()){
//            if (inc){
//                if (count2++ > 100)
//                    inc = false;
//            }
//            else{
//                if (count2-- < 0)
//                    inc = true;
//            }
//            float brightness = 6 + 0.54*count2;
//            IO_leds_on(brightness);
//            IO_set_change_LED(false);
//        }

        if (IO_getSendData()){
            IO_setSendData(false);
            ITG3200_readData();
            ADXL345_readData();
            data_pack[0] = ITG3200_getx();
            data_pack[1] = ITG3200_gety();
            data_pack[2] = ITG3200_getz();
            data_pack[3] = ADXL345_getx();
            data_pack[4] = ADXL345_gety();
            data_pack[5] = ADXL345_getz();
            data_pack[6] = ULTRASONIC_getData();
            data_pack[7] = IO_getBatteryVoltage();
            data_pack[8] = SPEKTRUM_getChannel3();;
            data_pack[9] = SPEKTRUM_getChannel5();
            data_pack[10] = SPEKTRUM_getChannel1();
            data_pack[11] = SPEKTRUM_getChannel2();
            data_pack[12] = SPEKTRUM_getChannel0();
            data_pack[13] = SPEKTRUM_getChannel4();;
            data_pack[14] = IO_getRotorSpeed();
            RN131_SendDataPacket(data_pack, 15);

            UINT32 current_time = IO_get_time_ms();
            dt = (current_time - prev_time)*1e-3;
            if (dt > 20e-3)
                dt = 20e-3;

            IMU_correctGyro(dt);
            IMU_propagateState(dt);
            prev_time = current_time;
        }

        if (SPEKTRUM_isPacketComplete()){
            SPEKTRUM_decodePacket();
            // Indicate packet complete to prevent redundant decoding
            SPEKTRUM_setPacketComplete(false);
        }

        if (SPEKTRUM_isAutoMode()){
            IO_setSystemState(OKAY);
            IO_leds_on(50.0);
        }
        else{
            IO_setSystemState(MANUAL);
            IO_setupPWM_default();
        }

        if (IO_getPWMUpdate()){
            IO_setPWMUpdate(false);
            IO_speedController();
        }
//
//        if (RN131_dataAvailable() && RN131_dataMatch()){
//            //RN131_decodeRxPacket();
//            //RN131_logData();
//            RN131_setDataAvailable(false);
//        }
//
        if (IO_getSDFlush()){
            IO_flush_FS();
            IO_setSDFlush(false);
        }

        if (IO_getCloseFiles()){
            IO_terminate_FS();
        }
    }

    IO_terminate_FS();
    return (EXIT_SUCCESS);
}

