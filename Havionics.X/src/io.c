#define IO_H_IMPORT

#include "../inc/io.h"
#include "../inc/ultrasonic.h"
#include "../inc/rn131.h"
#include "../inc/imu.h"

/*
 * Describe interrupts:
 *      CoreTimer INT PRIORITY:SUBPRIORITY 7:2
 *      Battery ADC interrupt 1:3
 * 
 * Other notes:
 * IO uses timer2 for PWM module
 * IO uses time 1 for delays
 */

IO_file IO_dmesg;
IO_file IO_log;

volatile UINT32 IO_time_ms = 0;
volatile UINT32 IO_DATA_TIME = 0;
IO_SYSTEM_STATE IO_SystemState = BOOT;
IO_SYSTEM_STATE IO_SystemPreviousState = BOOT;
UINT16 IO_batteryVoltage = 0;
volatile bool IO_change_LED = false;
volatile bool IO_sendData = false;
volatile bool IO_localControl = true;
volatile bool IO_stateProject = false;
volatile bool IO_sdFlush = false;
volatile bool IO_closeFiles = false;

void IO_enableLevelShifter1(BOOL val);
void IO_enableLevelShifter2(BOOL val);
void IO_setupPWM(void);
void IO_setupTICK(void);
void IO_setupADC(void);
void IO_delayUnder200ms(unsigned int val);
bool IO_write_FS(IO_file * io_file, char * data, UINT16 len);
void IO_terminate_FS(void);
bool IO_flush_FS(void);

#define CONFIG (CN_ON | CN_IDLE_CON)
//#define PINS (CN3_ENABLE | CN4_ENABLE | CN5_ENABLE | CN6_ENABLE | CN15_ENABLE | CN16_ENABLE | CN19_ENABLE)
#define PINS (CN15_ENABLE | CN16_ENABLE)
#define PULLUPS (CN_PULLUP_DISABLE_ALL)

volatile unsigned int IO_rotor_speed_dt = 0, IO_gyro_gain_dt = 0, IO_servo_left_dt = 0;
volatile unsigned int IO_esc_dt = 0, IO_servo_right_dt = 0, IO_servo_rear_dt = 0, IO_rudder_dt = 0;
volatile float IO_rotor_speed = 0;
volatile bool IO_gyro_gain_update = false, IO_servo_left_update = false;
volatile bool IO_esc_update = false, IO_servo_right_update = false, IO_servo_rear_update = false, IO_rudder_update = false;
volatile unsigned int IO_rotor_speed_update = 0;
volatile bool IO_rotor_speed_timeout = false;
volatile bool IO_PWM_update = false;
volatile bool IO_LED_ON = false;

// **********************************
// Roll channel control parameters
// QFT controller: 1/(1+L(s)) <= 6dB V P
//             ((s/16.35)^2 + 2*0.1*s/16.35 + 1)
// G(s) = 8.06 --------------------------------------
//                      (s/16.35 + 1)^2
// ***********************************
#define IO_ROLL_KP          (8.06)
#define IO_ROLL_UPPER_LIMIT (180)
#define IO_ROLL_LOWER_LIMIT (-180)
#define IO_ROLL_NOTCH_WN    (16.35)
#define IO_ROLL_NOTCH_WZ_DAMP (0.1)
#define IO_ROLL_NOTCH_WP_DAMP (1)

// **********************************
// Pitch channel control parameters
// QFT controller: 1/(1+L(s)) <= 6dB V P
//             ((s/15.747)^2 + 2*0.1558*s/15.747 + 1)
// G(s) = 4.39 --------------------------------------
//                      (s/15.747 + 1)^2
// ***********************************
#define IO_PITCH_KP          (4.39)
#define IO_PITCH_UPPER_LIMIT (180)
#define IO_PITCH_LOWER_LIMIT (-180)
#define IO_PITCH_NOTCH_WN    (15.747)
#define IO_PITCH_NOTCH_WZ_DAMP (0.1558)
#define IO_PITCH_NOTCH_WP_DAMP (1)

// Pitch channel control parameters
#define IO_LONG_MAX          (110)
#define IO_LONG_MIN          (-110)

// **********************************
// Heave channel control parameters
// QFT controller: 1/(1+L(s)) <= 6dB V P
//             (s/0.3 + 1) (s/0.3 + 1)
// G(s) = 6.2 ----------- -----------
//                  s      (s/30 + 1)
// ***********************************
#define IO_HEAVE_KP          (7)//(6.2)
#define IO_HEAVE_OMEGA_LEAD  (0.1)//(0.3)
#define IO_HEAVE_OMEGA_LAG   (30)//(30)
#define IO_HEAVE_OMEGA_TI    (1)//(0.3)
#define IO_HEAVE_UPPER_LIMIT (180)
#define IO_HEAVE_LOWER_LIMIT (0)
#define IO_ALTITUDE_LIMIT    (4)

// **********************************
// Yaw channel control parameters
// QFT controller: 1/(1+L(s)) <= 6dB V P
//             (s/0.5 + 1) (s/0.5 + 1)
// G(s) = 0.14 ----------- -----------
//                  s      (s/30 + 1)
// ***********************************
#define IO_YAW_KP          (150)
#define IO_YAW_OMEGA_LEAD  (2)
#define IO_YAW_OMEGA_LAG   (30)
#define IO_YAW_OMEGA_TI    (2)
#define IO_YAW_UPPER_LIMIT (191)
#define IO_YAW_LOWER_LIMIT (-274)

// Speed control parameters
// Input PWM 0-255, Output speed rad/s
// **********************************
//             (s/2.5 + 1) 
// G(s) = 1.44 -----------      omega_gc = 4.23 rad/s
//                  s     
// Gain reduction due to noise from ESC
// Kp' = 0.7 ---> omega_gc' = 2.2 rad/s

#define IO_SPEED_KP          (0.7)
#define IO_SPEED_OMEGA_TI    (2.5)
/*
 * if integrator upper limit is chosen as x
 * UPPER_LIMIT = x/(KP/OMEGA_Ti)
 */
#define IO_SPEED_UPPER_LIMIT (250)
#define IO_SPEED_LOWER_LIMIT (0)

void IO_setup(void){
    // Setup direction for heartbeat LED
    IO_HEARTBEAT_TRIS();
    // Set PROGRAM switch as digital input
    IO_PROGRAM_SWITCH_TRIS();

    // Setup debug direction
    IO_DEBUG_TRIS();
    IO_DEBUG_LAT_LOW();

    // Enforce PWM pins as output
    mPORTDSetPinsDigitalOut(BIT_1|BIT_2|BIT_3);
    // Start up as low
    PORTClearBits(IOPORT_D,BIT_1|BIT_2|BIT_3);

    int i = 0;
    for (i = 0; i < 100000; i++);

    // Enable level shifter
    IO_enableLevelShifter1(true);
    IO_enableLevelShifter2(true);

    // Setup PWM
    IO_setupPWM();

    // Setup CT interrupt
    IO_setupTICK();

    // Setup ADC for battery voltage read
    IO_setupADC();

    IO_SystemState = OKAY;
}

void IO_changeNotificationSetup(void){
    // Set speed sensor pin and config change notice interrupt
    // D6,CN15: OPTO-ENCODER
    // D7,CN16: GYRO_GAIN [6]
    // D13,CN19: SERVO_LEFT [7]
    // AN5,B5,CN7: NOT USED
    // AN4,B4,CN6: ESC [2]
    // AN3,B3,CN5: SERVO_RIGHT [3]
    // AN2,B2,CN4: SERVO_REAR [4]
    // AN1,B1,CN3: RUDDER [5]
    PORTSetPinsDigitalIn(IOPORT_D, BIT_6 | BIT_7);// | BIT_13);
//    PORTSetPinsDigitalIn(IOPORT_B, BIT_4 | BIT_3 | BIT_2 | BIT_1);

    mCNOpen(CONFIG, PINS, PULLUPS);
    ConfigIntCN(CHANGE_INT_ON | CHANGE_INT_PRI_7);
    INTSetVectorSubPriority(_CHANGE_NOTICE_VECTOR, INT_SUB_PRIORITY_LEVEL_3);
}

void IO_initialize_FS(void){
    // Wait here until SD card responds with idle
    while(!MDD_SDSPI_MediaDetect());
    // Initialize file system
    while (!FSInit());

    IO_initializeFile(&IO_log, IO_FS_LOG);
}

bool IO_initializeFile(IO_file * IO_file_ds, const char * IO_file_name){
    // setup debug file for writing
    memset(IO_file_ds,0,sizeof(IO_file));
    sprintf(&(IO_file_ds->file_name[0]), IO_file_name);
    IO_file_ds->fsFile = FSfopen(&(IO_file_ds->file_name[0]), "w");
    if (IO_file_ds->fsFile == NULL){
        IO_file_ds->error = OPEN_ERROR;
        IO_file_ds->file_opened = false;
        return false;
    }
    else
    {
        IO_file_ds->file_opened = true;
        IO_file_ds->write_length = 0;
        return true;
    }
}

void IO_terminate_FS(void){
    if (IO_dmesg.file_opened){
        if(FSfclose(IO_dmesg.fsFile))
            IO_dmesg.error = CLOSE_ERROR;
        else
            IO_dmesg.file_opened = false;
    }

    if (IO_log.file_opened){
        if(FSfclose(IO_log.fsFile))
            IO_log.error = CLOSE_ERROR;
        else
            IO_log.file_opened = false;
    }
}

bool IO_debugMessage(char * str){
    IO_write_FS(&IO_dmesg, str, strlen(str));
}

bool IO_write_FS(IO_file * io_file, char * data, UINT16 len){
    // this function writes data to the fd buffer
    // if the fd buffer overflows then it copies the balance into the overflow buffer
    // and signals buffer overflow--so that flush_FS can be called with full 512 buffer
    // assume len is greater than 512
    if (io_file->file_opened){
        if (!(io_file->buffer_overflow)){
            // if more data can be written to main buffer--> add to buffer
            UINT16 buffer_remainder = SD_WRITE_BUFFER_SIZE - io_file->write_length;
            if (buffer_remainder > len){
                memcpy(&(io_file->data_buffer[io_file->write_length]), data, len);
                io_file->write_length += len;
            }
            else
            {
                // first fill in whatever fits in main buffer
                memcpy(&(io_file->data_buffer[io_file->write_length]), data, buffer_remainder);
                io_file->write_length += buffer_remainder;

                // now write remainer into overflow buffer
                memcpy(&(io_file->overflow_buffer[0]), data + buffer_remainder, len - buffer_remainder);
                io_file->overflow_length = len - buffer_remainder;
                io_file->buffer_overflow = true;
            }
        }
        else{
            // if in here-means that overflow buffer has been written to and has not been flushed
            memcpy(&(io_file->overflow_buffer[io_file->overflow_length]), data, len);
            io_file->overflow_length += len;
        }
        
        return true;
    }
    else
        return false;
}

bool IO_flush_FS(void){
    // only if main buffer is full (512), do a write
    if (IO_dmesg.buffer_overflow && IO_dmesg.file_opened){
        if (FSfwrite (IO_dmesg.data_buffer, 1, IO_dmesg.write_length, IO_dmesg.fsFile) != IO_dmesg.write_length){
            IO_dmesg.error = WRITE_ERROR;
            return false;
        }
        else{
            IO_dmesg.buffer_overflow = false;
            IO_dmesg.write_length = 0;
            // now move data from overflow to main buffer
            memcpy(&(IO_dmesg.data_buffer[0]), &(IO_dmesg.overflow_buffer[0]), IO_dmesg.overflow_length);
            IO_dmesg.write_length = IO_dmesg.overflow_length;
            IO_dmesg.overflow_length = 0;
            return true;
        }
    }

    // only if main buffer is full (512), do a write
    if (IO_log.buffer_overflow && IO_log.file_opened){
        if (FSfwrite (IO_log.data_buffer, 1, IO_log.write_length, IO_log.fsFile) != IO_log.write_length){
            IO_log.error = WRITE_ERROR;
            return false;
        }
        else{
            IO_log.buffer_overflow = false;
            IO_log.write_length = 0;
            // now move data from overflow to main buffer
            memcpy(&(IO_log.data_buffer[0]), &(IO_log.overflow_buffer[0]), IO_log.overflow_length);
            IO_log.write_length = IO_log.overflow_length;
            IO_log.overflow_length = 0;
            return true;
        }
    }
}

void IO_logMessage(char * str, UINT16 len){
    IO_write_FS(&IO_log, str, len);
}

bool IO_getCloseFiles(void){
    return IO_closeFiles;
}

void IO_enableLevelShifter1(BOOL val)
{
    IO_LEVELSHIFT1TRIS();
    if (val)
	PORTSetBits(IOPORT_A, BIT_4);
    else
	PORTClearBits(IOPORT_A, BIT_4);
}

void IO_enableLevelShifter2(BOOL val)
{
    IO_LEVELSHIFT2TRIS();
    if (val)
	PORTSetBits(IOPORT_A, BIT_5);
    else
	PORTClearBits(IOPORT_A, BIT_5);
}

void IO_setupPWM(void){
    // Enforce PWM pins as output
    mPORTDSetPinsDigitalOut(BIT_1|BIT_2|BIT_3);//|BIT_4);
    // Start up as low
    PORTClearBits(IOPORT_D,BIT_1|BIT_2|BIT_3);//|BIT_4);

    OpenOC2( OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0);
    OpenOC3( OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0);
    OpenOC4( OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0);
    OpenOC5( OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0);

    // Timer 2 drives PWM module
    OpenTimer2( T2_ON | T2_PS_1_32 | T2_SOURCE_INT, IO_PWM_PERIOD);

    // Put out PWM but LED's are off!
    IO_setupPWM_default();
}

void IO_setupPWM_default(void){
    IO_LED_ON = false;
    SetDCOC2PWM(IO_PWM_dc(0.0));
    SetDCOC3PWM(IO_PWM_dc(0.0));
    SetDCOC4PWM(IO_PWM_dc(0.0));
}

UINT16 IO_PWM_dc(float dc){
    if (dc > 100.0)
        return IO_PWM_PERIOD;
    
    if (dc < 0.0)
        return 0;

    return (UINT16)floor(IO_PWM_PERIOD*dc/100);
}

void IO_LED_init(void){
    IO_leds_on(60.0);
    IO_delayms(200);
    
    IO_setupPWM_default();
    IO_delayms(200);

    IO_leds_on(60.0);
    IO_delayms(200);
    
    IO_setupPWM_default();
    IO_delayms(200);

    IO_leds_on(60.0);
    IO_delayms(200);
    
    IO_setupPWM_default();
    IO_delayms(200);

    IO_leds_on(60.0);
    IO_delayms(200);

    IO_setupPWM_default();
    IO_delayms(200);
    IO_leds_on(50.0);
}

void IO_leds_on(float power){
    IO_LED_ON = true;
    SetDCOC2PWM(IO_PWM_dc(power));
    SetDCOC3PWM(IO_PWM_dc(power));
    SetDCOC4PWM(IO_PWM_dc(power));
}

bool IO_getLEDState(void){
    return IO_LED_ON;
}

void IO_setupTICK(void){
    // Open up the core timer at 1ms rate
    OpenCoreTimer(CORE_TICK_RATE);
    // Set up the core timer interrupt with a prioirty of 7 and sub-priority 2
    mConfigIntCoreTimer((CT_INT_ON | CT_INT_PRIOR_7 | CT_INT_SUB_PRIOR_2));
}

void IO_setSystemState(IO_SYSTEM_STATE state){
    IO_SystemState = state;
}

IO_SYSTEM_STATE IO_getSystemState(void){
    return IO_SystemState;
}

void IO_delayms(unsigned int num_ms_delay)
{
    if (num_ms_delay < 200)
    {
	IO_delayUnder200ms(num_ms_delay);
    }
    else
    {
	// if the delay is greater than 200ms
	unsigned int remainder = num_ms_delay;
	while(remainder > 200)
	{
            IO_delayUnder200ms(100);
            remainder -= 100;
	}
	if (remainder > 0)
            IO_delayUnder200ms(remainder);
    }
}

void IO_delayUnder200ms(unsigned int val)
{
    // calculate required period
    unsigned short int temp_period = (unsigned short int)((double)val*80e3/256);
    // clear interrupt flag
    mT1ClearIntFlag();
    // setup timer with pre-scaler = 256 --> a resolution of 3.2e-6s
    OpenTimer1(T1_ON | T1_SOURCE_INT | T1_PS_1_256, temp_period);
    // wait here until period match
    while(!mT1GetIntFlag());
    // clear interrupt flag
    mT1ClearIntFlag();
    CloseTimer1();
}

void IO_setupADC(void)
{
    // make AD0 an analog pin
    AD1PCFG = 0xFFFE;
    AD1CHS = 0x0000;
    AD1CON1 = 0x00E0;
    AD1CSSL = 0x0001;
    AD1CON2 = 0x6000;
    AD1CON3 = 0x1FFF;

    /* Set up interrupts */
    ConfigIntADC10(ADC_INT_PRI_1 | ADC_INT_SUB_PRI_3 |ADC_INT_ON);
    AD1CON1 |= 0x8000;					/* Turn ADC module on */
}

volatile UINT32 IO_get_time_ms(void){
    return IO_time_ms;
}

void IO_setSendDataTime(UINT16 delta_t){
    IO_DATA_TIME = IO_time_ms + delta_t;
}

void IO_set_change_LED(bool val){
    IO_change_LED = val;
}

bool IO_get_change_LED(void){
    return IO_change_LED;
}

UINT16 IO_getBatteryVoltage(void){
    return IO_batteryVoltage;
}

bool IO_getSendData(void){
    return IO_sendData;
}

void IO_setSendData(bool val){
    IO_sendData = val;
}

bool IO_getLocalControl(void){
    return IO_localControl;
}

void IO_setLocalControl(bool val){
    IO_localControl = val;
}

bool IO_getStateProject(void){
    return IO_stateProject;
}

void IO_setStateProject(bool val){
    IO_stateProject = val;
}

bool IO_getPWMUpdate(void){
    return IO_PWM_update;
}

void IO_setPWMUpdate(bool val){
    IO_PWM_update = val;
}

bool IO_getSDFlush(void){
    return IO_sdFlush;
}

void IO_setSDFlush(bool val){
    IO_sdFlush = val;
}

float IO_getRudder_dt(void){
    return (float) IO_rudder_dt / (40e3);
}

float IO_getServoLeft_dt(void){
    return (float) IO_servo_left_dt / (40e3);
}

float IO_getServoRight_dt(void){
    return (float) IO_servo_right_dt / (40e3);
}

float IO_getServoRear_dt(void){
    return (float) IO_servo_rear_dt / (40e3);
}

float IO_getESC_dt(void){
    return (float) IO_esc_dt / (40e3);
}

float IO_getGyroGain_dt(void){
    return (float) IO_gyro_gain_dt / (40e3);
}

unsigned short int IO_getRotorSpeed(void){
    if ((IO_rotor_speed_dt != 0) && (!IO_rotor_speed_timeout)){
        IO_rotor_speed = ((float)240e6)/((float)IO_rotor_speed_dt);
        return (unsigned short int) floor(IO_rotor_speed + 0.5);
    }
    return 0;
}

bool IO_timeoutSpeed(void){
    if (IO_rotor_speed_update++ > IO_SPEED_TIMEOUT){
        IO_rotor_speed_timeout = true;
        return true;
    }
    else
    {
        IO_rotor_speed_timeout = false;
        return false;
    }
}

UINT16 IO_getRudder(void){
    return (UINT16) (IO_rudder_dt / (40));
}

UINT16 IO_getServoLeft(void){
    return (UINT16) (IO_servo_left_dt / (40));
}

UINT16 IO_getServoRight(void){
    return (UINT16) (IO_servo_right_dt / (40));
}

UINT16 IO_getServoRear(void){
    return (UINT16) (IO_servo_rear_dt / (40));
}

UINT16 IO_getESC(void){
    return (UINT16) (IO_esc_dt / (40));
}

UINT16 IO_getGyroGain(void){
    return (UINT16) (IO_gyro_gain_dt / (40));
}

char IO_sd_buffer[64];
void IO_speedController(void){
    static float ref_speed = 0;
    static float internal_speed_ref = 0;
    static float heave_control_sig = 0;
    static float speed_input[3] = {0}, speed_output[3]={0};
    static UINT32 prev_timestamp_heave = 0;
    static UINT32 prev_timestamp_speed = 0;
    static bool reference_above_landing = false;
    static bool landing_routine_active = false;
    char PWM_data[10] = {0};

    // ********************************************************************* //
    // *** BEGIN Motor speed control *************************************** //
    // ********************************************************************* //
    float throttle = (float)SPEKTRUM_getESC();
    if (throttle > 1100)
        internal_speed_ref = internal_speed_ref + 1;
    else
        internal_speed_ref = 0;

    // double rate of increase if speed if greater than 1500 rpm
    if (internal_speed_ref > 1500)
        internal_speed_ref = internal_speed_ref + 1;

    ref_speed = internal_speed_ref;

    // Limit controllable reference
    if (ref_speed < IO_RPM_MIN){
        ref_speed = 0;
    }
    if (ref_speed > IO_RPM_MAX){
        ref_speed = IO_RPM_MAX;
    }

    float current_speed = (float)IO_getRotorSpeed();
    float current_error = (ref_speed - current_speed)*IO_RPM_2_RAD_PER_SEC;
    UINT32 current_time_speed = IO_get_time_ms();
    float speed_dt = (current_time_speed - prev_timestamp_speed)*1e-3;
    float upper_limit_int_speed = IO_SPEED_UPPER_LIMIT*IO_SPEED_OMEGA_TI/IO_SPEED_KP;
    float motor_pi = IO_PI_with_limits(IO_SPEED_KP, IO_SPEED_OMEGA_TI, speed_dt, current_error,
        &speed_input[0], &speed_output[0], 0, upper_limit_int_speed);
    prev_timestamp_speed = current_time_speed;
    // ******************************************* END Motor speed control ***//


    int right = (int)SPEKTRUM_getRIGHT() - SPEKTRUM_RIGHT_NOM;
    int rear = -((int)SPEKTRUM_getREAR() - SPEKTRUM_REAR_NOM);
    int left = -((int)SPEKTRUM_getLEFT() - SPEKTRUM_LEFT_NOM);
    float collective = (right+left+rear)/3;
    IO_roll_ref = -(float)(right-left)/(10);
    IO_pitch_ref = (float)((right+left)/2-rear)/(10);   // 10 scales signal for 10 deg range

    float lateral = (float)(right-left)/(2);
    float longitudinal = (float)((right+left)/2-rear)/3;

    // ********************************************************************* //
    // *** BEGIN Heave control ********************************************* //
    // ********************************************************************* //
    float altitude_ref = 0;
    float altitude_raw_ref = 0;
    if (throttle < 1320)
        altitude_ref = (throttle - 931)/1297;
    else
        altitude_ref = (throttle - 1320)/197 + 0.3;

    altitude_raw_ref = altitude_ref;

    if (altitude_ref > 0.3)
        reference_above_landing = true;

    if (reference_above_landing){
        if ((altitude_ref < 0.2879) || (landing_routine_active)){
            altitude_ref = IO_landingRefGen(speed_dt, false);
            landing_routine_active = true;
        }
        if (altitude_raw_ref < 0.01){
            reference_above_landing = false;
            IO_landingRefGen(0, true);
            landing_routine_active = false;
        }
    }

    if (altitude_ref > IO_ALTITUDE_LIMIT)   
        altitude_ref = IO_ALTITUDE_LIMIT;
    if (altitude_ref < 0)
        altitude_ref = 0;

    float height_sensor = 0;
    if (IO_getLocalControl())
        height_sensor = ULTRASONIC_getHeave() + 0.0879;
    else
        height_sensor = (float)RN131_get_tz()/1000;

//    if (ULTRASONIC_isUpdated())
//    {
//        ULTRASONIC_setUpdatedFalse();

        UINT32 current_time = IO_get_time_ms();
        float heave_dt = (current_time - prev_timestamp_heave)*1e-3;
        prev_timestamp_heave = current_time;
        heave_control_sig = IO_heave_control(altitude_ref, height_sensor, heave_dt);//ULTRASONIC_getHeave() + 0.0879, heave_dt);

        // limit heave control sig to 170 units
        heave_control_sig = IO_limits(IO_HEAVE_LOWER_LIMIT, IO_HEAVE_UPPER_LIMIT, heave_control_sig);
//    }
    // ************************************************* END Heave control ***//

    /*****************/
    /* Yaw control */
    /*****************/
//    float yaw_rate_ref = -50*(float)(SPEKTRUM_getRUDDER()-SPEKTRUM_getYawNominal())/300;
//    if ((yaw_rate_ref > -3.3) && (yaw_rate_ref < 3.3)){
//        yaw_rate_ref = 0;
//    }
//    else{
//        if (yaw_rate_ref > 0)
//            yaw_rate_ref += 3.3;
//        if (yaw_rate_ref < 0)
//            yaw_rate_ref -= 3.3;
//    }
    float yaw_rate_ref = -200*(float)(SPEKTRUM_getRUDDER()-SPEKTRUM_getYawNominal())/300;
    if ((yaw_rate_ref > -13.2) && (yaw_rate_ref < 13.2)){
        yaw_rate_ref = 0;
    }
    else{
        if (yaw_rate_ref > 0)
            yaw_rate_ref -= 13.2;
        if (yaw_rate_ref < 0)
            yaw_rate_ref += 13.2;
    }

    float yaw_control_sig = IO_yaw_control(speed_dt, yaw_rate_ref);
    UINT8 yaw_pwm = PWM_PULSEWIDTH2BYTE((int)yaw_control_sig);

    /*****************/
    /* Roll control */
    /*****************/
    IO_lateral = IO_roll_control(IO_roll_ref, speed_dt);
    IO_lateral = IO_limits(IO_ROLL_LOWER_LIMIT, IO_ROLL_UPPER_LIMIT, IO_lateral);
    /*****************/

    /*****************/
    /* Pitch control */
    /*****************/
    IO_longitudinal = IO_pitch_control(IO_pitch_ref, speed_dt);
    IO_longitudinal = IO_limits(IO_PITCH_LOWER_LIMIT, IO_PITCH_UPPER_LIMIT, IO_longitudinal);
    /*****************/

    IO_longitudinal = longitudinal;
    IO_lateral = lateral;
    IO_collective = collective;
    heave_control_sig = collective;

    int servo_right = PWM_PULSEWIDTH2BYTE((int)(SPEKTRUM_RIGHT_NOM + IO_lateral + IO_longitudinal + heave_control_sig));
    int servo_rear = PWM_PULSEWIDTH2BYTE((int)(SPEKTRUM_REAR_NOM + 2*IO_longitudinal - heave_control_sig));
    int servo_rudder = PWM_PULSEWIDTH2BYTE(SPEKTRUM_getRUDDER());
    int servo_left = PWM_PULSEWIDTH2BYTE((int)(SPEKTRUM_LEFT_NOM + IO_lateral - IO_longitudinal - heave_control_sig));

    UINT8 ESC_input = (UINT8) IO_limits(IO_PWM_SAT_MIN, IO_PWM_SAT_MAX, motor_pi);

    PWM_data[0] = ESC_input;
    PWM_data[1] = servo_right;
    PWM_data[2] = servo_rear;
    PWM_data[3] = servo_rudder;//yaw_pwm;
    PWM_data[4] = PWM_PULSEWIDTH2BYTE(SPEKTRUM2PULSEWIDTH(422));
    PWM_data[5] = servo_left;

    bool auto_mode_on = SPEKTRUM_isAutoMode();

    bool ack = PWM_sendPacket(auto_mode_on,PWM_data,6);
//    IO_generalLog((float)SPEKTRUM_getRUDDER(), ITG3200_getz_float(), (float)servo_rudder);
//    IO_generalLog(IO_yaw_control_sig, ITG3200_getz_float(), yaw_sense_sig);
    IO_logVital(auto_mode_on, current_speed, altitude_ref, ULTRASONIC_getHeave() + 0.0879);
}

char IO_limit_PWM(int sig){
    if (sig > 255)
        sig = 255;
    if (sig < 0)
        sig = 0;
    return (char)sig;
}

float IO_landingRefGen(float dt, bool reset){
    static float ref_height = 0.2879;
    static float time = 0;
    
    if (reset == true){
        ref_height = 0.2879;
        time = 0;
    }

    time = time + dt;
    ref_height = 0.2879 - time*(0.2879/5);

    ref_height = IO_limits(0, 0.2879, ref_height);

    return ref_height;
}

float IO_limits(float lower, float upper, float input){
    if (input > upper)
        input = upper;
    if (input < lower)
        input = lower;
    return input;
}

void IO_logVital(bool ack, float motor_speed, float altitude_ref,
        float altitude_sense){
    int sdlen = sprintf(&IO_sd_buffer[0],"%d,%.2f,%.3f,%.3f,%.3f,%d,%.3f,%.3f,%d,%d,%d,%lu\r\n",
                        ack,
                        motor_speed,
                        altitude_ref,
                        altitude_sense,
                        (float)RN131_get_tz()/1000,
                        IO_getLocalControl(),
                        yaw_ref_sig,
                        yaw_sense_sig,
                        IO_getBatteryVoltage(),
                        SPEKTRUM_getChannel4(),
                        SPEKTRUM_getTimeout(),
                        IO_get_time_ms());

    IO_logMessage(&IO_sd_buffer[0], sdlen);
}

void IO_logSensorData(void){
    int sdlen = sprintf(&IO_sd_buffer[0],"%.3f,%.3f,%.3f,%.3f,%lu\r\n",
                        ITG3200_getFilteredTemperature(),
                        ITG3200_getxDPS(),
                        ITG3200_getyDPS(),
                        ITG3200_getzDPS(),
                        IO_get_time_ms());

    IO_logMessage(&IO_sd_buffer[0], sdlen);
}

void IO_logAttitudeData(void){
    int sdlen = sprintf(&IO_sd_buffer[0],"%.2f,%.2f,%.2f,%lu\r\n",
                        IMU_getRoll(),
                        IMU_getPitch(),
                        IMU_getYaw(),
                        IO_get_time_ms());
    IO_logMessage(&IO_sd_buffer[0], sdlen);
}

void IO_logPitchData(void){
    int sdlen = sprintf(&IO_sd_buffer[0],"%5.2f,%5.2f,%5.2f,%lu\r\n",
                        IO_pitch_ref,
                        IMU_getPitch(),
                        IO_longitudinal,
                        IO_get_time_ms());
    IO_logMessage(&IO_sd_buffer[0], sdlen);
}

void IO_logRollData(void){
    int sdlen = sprintf(&IO_sd_buffer[0],"%5.2f,%5.2f,%5.2f,%5.2f,%lu\r\n",
                        IO_roll_ref,
                        IMU_getRoll(),
                        IO_droll_err,
                        IO_lateral,
                        IO_get_time_ms());
    IO_logMessage(&IO_sd_buffer[0], sdlen);
}

void IO_logYawData(void){
    int sdlen = sprintf(&IO_sd_buffer[0],"%5.2f,%5.2f,%5.2f,%5.2f,%lu\r\n",
                        yaw_ref_sig,
                        yaw_sense_sig,
                        IO_get_time_ms());
    IO_logMessage(&IO_sd_buffer[0], sdlen);
}

void IO_generalLog(float a1, float a2, float a3){
    int sdlen = sprintf(&IO_sd_buffer[0],"%.3f,%.3f,%.3f,%lu\r\n",
                a1, a2, a3,IO_get_time_ms());
    IO_logMessage(&IO_sd_buffer[0], sdlen);
}

void IO_PIDComputation(IO_PID_TYPE pidtype, float Kp, float omega_Td, float omega_Ti, float alpha, float Ts,float u[3], float e[3]){
    float nz1 = 0, nz2 = 0, nz3 = 0;
    float dz1 = 0, dz2 = 0, dz3 = 0;
    // compute coefficients

    if (pidtype == IO_PID){
        float Ts_squared = pow(Ts,2);
        nz1 = (4*Kp*alpha + 2*Kp*Ts*alpha*omega_Td + 2*Kp*Ts*alpha*omega_Ti + Kp*Ts_squared*alpha*omega_Td*omega_Ti);
        nz2 = (2*Kp*alpha*omega_Td*omega_Ti*Ts_squared - 8*Kp*alpha);
        nz3 = 4*Kp*alpha - 2*Kp*Ts*alpha*omega_Td - 2*Kp*Ts*alpha*omega_Ti + Kp*Ts_squared*alpha*omega_Td*omega_Ti;
        dz1 = (4*omega_Ti);
        dz2 = (-8*omega_Ti);
        dz3 = (4*omega_Ti);
    }
    else
    {
        nz1 = (2*Kp*alpha + Kp*Ts*alpha*omega_Td);
        nz2 = Kp*Ts*alpha*omega_Td - 2*Kp*alpha;
        nz3 = 0;
        dz1 = (Ts*alpha*omega_Td + 2);
        dz2 = Ts*alpha*omega_Td - 2;
        dz3 = 0;
    }
    // second-order discrete algo from z-transform
    u[0] = (nz1*e[0] + nz2*e[1] + nz3*e[2] - dz2*u[1] - dz3*u[2])/dz1;
}

void IO_PID_rateAux(IO_PID_TYPE pidtype, float Kp, float omega_Td, float omega_Ti, float alpha, float Ts,float u[3], float e[3]){
    float nz1 = 0, nz2 = 0, nz3 = 0;
    float dz1 = 0, dz2 = 0, dz3 = 0;
    // compute coefficients

    if (pidtype == IO_PID){
        float Ts_squared = pow(Ts,2);
        nz1 = (4*Kp*alpha + 2*Kp*Ts*alpha*omega_Td + 2*Kp*Ts*alpha*omega_Ti + Kp*Ts_squared*alpha*omega_Td*omega_Ti);
        nz2 = (2*Kp*alpha*omega_Td*omega_Ti*Ts_squared - 8*Kp*alpha);
        nz3 = 4*Kp*alpha - 2*Kp*Ts*alpha*omega_Td - 2*Kp*Ts*alpha*omega_Ti + Kp*Ts_squared*alpha*omega_Td*omega_Ti;
        dz1 = (4*omega_Ti);
        dz2 = (-8*omega_Ti);
        dz3 = (4*omega_Ti);
    }
    else
    {
        nz1 = (Kp*Ts*alpha*omega_Td);
        nz2 = (Kp*Ts*alpha*omega_Td);
        nz3 = 0;
        dz1 = (Ts*alpha*omega_Td + 2);
        dz2 = (Ts*alpha*omega_Td - 2);
        dz3 = 0;
    }
    // second-order discrete algo from z-transform
    u[0] = (nz1*e[0] + nz2*e[1] + nz3*e[2] - dz2*u[1] - dz3*u[2])/dz1;
}

void IO_PID_with_limits(float Kp, float omega_lead, float omega_Ti, float alpha, float Ts,
        float int_inputs[3], float int_outputs[3], float current_error,
        float int_lower_limit, float int_upper_limit,
        float diff_inputs[3], float diff_outputs[3]){

    // Perform PI control first
    IO_PI_with_limits(Kp, omega_Ti, Ts, current_error,
        &int_inputs[0], &int_outputs[0], int_lower_limit, int_upper_limit);

    // supply the output of the PI controller to the lead-lag filter
    float omega_lag = alpha*omega_lead;
    IO_lead_lag(omega_lead, omega_lag, Ts, &diff_inputs[0], &diff_outputs[0]);
}

float IO_PI_with_limits(float Kp, float omega_Ti, float Ts, float current_error,
    float filter_input[3], float filter_output[3], float lower_limit, float upper_limit){
    float nz1 = 0, nz2 = 0;
    float dz1 = 0, dz2 = 0;

    nz1 = 0;
    nz2 = 2*(Ts*omega_Ti);
    dz1 = (Ts*omega_Ti + 2);
    dz2 = (Ts*omega_Ti - 2);

    // shift input and output data
    IO_shiftData(&filter_input[0]);
    IO_shiftData(&filter_output[0]);
    // perform filter operation
    IO_filter_1st_order(nz1,nz2,dz1,dz2,&filter_input[0],&filter_output[0]);

    float current_output = current_error + filter_output[0];

    // Put saturation limits on integral control
    filter_input[0] = IO_limits(lower_limit, upper_limit, current_output);

    return filter_input[0]*Kp/omega_Ti;
}

float IO_lead_lag(float omega_lead, float omega_lag, float Ts, float filter_input[3], float filter_output[3]){
    float n1 = omega_lag*(Ts*omega_lead + 2);
    float n2 = omega_lag*(Ts*omega_lead - 2);
    float d1 = omega_lead*(Ts*omega_lag + 2);
    float d2 = omega_lead*(Ts*omega_lag - 2);

    IO_filter_1st_order(n1,n2,d1,d2,&filter_input[0],&filter_output[0]);

    return filter_output[0];
}

void IO_filter_1st_order(float n1, float n2, float d1, float d2, float input[3], float output[3]){
    output[0] = (n1*input[0] + n2*input[1] - d2*output[1])/d1;
}

void IO_filter_2nd_order(float n0, float n1, float n2, float d0, float d1, float d2, float input[3], float output[3]){
    output[0] = (n0*input[0] + n1*input[1] + n2*input[2] - d1*output[1] - d2*output[2])/d0;
}

float IO_filter_notch(float wn, float damp_p, float damp_z, float Ts, float input[3], float output[3]){
    float Ts_sqrd = pow(Ts,2);
    float wn_sqrd = pow(wn,2);
    float n0 = (Ts_sqrd*wn_sqrd + 4*damp_z*Ts*wn + 4);
    float n1 = (2*Ts_sqrd*wn_sqrd - 8);
    float n2 =  Ts_sqrd*wn_sqrd - 4*damp_z*Ts*wn + 4;
    float d0 = (Ts_sqrd*wn_sqrd + 4*damp_p*Ts*wn + 4);
    float d1 = (2*Ts_sqrd*wn_sqrd - 8);
    float d2 =  Ts_sqrd*wn_sqrd - 4*damp_p*Ts*wn + 4;

    IO_filter_2nd_order(n0, n1, n2, d0, d1, d2, &input[0], &output[0]);
    return output[0];
}

float IO_roll_control(float roll_reference, float dt){
    static float notch_input[3] = {0}, notch_output[3] = {0};

    float roll_error = -(roll_reference - IMU_getRoll());

    IO_shiftData(&notch_input[0]);
    notch_input[0] = roll_error;
    IO_shiftData(&notch_output[0]);

    return IO_ROLL_KP*IO_filter_notch(IO_ROLL_NOTCH_WN, IO_ROLL_NOTCH_WP_DAMP, IO_ROLL_NOTCH_WZ_DAMP, dt, &notch_input[0], &notch_output[0]);
}

float IO_pitch_control(float pitch_reference, float dt){
    static float notch_input[3] = {0}, notch_output[3] = {0};

    float pitch_error = (pitch_reference - IMU_getPitch());

    IO_shiftData(&notch_input[0]);
    notch_input[0] = pitch_error;
    IO_shiftData(&notch_output[0]);

    return IO_PITCH_KP*IO_filter_notch(IO_PITCH_NOTCH_WN, IO_PITCH_NOTCH_WP_DAMP, IO_PITCH_NOTCH_WZ_DAMP, dt, &notch_input[0], &notch_output[0]);
}

float IO_heave_control(float heave_reference, float height_sensor, float dt){
    static float heave_error[3] = {0}, heave_input[3] = {0};
    static float int_input[3] = {0}, int_output_[3] = {0};

    IO_shiftData(&heave_error[0]);
    heave_error[0] = heave_reference - height_sensor;
    IO_shiftData(&heave_input[0]);

    // calculate derivative control signal
    float derivative_signal = IO_HEAVE_KP*IO_lead_lag(IO_HEAVE_OMEGA_LEAD, IO_HEAVE_OMEGA_LAG, dt, &heave_error[0], &heave_input[0]);
    float lower_limit_int = 0;
    float upper_limit_int = IO_HEAVE_UPPER_LIMIT*IO_HEAVE_OMEGA_TI;

    return IO_PI_with_limits(1, IO_HEAVE_OMEGA_TI, dt, derivative_signal,
        &int_input[0], &int_output_[0], lower_limit_int, upper_limit_int);
}

float IO_yaw_control(float dt, float yaw_rate_ref){
    static float yaw_error[3] = {0}, yaw_input[3] = {0};
    static float int_input[3] = {0}, int_output_[3] = {0};

    static float yaw_sense_cont = 0;
    static bool first_enter = true;
    static float yaw_ref = 0;
    static float prev_yaw_rate_ref = 0;

    if (first_enter){
        yaw_sense_cont = IMU_getYaw();
        yaw_ref = yaw_sense_cont;
        first_enter = false;
    }
    else
    {
        int yaw_diff = (int)COM_round((yaw_sense_cont - IMU_getYaw())/(360));
        yaw_sense_cont = IMU_getYaw() + yaw_diff*360;
    }

    // Compute yaw ref
    yaw_ref = yaw_ref + dt*(yaw_rate_ref + prev_yaw_rate_ref)/2;
//    yaw_ref = yaw_rate_ref;
    prev_yaw_rate_ref = yaw_rate_ref;

    IO_shiftData(&yaw_error[0]);
    /** Convert this error to rad/s !!!**/
    yaw_error[0] = (yaw_ref - yaw_sense_cont)*M_PI/180;
    IO_shiftData(&yaw_input[0]);

    yaw_ref_sig = yaw_ref;
    yaw_sense_sig = yaw_sense_cont;

    // calculate derivative control signal
    float derivative_signal = IO_YAW_KP*IO_lead_lag(IO_YAW_OMEGA_LEAD, IO_YAW_OMEGA_LAG, dt, &yaw_error[0], &yaw_input[0]);
    float lower_limit_int = IO_YAW_LOWER_LIMIT*IO_YAW_OMEGA_TI;
    float upper_limit_int = IO_YAW_UPPER_LIMIT*IO_YAW_OMEGA_TI;

    float yaw_control_sig = IO_PI_with_limits(1, IO_YAW_OMEGA_TI, dt, derivative_signal,
        &int_input[0], &int_output_[0], lower_limit_int, upper_limit_int);
    IO_yaw_control_sig = yaw_control_sig;

    return (SPEKTRUM_getYawNominal() - yaw_control_sig);
}


void IO_shiftData(float data[3]){
    data[2] = data[1];
    data[1] = data[0];
}

/*
    ADC interrupt
    PRIORITY: 1
    SUBPRIORITY: 3
*/
void __ISR(_ADC_VECTOR, ipl1) ADCInterrupt( void)
{
    mAD1ClearIntFlag();
    IO_batteryVoltage = ADC1BUF0;
}

/*
    Enter interrupt every 1ms
    PRIORITY: 7
    SUBPRIORITY: 2
 *      - used for timing reference
 *      - indicate system running
 *      - clear watchdog
*/
void __ISR(_CORE_TIMER_VECTOR, IPL7SRS) CoreTimerHandler(void)
{
    static UINT16 heartbeat_count = 0;
    
    // Clear the interrupt flag
    mCTClearIntFlag();

    IO_time_ms++;

    // Update the period
    UpdateCoreTimer(CORE_TICK_RATE);

    // Must be called at least every ms to keep USB active
    #ifdef USE_USB
        CDCTxService();
    #endif

    // Initiate ADC read
    if ((IO_time_ms % IO_ADC_READ) == 0)
    {
	AD1CON1bits.CLRASAM = 1;
	AD1CON1bits.ASAM = 1;
    }

    if ((IO_time_ms % IO_DATA_PERIOD) == 0){
        IO_sendData = true;
    }

//    if ((IO_time_ms % IO_STATE_PROJECT) == 0){
//        IO_stateProject = true;
//    }

//    // If data from pc takes too long set flag to switch to local control
//    if ((IO_time_ms % IO_RX_TIMEOUT_CHECK) == 0){
//        // if data is older than 1 second switch to local control
//        if ((IO_time_ms - RN131_getLastRxTime())  > (1000))
//            IO_setLocalControl(true);
//        else
//            IO_setLocalControl(false);
//    }

//    if (IO_time_ms == IO_DATA_TIME){
//        IO_sendData = true;
//    }

    if ((IO_time_ms % IO_SD_FLUSH) == 0){
        IO_sdFlush = true;
    }

    // Led dimming
    if(!(IO_time_ms % 10)){
        IO_change_LED = true;
    }

    // Take care of spektrum timeout. If new data arrives, SPEKTRUM_decodePacket() will
    // set autoMode appropriately
    if (SPEKTRUM_timeoutInc()){
        SPEKTRUM_setAutoMode(false);
    }

    // When rotor does not get updated, speed should be set to zero
    if (IO_timeoutSpeed()){
        IO_rotor_speed = 0;
    }

    if ((IO_time_ms % IO_PWM_UPDATE_DT) == 0){
        IO_PWM_update = true;
    }

    if (IO_SystemPreviousState != IO_SystemState)
        heartbeat_count = 0;

    switch(IO_SystemState){
        case IDLE:
            break;
        case OKAY:
            switch (heartbeat_count++){
                case 0:
                    IO_SET_HEARTBEAT();
                    break;
                case 100:
                    IO_CLEAR_HEARTBEAT();
                    break;
                case 175:
                    IO_SET_HEARTBEAT();
                    break;
                case 300:
                    IO_CLEAR_HEARTBEAT();
                    break;
                case 375:
                    IO_SET_HEARTBEAT();
                    break;
                case 1500:
                    heartbeat_count = 0;
                    break;
            }
            break;
        case MANUAL:
            switch (heartbeat_count++){
                case 1:
                    IO_SET_HEARTBEAT();
                    break;
                case 50:
                    IO_CLEAR_HEARTBEAT();
                    break;
                case 100:
                    heartbeat_count = 0;
                    break;
            }
            break;
        case FILE_CLOSED:

            break;
        case ERROR:
            // If an error, toggle every 1 second
            if ((IO_time_ms % IO_ERROR_BLINK) == 0){
                IO_TOGGLE_HEARTBEAT();
            }
            break;
    }

    IO_SystemPreviousState = IO_SystemState;

    // check if PROGRAM button is depressed, if so, close any open files
    if (!PORTReadBits(IOPORT_A, BIT_1))
    {
        IO_closeFiles = true;
    }

    // Clear watchdog timer
    ClearWDT();
}

/*
 * This interrupt is used for measuring motor speed
 * PRIORITY: 7
 * SUBPRIORITY: 3
 */
void __ISR(_CHANGE_NOTICE_VECTOR, IPL7SRS) changeNotificationHandler(void)
{
    static int prev_time_stamp[7] = {0};
    static bool first_run[7] = {true};
    static bool pin_prev_state[7] = {low};

    int temp_time;

    // clear the mismatch condition
    unsigned int port_val_D = mPORTDRead();
//    unsigned int port_val_B = mPORTBRead();

    // clear the interrupt flag
    mCNClearIntFlag();

    if ((port_val_D & IO_OPTO_ENCODER_PIND) && (pin_prev_state[0] == low))
    {
        pin_prev_state[0] = high;
        if (first_run[0])
        {
            prev_time_stamp[0] = ReadCoreTimer();
            first_run[0] = false;
        }
        else
        {
            temp_time = ReadCoreTimer();
            IO_rotor_speed_dt = (unsigned int) ReadCoreTimer() - (int) prev_time_stamp[0];
            IO_rotor_speed_update = 0;
            prev_time_stamp[0] = temp_time;
        }
    }
    if (!(port_val_D & IO_OPTO_ENCODER_PIND) && (pin_prev_state[0] == high))
    {
        pin_prev_state[0] = low;
    }

    if ((port_val_D & IO_GYRO_GAIN_PIND) && (pin_prev_state[1] == low))
    {
        pin_prev_state[1] = high;
        prev_time_stamp[1] = ReadCoreTimer();
    }
    if(!(port_val_D & IO_GYRO_GAIN_PIND) && (pin_prev_state[1] == high))
    {
        temp_time = ReadCoreTimer();
        IO_gyro_gain_dt = (unsigned int) ReadCoreTimer() - (int) prev_time_stamp[1];
        prev_time_stamp[1] = temp_time;
        IO_gyro_gain_update = true;
        pin_prev_state[1] = low;
    }

//    if ((port_val_D & IO_SERVO_LEFT_PIND) && (pin_prev_state[2] == low))
//    {
//        pin_prev_state[2] = high;
//        prev_time_stamp[2] = ReadCoreTimer();
//    }
//    if(!(port_val_D & IO_SERVO_LEFT_PIND) && (pin_prev_state[2] == high))
//    {
//        temp_time = ReadCoreTimer();
//        IO_servo_left_dt = (unsigned int) ReadCoreTimer() - (int) prev_time_stamp[2];
//        prev_time_stamp[2] = temp_time;
//        IO_servo_left_update = true;
//        pin_prev_state[2] = low;
//    }
//
//    if ((port_val_B & IO_ESC_PINB) && (pin_prev_state[3] == low))
//    {
//        pin_prev_state[3] = high;
//        prev_time_stamp[3] = ReadCoreTimer();
//    }
//    if(!(port_val_B & IO_ESC_PINB) && (pin_prev_state[3] == high))
//    {
//        temp_time = ReadCoreTimer();
//        IO_esc_dt = (unsigned int) ReadCoreTimer() - (int) prev_time_stamp[3];
//        prev_time_stamp[3] = temp_time;
//        IO_esc_update = true;
//        pin_prev_state[3] = low;
//    }
//
//    if ((port_val_B & IO_SERVO_RIGHT_PINB) && (pin_prev_state[4] == low))
//    {
//        pin_prev_state[4] = high;
//        prev_time_stamp[4] = ReadCoreTimer();
//    }
//    if(!(port_val_B & IO_SERVO_RIGHT_PINB) && (pin_prev_state[4] == high))
//    {
//        temp_time = ReadCoreTimer();
//        IO_servo_right_dt = (unsigned int) ReadCoreTimer() - (int) prev_time_stamp[4];
//        prev_time_stamp[4] = temp_time;
//        IO_servo_right_update = true;
//        pin_prev_state[4] = low;
//    }
//
//    if ((port_val_B & IO_SERVO_REAR_PINB) && (pin_prev_state[5] == low))
//    {
//        pin_prev_state[5] = high;
//        prev_time_stamp[5] = ReadCoreTimer();
//    }
//    if(!(port_val_B & IO_SERVO_REAR_PINB) && (pin_prev_state[5] == high))
//    {
//        temp_time = ReadCoreTimer();
//        IO_servo_rear_dt = (unsigned int) ReadCoreTimer() - (int) prev_time_stamp[5];
//        prev_time_stamp[5] = temp_time;
//        IO_servo_rear_update = true;
//        pin_prev_state[5] = low;
//    }
//
//    if ((port_val_B & IO_RUDDER_PINB) && (pin_prev_state[6] == low))
//    {
//        pin_prev_state[6] = high;
//        prev_time_stamp[6] = ReadCoreTimer();
//    }
//    if(!(port_val_B & IO_RUDDER_PINB) && (pin_prev_state[6] == high))
//    {
//        temp_time = ReadCoreTimer();
//        IO_rudder_dt = (unsigned int) ReadCoreTimer() - (int) prev_time_stamp[6];
//        prev_time_stamp[6] = temp_time;
//        IO_rudder_update = true;
//        pin_prev_state[6] = low;
//    }
}