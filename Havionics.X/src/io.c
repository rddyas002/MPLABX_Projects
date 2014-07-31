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
 * IO uses Timer 1 for delays
 * IO uses Timer 2 for PWM module
 * IO uses Timer 3 for executing the control loop
 * Interrupt on CN is highest priority for pulsewidth measurements
 */

IO_file IO_log;
IO_file IO_dmesg;

char IO_sd_buffer[256];

volatile UINT32 IO_time_ms = 0;
volatile UINT64 IO_time_raw = 0;
volatile UINT32 IO_DATA_TIME = 0;
IO_SYSTEM_STATE IO_SystemState = BOOT;
IO_SYSTEM_STATE IO_SystemPreviousState = BOOT;
UINT16 IO_batteryVoltage = 0;
volatile bool IO_sendData = false;
volatile bool IO_localControl = true;
volatile bool IO_stateProject = false;
volatile bool IO_sdFlush = false;
volatile bool IO_closeFiles = false;

/* Mutex type variables */
volatile bool IO_coreupdate_mutex = false;

void IO_enableLevelShifter1(BOOL val);
void IO_enableLevelShifter2(BOOL val);
void IO_setupPWM(void);
void IO_setupTICK(void);
void IO_setupADC(void);
void IO_delayUnder200ms(unsigned int val);
bool IO_write_FS(IO_file * io_file, char * data, UINT16 len);
bool IO_write_FSwInt(IO_file * io_file, char * data, UINT16 len);
bool IO_flush_FS(void);

#define CONFIG (CN_ON | CN_IDLE_CON)
#define PINS (CN3_ENABLE | CN4_ENABLE | CN5_ENABLE | CN6_ENABLE | CN7_ENABLE | CN15_ENABLE | CN16_ENABLE | CN19_ENABLE)
#define PULLUPS (CN_PULLUP_DISABLE_ALL)

volatile unsigned int IO_rotor_speed_dt = 0, IO_gyrogain_dt = 0, IO_ultrasonic_dt = 0;
volatile unsigned int IO_esc_dt = 0, IO_right_dt = 0, IO_rear_dt = 0, IO_rudder_dt = 0, IO_left_dt = 0;
volatile UINT16 IO_esc_nom = 0, IO_right_nom = 0, IO_rear_nom = 0, IO_rudder_nom = 0, IO_left_nom = 0;
volatile float IO_rotor_speed = 0;
volatile float IO_filteredRotorSpeed = 0;
volatile unsigned int IO_rotor_speed_update = 0;
volatile bool IO_rotor_speed_timeout = false;
volatile bool IO_LED_ON = false;
volatile bool IO_RADIO_ERROR = true;
volatile bool ultrasonic_updated = false;
volatile UCHAR IO_valid_PWM_data = 0x00;

// **********************************
// Roll channel control parameters
// ***********************************
#define IO_ROLL_KP              (720)
#define IO_ROLL_NOTCH_WN_Z      (15.45)
#define IO_ROLL_NOTCH_WZ_DAMP   (0.2)
#define IO_ROLL_NOTCH_WN_P      (15.5)
#define IO_ROLL_NOTCH_WP_DAMP   (2)
#define IO_LATERAL_BIAS         (0)
#define IO_ROLL_BIAS            (3.8*M_PI/180)
#define IO_ROLL_ANGLE_LOWER_LIMIT   (-30*M_PI/180)
#define IO_ROLL_ANGLE_UPPER_LIMIT   (30*M_PI/180)
#define IO_LATERAL_UPPER_LIMIT  (180)
#define IO_LATERAL_LOWER_LIMIT  (-180)

// **********************************
// Pitch channel control parameters
// ***********************************
#define IO_PITCH_KP              (295)
#define IO_PITCH_NOTCH_WN_Z      (14.5)
#define IO_PITCH_NOTCH_WZ_DAMP   (0.2)
#define IO_PITCH_NOTCH_WN_P      (18.5)
#define IO_PITCH_NOTCH_WP_DAMP   (1.4)
#define IO_LONGITUDINAL_BIAS     (5)
#define IO_PITCH_BIAS           (0.4*M_PI/180)
#define IO_PITCH_ANGLE_LOWER_LIMIT   (-30*M_PI/180)
#define IO_PITCH_ANGLE_UPPER_LIMIT   (30*M_PI/180)
#define IO_LONGITUDINAL_UPPER_LIMIT (110)
#define IO_LONGITUDINAL_LOWER_LIMIT (-110)

// ***********************************
// Position control
#define IO_X_KP             (-0.048)
#define IO_X_OMEGA_TI       (0.7)
#define IO_X_OMEGA_LEAD_1   (0.4)
#define IO_X_OMEGA_LAG_1    (40)
#define IO_X_OMEGA_LEAD_2   (0.6)
#define IO_X_OMEGA_LAG_2    (40)
#define IO_X_LPF_WN         (45)

#define IO_Y_KP             (0.045)
#define IO_Y_OMEGA_TI       (0.7)
#define IO_Y_OMEGA_LEAD_1   (0.45)
#define IO_Y_OMEGA_LAG_1    (40)
#define IO_Y_OMEGA_LEAD_2   (0.65)
#define IO_Y_OMEGA_LAG_2    (40)
#define IO_Y_LPF_WN         (45)

#define IO_X_LOWER_LIMIT    (-1)
#define IO_X_UPPER_LIMIT    (1)
#define IO_Y_LOWER_LIMIT    (-1)
#define IO_Y_UPPER_LIMIT    (1)

// ***********************************

// **********************************
// Heave channel control parameters
// ***********************************
#define IO_HEAVE_KP          (50)
#define IO_HEAVE_OMEGA_LEAD  (1.5)
#define IO_HEAVE_OMEGA_LAG   (40)
#define IO_HEAVE_OMEGA_TI    (1)
#define IO_NORMAL_HEAVE_UPPER_LIMIT (160)
#define IO_NORMAL_HEAVE_LOWER_LIMIT (0)
#define IO_TAKEOFF_HEAVE_UPPER_LIMIT (110)
#define IO_TAKEOFF_HEAVE_LOWER_LIMIT (0)
#define IO_ALTITUDE_LIMIT    (2)
#define IO_ALTITUDE_LPF_WN   (3)

// **********************************
// Yaw channel control parameters
// ***********************************
#define IO_YAW_KP          (87)
#define IO_YAW_OMEGA_LEAD  (1.5)
#define IO_YAW_OMEGA_LAG   (40)
#define IO_YAW_OMEGA_TI    (1.5)
#define IO_YAW_UPPER_LIMIT (191)
#define IO_YAW_LOWER_LIMIT (-274)
#define IO_YAW_LPF_WN        (5)

// Speed control parameters
// Input PWM 0-255, Output speed rad/s
// **********************************
//             (s/2.13 + 1)
// G(s) = 1.44 -----------      omega_gc = 4.12 rad/s
//                  s     
// With H(s) = 1/(s/(2*pi*10 + 1) + 1) on sensor signal
#define IO_SPEED_KP          (1.44)
#define IO_SPEED_OMEGA_TI    (2.13)
#define IO_SPEED_LPF_WN      (2*M_PI*5)
#define IO_SPEED_UPPER_LIMIT (254)
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

    IO_roll_ref = 0;
    IO_pitch_ref = 0;
    IO_yaw_ref = 0;
    
    IO_longitudinal = 0;
    IO_lateral = 0;
    IO_collective = 0;
    IO_esc = 0;
    IO_x_ref = 0;
    IO_y_ref = 0;
}

void IO_Control_Int_Enable(void){
    // clear interrupt flag
    mT3ClearIntFlag();
    // setup timer with pre-scaler = 256 --> a resolution of 3.2e-6s
    OpenTimer3(T3_ON | T3_SOURCE_INT | T3_PS_1_256, IO_CONTROL_PERIOD_MATCH);

    ConfigIntTimer3(T3_INT_ON | T3_INT_PRIOR_1);

    // clear interrupt flag
    mT3ClearIntFlag();
}

void IO_getRadioNominal(void){
    // For simplicity, assume capture is working and can retrieve updated
    // pulsewidth atleast every 25ms
    long int IO_esc_tmp = 0, IO_right_tmp = 0, IO_rear_tmp = 0, IO_rudder_tmp = 0, IO_left_tmp = 0;
    int i = 0;
    int samples[5] = {0};

    // Wait here while radio not detected
    while(IO_RADIO_ERROR);
    IO_delayms(200);

    // try for 100 samples
    for (i = 0; i < 100; i++){
        if (IO_getESC()){
            // non-zero
            IO_esc_tmp += IO_getESC();
            samples[0]++;
        }
        if (IO_getRight()){
            // non-zero
            IO_right_tmp += IO_getRight();
            samples[1]++;
        }
        if (IO_getRear()){
            // non-zero
            IO_rear_tmp += IO_getRear();
            samples[2]++;
        }
        if (IO_getRudder()){
            // non-zero
            IO_rudder_tmp += IO_getRudder();
            samples[3]++;
        }
        if (IO_getLeft()){
            // non-zero
            IO_left_tmp += IO_getLeft();
            samples[4]++;
        }
        IO_delayms(25); // wait 25 ms
    }

    // find average
    IO_esc_nom = IO_esc_tmp / samples[0];
    IO_right_nom = IO_right_tmp / samples[1];
    IO_rear_nom = IO_rear_tmp / samples[2];
    IO_rudder_nom = IO_rudder_tmp / samples[3];
    IO_left_nom = IO_left_tmp / samples[4];

    // sanity check: all norms should be below 2000
    if ((IO_esc_nom > 2000) ||
        (IO_right_nom > 2000) ||
        (IO_rear_nom > 2000) ||
        (IO_rudder_nom > 2000) ||
        (IO_left_nom > 2000)){
        while(1);
        // do not proceed...something is wrong!
    }

}

void IO_changeNotificationSetup(void){
    // Set speed sensor pin and config change notice interrupt
    // D6,CN15: OPTO-ENCODER
    // D7,CN16: GYROGAIN [5]
    // D13,CN19: ULTRASONIC SENSOR PULSEWIDTH
    
    // PGEC1/AN1/CN3/RB1 (24): ESC
    // AN2/C2IN-/CN4/RB2 (23): RIGHT
    // AN3/C2IN+/CN5/RB3 (22): REAR
    // AN4/C1IN-/CN6/RB4 (21): RUDDER
    // AN5/C1IN+/V BUSON /CN7/RB5 (20): LEFT
    
    #if defined(SETUP_RADIO_DETECT)
        PORTSetPinsDigitalIn(IOPORT_D, BIT_6 | BIT_7 | BIT_13);
        PORTSetPinsDigitalIn(IOPORT_B, BIT_1 | BIT_2 | BIT_3 | BIT_4 | BIT_5);
    #else
        PORTSetPinsDigitalIn(IOPORT_D, BIT_6 | BIT_13);
    #endif

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
    
#if defined (DEBUG)
    IO_initializeFile(&IO_dmesg, IO_FS_DMESG);
#endif
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
    // Flush any remaining data before closing files
    IO_flush_FS();  // Flush overflow

    if (IO_log.file_opened){
        if (FSfwrite (IO_log.data_buffer, 1, IO_log.write_length, IO_log.fsFile)){
            IO_log.error = WRITE_ERROR;
        }
    }
    if (IO_dmesg.file_opened){
        if (FSfwrite (IO_dmesg.data_buffer, 1, IO_dmesg.write_length, IO_dmesg.fsFile)){
            IO_dmesg.error = WRITE_ERROR;
        }
    }

    if (IO_log.file_opened){
        if(FSfclose(IO_log.fsFile))
            IO_log.error = CLOSE_ERROR;
        else
            IO_log.file_opened = false;
    }

    if (IO_dmesg.file_opened){
        if(FSfclose(IO_dmesg.fsFile))
            IO_dmesg.error = CLOSE_ERROR;
        else
            IO_dmesg.file_opened = false;
    }
}

bool IO_write_FSwInt(IO_file * io_file, char * data, UINT16 len){
    if (io_file->file_opened){
        // if currently being flushed then write to interrupt buffer
        if (io_file->flush_active){
            UINT16 buffer_remainder = SD_WRITE_BUFFER_SIZE - io_file->int_write_length;
            if (buffer_remainder > len){
                memcpy(&(io_file->interrupt_buffer[io_file->int_write_length]), data, len);
                io_file->int_write_length += len;
                return true;
            }
            else{
                // if delayed data cannot be stored in interrupt buffer then do not bother
                // to store i.e. discard
                return false;
            }
        }

        // if interrupt_buffer was written during flush move data to main buffer
        if (io_file->int_write_length > 0){
            IO_write_FS(io_file, &(io_file->interrupt_buffer[0]), io_file->int_write_length);
            io_file->int_write_length = 0;
        }
        // Then fill in current data
        IO_write_FS(io_file, data, len);
    }
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

bool IO_flush_file(IO_file * io_file){
    io_file->flush_active = true;
    // only if main buffer is full (512), do a write
    if (io_file->buffer_overflow && io_file->file_opened){
        if (FSfwrite (io_file->data_buffer, 1, io_file->write_length, io_file->fsFile) != io_file->write_length){
            io_file->error = WRITE_ERROR;
            io_file->file_opened = false;
            io_file->flush_active = false;
            return false;
        }
        else{
            io_file->buffer_overflow = false;
            io_file->write_length = 0;
            // now move data from overflow to main buffer
            memcpy(&(io_file->data_buffer[0]), &(io_file->overflow_buffer[0]), io_file->overflow_length);
            io_file->write_length = io_file->overflow_length;
            io_file->overflow_length = 0;
            io_file->flush_active = false;
            return true;
        }
    }
    io_file->flush_active = false;
}

bool IO_flush_FS(void){
    int ret = 0;
    ret = IO_flush_file(&IO_log);
    ret = IO_flush_file(&IO_dmesg);
    return ret;
}

void IO_logMessage(char * str, UINT16 len){
    IO_write_FSwInt(&IO_log, str, len);
}

void IO_dmesgMessage(char * str, UINT16 len){
    IO_write_FSwInt(&IO_dmesg, str, len);
}

bool IO_getCloseFiles(void){
    return IO_closeFiles;
}

void IO_setCloseFiles(bool val){
    IO_closeFiles = val;
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

void IO_LED_init(int toggle_number){
    int i = 0;

    for (i = 0; i < toggle_number; i++){
        IO_leds_on(60.0);
        IO_delayms(200);

        IO_setupPWM_default();
        IO_delayms(200);
    }
    
    IO_delayms(1000);
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

void IO_toggle_beacon(void){
    if (IO_LED_ON)
        IO_setupPWM_default();
    else
        IO_leds_on(50);
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

void IO_clrDataTime(void){
    IO_DATA_TIME = 0;
}

UINT32 IO_getDataTime(void){
    return IO_DATA_TIME;
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

bool IO_getSDFlush(void){
    return IO_sdFlush;
}

void IO_setSDFlush(bool val){
    IO_sdFlush = val;
}

bool IO_getRadioError(void){
    return IO_RADIO_ERROR;
}

float IO_getGyroGain_dt(void){
    return (float) IO_gyrogain_dt / (40e3);
}

float IO_getUltrasonic_dt(void){
    return (float) IO_ultrasonic_dt / (40e6);
}

float IO_getAltitude(void){
    static float previous_altitude = 0;

    // 232e3 = 40e6*5.8...e-3(s/m)
    // 16cm -> 928e-6s -> 37120
    // 765cm -> 44.37e-3 -> 1774800

    float current_altitude = (float)IO_ultrasonic_dt/(232000);
    // If there is a sudden change in altitude signal greater than 1 meters
    // assume last calculation is the best estimate
    if (fabs(current_altitude - previous_altitude) < 1){
        previous_altitude = current_altitude;
        return current_altitude;
    }
    else
        return previous_altitude;
}

unsigned short int IO_getRotorSpeed(void){

    // angular_speed    1 rev     60 rev       6 rpm
    //               = ------- = ---------- = ------ , where dt is the period NOT mark time
    //                 dt*10 s    dt*10 min    dt
    if ((IO_rotor_speed_dt != 0) && (!IO_rotor_speed_timeout)){
        IO_rotor_speed = ((float)240e6)/((float)IO_rotor_speed_dt);
        return (unsigned short int) floor(IO_rotor_speed + 0.5);
    }
    return 0;
}

float IO_getFilteredRotorSpeed(void){
    return IO_filteredRotorSpeed;
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

UINT16 IO_getGyroGain(void){
    return (UINT16) (IO_gyrogain_dt / (40));
}

UINT16 IO_getUltrasonic(void){
    return (UINT16) (IO_ultrasonic_dt / (40));
}

UINT16 IO_getESC(void){
    return (UINT16) (IO_esc_dt / (40));
}

UINT16 IO_getRight(void){
    return (UINT16) (IO_right_dt / (40));
}

UINT16 IO_getRear(void){
    return (UINT16) (IO_rear_dt / (40));
}

UINT16 IO_getRudder(void){
    return (UINT16) (IO_rudder_dt / (40));
}

UINT16 IO_getLeft(void){
    return (UINT16) (IO_left_dt / (40));
}

void IO_control_exec(void){
    static UINT32 prev_timestamp = 0;
    UINT16 data_pack[20] = {0};

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
    data_pack[8] = IMU_getRoll16BIT();;
    data_pack[9] = IMU_getPitch16BIT();
    data_pack[10] = IMU_getYaw16BIT();
    data_pack[11] = RN131_getTx();
    data_pack[12] = RN131_getTy();
    data_pack[13] = RN131_getTz();
    data_pack[14] = IO_getRotorSpeed();
    data_pack[15] = 0x0000;
    data_pack[16] = ITG3200_getRawTemperature();
    data_pack[17] = RN131_getIncrKey();
    RN131_SendDataPacket(data_pack, 18);

    int temp_time = ReadCoreTimer();
    float control_dt = (float)((unsigned int)((unsigned int) temp_time - (int) prev_timestamp))/40e6;
    prev_timestamp = temp_time;
    
    IMU_propagateState(control_dt);
    IO_Control();
}

void IO_Control(void){
    static float ref_speed = 0;
    static float internal_speed_ref = 0;
    static float speed_input[3] = {0}, speed_output[3]={0};
    static UINT32 IO_prev_timestamp = 0;
    static UINT32 IO_prev_heave_timestamp = 0;
    static bool reference_above_landing = false;
    static bool landing_routine_active = false;
    static bool semi_automode1 = false;
    static UINT32 height_above_ref_timer = 0;
    static bool takeoff = false;
    char PWM_data[10] = {0};

    // ********************************************************************* //
    // *** BEGIN Motor speed control *************************************** //
    // ********************************************************************* //
    float throttle = (float)IO_getESC();
    if (throttle > 1100)
        internal_speed_ref = internal_speed_ref + 1;
    else
        internal_speed_ref = 0;

    // double rate of increase if speed is greater than 1500 rpm
    if (internal_speed_ref > 1500)
        internal_speed_ref = internal_speed_ref + 2;

    ref_speed = internal_speed_ref;

    // Limit controllable reference
    if (ref_speed < IO_RPM_MIN){
        ref_speed = 0;
    }
    if (ref_speed > IO_RPM_MAX){
        ref_speed = IO_RPM_MAX;
    }

    UINT32 current_time = IO_get_time_ms();
    float IO_dt = (current_time - IO_prev_timestamp)*1e-3;
    IO_prev_timestamp = current_time;

    float current_speed = (float)IO_getRotorSpeed();
    float current_error = (ref_speed - current_speed)*IO_RPM_2_RAD_PER_SEC;
    float upper_limit_int_speed = IO_SPEED_UPPER_LIMIT*IO_SPEED_OMEGA_TI/IO_SPEED_KP;
    IO_esc = IO_PI_with_limits(IO_SPEED_KP, IO_SPEED_OMEGA_TI, IO_dt, current_error,
        &speed_input[0], &speed_output[0], 0, upper_limit_int_speed);
    // ******************************************* END Motor speed control ***//

    int right = (int)SPEKTRUM_getRIGHT() - SPEKTRUM_getRightNominal();
    int rear = -((int)SPEKTRUM_getREAR() - SPEKTRUM_getRearNominal());
    int left = -((int)SPEKTRUM_getLEFT() - SPEKTRUM_getLeftNominal());
    float collective = (right+left+rear)/3;

    // Get roll and pitch reference in degrees
    IO_roll_ref = -(float)(right-left)*IO_ROLL_ANGLE_UPPER_LIMIT/SPEKTRUM_DELTA_LAT_MAX;
    IO_pitch_ref = (float)((right+left)/2-rear)*IO_PITCH_ANGLE_UPPER_LIMIT/SPEKTRUM_DELTA_LONG_MAX;
    // Put limit on angle range in degrees
    IO_roll_ref = IO_limits(IO_ROLL_ANGLE_LOWER_LIMIT, IO_ROLL_ANGLE_UPPER_LIMIT, IO_roll_ref);
    IO_pitch_ref = IO_limits(IO_PITCH_ANGLE_LOWER_LIMIT, IO_PITCH_ANGLE_UPPER_LIMIT, IO_pitch_ref);

    // Get position reference in meters
    IO_x_ref = -(float)((right+left)/2-rear)*IO_Y_UPPER_LIMIT/SPEKTRUM_DELTA_LONG_MAX;
    IO_y_ref = (float)(right-left)*IO_X_UPPER_LIMIT/SPEKTRUM_DELTA_LAT_MAX;
    // Put limit on range
    IO_x_ref = IO_limits(IO_X_LOWER_LIMIT, IO_X_UPPER_LIMIT, IO_x_ref);
    IO_y_ref = IO_limits(IO_Y_LOWER_LIMIT, IO_Y_UPPER_LIMIT, IO_y_ref);

    int right_new = (int)IO_getRight() - SPEKTRUM_getRightNominal();
    int rear_new = -((int)IO_getRear() - SPEKTRUM_getRearNominal());
    int left_new = -((int)IO_getLeft() - SPEKTRUM_getLeftNominal());
    float collective_new = (right_new+left_new+rear_new)/3;
    float lateral = (float)(right_new-left_new)/(2);
    float longitudinal = (float)((right_new+left_new)/2-rear_new)/3;

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
            altitude_ref = IO_landingRefGen(IO_dt, false);
            landing_routine_active = true;
            height_above_ref_timer = 0;
        }
        if (altitude_raw_ref < 0.01){
            reference_above_landing = false;
            IO_landingRefGen(0, true);
            landing_routine_active = false;
        }
    }

    // Pre-filter altitude ref
    float altitude_ref_filtered = IO_filter_altitude_prefilter(altitude_ref, IO_dt, IO_ALTITUDE_LPF_WN);
    altitude_ref = altitude_ref_filtered;
    altitude_ref = IO_limits(0, IO_ALTITUDE_LIMIT, altitude_ref);

    float height_sensor = 0;
    height_sensor = IO_getAltitude();


    if (height_sensor > 0.3){
        takeoff = true;
    }
    IO_z_ref = altitude_ref;
    if (!takeoff){
        if (ultrasonic_updated){
            float IO_heave_dt = (IO_get_time_ms() - IO_prev_heave_timestamp)*1e-3;
            IO_prev_heave_timestamp = IO_get_time_ms();
            IO_collective = IO_heave_control(altitude_ref, height_sensor, IO_TAKEOFF_HEAVE_LOWER_LIMIT, IO_TAKEOFF_HEAVE_UPPER_LIMIT, IO_heave_dt);
            ultrasonic_updated = false;
        }
    }
    else{
        if (ultrasonic_updated){
            float IO_heave_dt = (IO_get_time_ms() - IO_prev_heave_timestamp)*1e-3;
            IO_prev_heave_timestamp = IO_get_time_ms();
            IO_collective = IO_heave_control(altitude_ref, height_sensor, IO_NORMAL_HEAVE_LOWER_LIMIT, IO_NORMAL_HEAVE_UPPER_LIMIT, IO_heave_dt);
            ultrasonic_updated = false;
        }
    }
    // ************************************************* END Heave control ***//

    /*****************/
    /*  Yaw control  */
    /*****************/
    float yaw_angle_ref = -(float)(SPEKTRUM_getRUDDER()-SPEKTRUM_getRudderNominal());
    if (yaw_angle_ref < 0)
        IO_yaw_ref = (float)360/520*yaw_angle_ref;
    else
        IO_yaw_ref = (float)360/270*yaw_angle_ref;
    // Yaw angle pre-filter
    float yaw_ref_filtered = IO_filter_yaw_prefilter(IO_yaw_ref, IO_dt, IO_YAW_LPF_WN);
    IO_yaw_ref = yaw_ref_filtered;
    float yaw_control_sig = IO_yaw_control(IO_dt, yaw_ref_filtered);
    UINT8 yaw_pwm = PWM_PULSEWIDTH2BYTE((int)yaw_control_sig);

    /********************/
    /* Position control */
    /********************/
    // IO_roll_ref and IO_pitch_ref are modified here
//    IO_position_control(IO_x_ref, IO_y_ref, &IO_roll_ref, &IO_pitch_ref, IO_dt, false);
    // Add roll and pitch input bias
    IO_roll_ref += IO_ROLL_BIAS;
    IO_pitch_ref += IO_PITCH_BIAS;
    // Limit roll and pitch angles
    IO_roll_ref = IO_limits(IO_ROLL_ANGLE_LOWER_LIMIT, IO_ROLL_ANGLE_UPPER_LIMIT, IO_roll_ref);
    IO_pitch_ref = IO_limits(IO_PITCH_ANGLE_LOWER_LIMIT, IO_PITCH_ANGLE_UPPER_LIMIT, IO_pitch_ref);

    /*****************/
    /* Atitude control  */
    /*****************/
    IO_lateral = IO_roll_control(IO_roll_ref, IO_dt, false) + IO_LATERAL_BIAS;
    IO_lateral = IO_limits(IO_LATERAL_LOWER_LIMIT, IO_LATERAL_UPPER_LIMIT, IO_lateral);
    IO_longitudinal = IO_pitch_control(IO_pitch_ref, IO_dt, false) + IO_LONGITUDINAL_BIAS;
    IO_longitudinal = IO_limits(IO_LONGITUDINAL_LOWER_LIMIT, IO_LONGITUDINAL_UPPER_LIMIT, IO_longitudinal);
    /*****************/

    IO_collective = collective_new;
    IO_lateral = lateral;
    IO_longitudinal = longitudinal;

    // Cut-off mode
    if (throttle < 1100){
        // set collective to be positive (but not too much) so blade will not
        // hit tail on landing [autorotation terminal phase]
        IO_collective = 100;
    }

    int esc = PWM_PULSEWIDTH2BYTE((int)(throttle));
    int servo_right = PWM_PULSEWIDTH2BYTE((int)(SPEKTRUM_getRightNominal() + IO_lateral + IO_longitudinal + IO_collective));
    int servo_rear = PWM_PULSEWIDTH2BYTE((int)(SPEKTRUM_getRearNominal() + 2*IO_longitudinal - IO_collective));
    int servo_rudder = PWM_PULSEWIDTH2BYTE(IO_getRudder());
    int servo_left = PWM_PULSEWIDTH2BYTE((int)(SPEKTRUM_getLeftNominal() + IO_lateral - IO_longitudinal - IO_collective));

    UINT8 ESC_input = (UINT8) IO_limits(IO_PWM_SAT_MIN, IO_PWM_SAT_MAX, IO_esc);

    PWM_data[0] = ESC_input;
    PWM_data[1] = servo_right;
    PWM_data[2] = servo_rear;
    PWM_data[3] = servo_rudder;//yaw_pwm;
    PWM_data[4] = PWM_PULSEWIDTH2BYTE(SPEKTRUM2PULSEWIDTH(422));
    PWM_data[5] = servo_left;

    bool auto_mode_on = SPEKTRUM_isAutoMode();
    bool ack = PWM_sendPacket(auto_mode_on,PWM_data,6);
    IO_logControl();
//    RN131_logSync();
//    RN131_logData();
//    IO_logSensorData();
//    IO_logRadio();
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
                        IO_yaw_ref,
                        IMU_getYaw(),
                        IO_getBatteryVoltage(),
                        SPEKTRUM_isAutoMode(),
                        SPEKTRUM_getTimeout(),
                        IO_get_time_ms());

    IO_logMessage(&IO_sd_buffer[0], sdlen);
}

void IO_logSensorData(void){
    int sdlen = sprintf(&IO_sd_buffer[0],"%d,%d,%d,%d,%d,%d,%lu\r\n",
                        ITG3200_getx(), ITG3200_gety(), ITG3200_getz(),
                        ADXL345_getx(), ADXL345_gety(), ADXL345_getz(),
                        IO_get_time_ms());

    IO_logMessage(&IO_sd_buffer[0], sdlen);
}

void IO_logControl(void){
    int sdlen = sprintf(&IO_sd_buffer[0],"%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.0f,%.0f,%.0f,%d,%d,%d,%.2f,%.2f,%.2f,%.2f,%u,%u,%lu\r\n",
                        IO_roll_ref*180/M_PI, IO_pitch_ref*180/M_PI, IO_yaw_ref,
                        IMU_getRoll(), IMU_getPitch(), IMU_getYaw(),
                        IO_x_ref*1000, IO_y_ref*1000, IO_z_ref*1000,
                        RN131_getTx(), RN131_getTy(), (short int)(IO_getAltitude()*1000),
                        IO_lateral, IO_longitudinal, IO_collective, IO_esc, 
                        IO_getRotorSpeed(),
                        IO_getBatteryVoltage(),
                        IO_get_time_ms());
    IO_logMessage(&IO_sd_buffer[0], sdlen);
}

void IO_dmesgLog(void){
    int sdlen = sprintf(&IO_sd_buffer[0],"%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.0f,%.0f,%.0f,%d,%d,%d,%.3f,%.3f,%.3f,%lu\r\n",
                        IO_roll_ref*180/M_PI, IO_pitch_ref*180/M_PI, IO_yaw_ref,
                        IMU_getRoll(), IMU_getPitch(), IMU_getYaw(),
                        IO_x_ref*1000, IO_y_ref*1000, IO_z_ref*1000,
                        RN131_getTx(), RN131_getTy(), RN131_getTz(),
                        IO_lateral, IO_longitudinal, IO_collective,
                        IO_get_time_ms());
    IO_dmesgMessage(&IO_sd_buffer[0], sdlen);
}

void IO_generalLog(float a1, float a2, float a3){
    int sdlen = sprintf(&IO_sd_buffer[0],"%.3f,%.3f,%.3f,%lu\r\n",
                a1, a2, a3,IO_get_time_ms());
    IO_logMessage(&IO_sd_buffer[0], sdlen);
}

void IO_logQuaternion(void){
    int sdlen = sprintf(&IO_sd_buffer[0],"%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%lu\r\n",
            IMU_getQuaternion_q0(),IMU_getQuaternion_q1(),IMU_getQuaternion_q2(),IMU_getQuaternion_q3(),
            RN131_getQuaternion_q0(),RN131_getQuaternion_q1(),RN131_getQuaternion_q2(),RN131_getQuaternion_q3(),
            gyro_wx_rad(), gyro_wy_rad(), gyro_wz_rad(),
            IO_get_time_ms());
    IO_logMessage(&IO_sd_buffer[0], sdlen);
}

void IO_logRadio(void){
    int sdlen = sprintf(&IO_sd_buffer[0],"%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%lu\r\n",
                (UINT16)IO_getESC(),
                (UINT16)IO_getRight(),
                (UINT16)IO_getRear(),
                (UINT16)IO_getRudder(),
                (UINT16)IO_getGyroGain(),
                (UINT16)IO_getLeft(),
                SPEKTRUM_getEscRaw(),
                SPEKTRUM_getRightRaw(),
                SPEKTRUM_getRearRaw(),
                SPEKTRUM_getRudderRaw(),
                SPEKTRUM_getGyrogainRaw(),
                SPEKTRUM_getLeftRaw(),
                IO_getRadioError(),
                IO_updateCoreTime_us());
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

float IO_PI_with_limits(const float Kp, const float omega_Ti, const float Ts, const float current_error,
    float filter_input[3], float filter_output[3], const float lower_limit, const float upper_limit){
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

float IO_complex_lead_lag(float wn_z, float damp_z, float wn_p, float damp_p, float Ts, float input[3], float output[3]){
    float Ts_sqrd = pow(Ts,2);
    float wn_z_sqrd = pow(wn_z,2);
    float wn_p_sqrd = pow(wn_p,2);

    float n0 = (Ts_sqrd*wn_p_sqrd*wn_z_sqrd + 4*damp_z*Ts*wn_p_sqrd*wn_z + 4*wn_p_sqrd);
    float n1 = (2*Ts_sqrd*wn_p_sqrd*wn_z_sqrd - 8*wn_p_sqrd);
    float n2 = Ts_sqrd*wn_p_sqrd*wn_z_sqrd - 4*damp_z*Ts*wn_p_sqrd*wn_z + 4*wn_p_sqrd;
    float d0 = (Ts_sqrd*wn_p_sqrd*wn_z_sqrd + 4*damp_p*Ts*wn_p*wn_z_sqrd + 4*wn_z_sqrd);
    float d1 = (2*Ts_sqrd*wn_p_sqrd*wn_z_sqrd - 8*wn_z_sqrd);
    float d2 = Ts_sqrd*wn_p_sqrd*wn_z_sqrd - 4*damp_p*Ts*wn_p*wn_z_sqrd + 4*wn_z_sqrd;

    IO_filter_2nd_order(n0, n1, n2, d0, d1, d2, &input[0], &output[0]);
    return output[0];
}

float IO_filter_speed(float speed_in, float dt, float wn){
    static float speed_input[3] = {0}, speed_output[3] = {0};

    float n0 = (dt*wn);
    float n1 = (dt*wn);
    float d0 = (dt*wn + 2);
    float d1 = (dt*wn - 2);

    IO_shiftData(&speed_input[0]);
    speed_input[0] = speed_in;
    IO_shiftData(&speed_output[0]);

    IO_filter_1st_order(n0, n1, d0, d1, &speed_input[0], &speed_output[0]);
    IO_filteredRotorSpeed = speed_output[0];
    return IO_filteredRotorSpeed;
}

float IO_filter_altitude_prefilter(float alt_in, float dt, float wn){
    static float alt_input[3] = {0}, alt_output[3] = {0};

    float n0 = (dt*wn);
    float n1 = (dt*wn);
    float d0 = (dt*wn + 2);
    float d1 = (dt*wn - 2);

    IO_shiftData(&alt_input[0]);
    alt_input[0] = alt_in;
    IO_shiftData(&alt_output[0]);

    IO_filter_1st_order(n0, n1, d0, d1, &alt_input[0], &alt_output[0]);
    return alt_output[0];
}

float IO_filter_yaw_prefilter(float yaw_in, float dt, float wn){
    static float yaw_input[3] = {0}, yaw_output[3] = {0};

    float n0 = (dt*wn);
    float n1 = (dt*wn);
    float d0 = (dt*wn + 2);
    float d1 = (dt*wn - 2);

    IO_shiftData(&yaw_input[0]);
    yaw_input[0] = yaw_in;
    IO_shiftData(&yaw_output[0]);

    IO_filter_1st_order(n0, n1, d0, d1, &yaw_input[0], &yaw_output[0]);
    return yaw_output[0];
}

float IO_LPF_1st(float input[3], float output[3], float dt, float wn){
    float n0 = (dt*wn);
    float n1 = (dt*wn);
    float d0 = (dt*wn + 2);
    float d1 = (dt*wn - 2);

    IO_filter_1st_order(n0, n1, d0, d1, &input[0], &output[0]);
    return output[0];
}

// Roll reference in rads
float IO_roll_control(float roll_reference, float dt, bool initialise){
    static float notch_input[3] = {0}, notch_output[3] = {0};

    float roll_error = -roll_reference + (IMU_getRoll())*M_PI/180;

    IO_shiftData(&notch_input[0]);
    notch_input[0] = roll_error;
    IO_shiftData(&notch_output[0]);

    if (initialise){
        memset(&notch_input[0],0,sizeof(float)*3);
        memset(&notch_output[0],0,sizeof(float)*3);
        return 0;
    }
    return IO_ROLL_KP*IO_complex_lead_lag(IO_ROLL_NOTCH_WN_Z, IO_ROLL_NOTCH_WZ_DAMP, IO_ROLL_NOTCH_WN_P, IO_ROLL_NOTCH_WP_DAMP, dt, &notch_input[0], &notch_output[0]);
}

// Pitch reference in rads
float IO_pitch_control(float pitch_reference, float dt, bool initialise){
    static float notch_input[3] = {0}, notch_output[3] = {0};

    float pitch_error = pitch_reference - (IMU_getPitch())*M_PI/180;

    IO_shiftData(&notch_input[0]);
    notch_input[0] = pitch_error;
    IO_shiftData(&notch_output[0]);

    if (initialise){
        memset(&notch_input[0],0,sizeof(float)*3);
        memset(&notch_output[0],0,sizeof(float)*3);
        return 0;
    }
    
    return IO_PITCH_KP*IO_complex_lead_lag(IO_PITCH_NOTCH_WN_Z, IO_PITCH_NOTCH_WZ_DAMP, IO_PITCH_NOTCH_WN_P, IO_PITCH_NOTCH_WP_DAMP, dt, &notch_input[0], &notch_output[0]);
}

float IO_heave_control(float heave_reference, float height_sensor, float lower_ctrl_limit, float upper_ctrl_limit, float dt){
    static float heave_error[3] = {0}, heave_input[3] = {0};
    static float int_input[3] = {0}, int_output_[3] = {0};

    IO_shiftData(&heave_error[0]);
    heave_error[0] = heave_reference - height_sensor;
    IO_shiftData(&heave_input[0]);

    // calculate derivative control signal
    float derivative_signal = IO_HEAVE_KP*IO_lead_lag(IO_HEAVE_OMEGA_LEAD, IO_HEAVE_OMEGA_LAG, dt, &heave_error[0], &heave_input[0]);
    float lower_limit_int = lower_ctrl_limit*IO_HEAVE_OMEGA_TI;
    float upper_limit_int = upper_ctrl_limit*IO_HEAVE_OMEGA_TI;

    return IO_PI_with_limits(1, IO_HEAVE_OMEGA_TI, dt, derivative_signal,
        &int_input[0], &int_output_[0], lower_limit_int, upper_limit_int);
}

float IO_yaw_control(float dt, float yaw_ref){
    static float yaw_error[3] = {0}, yaw_input[3] = {0};
    static float int_input[3] = {0}, int_output_[3] = {0};

    IO_shiftData(&yaw_error[0]);
    
    yaw_error[0] = (yaw_ref - IMU_getYaw())*M_PI/180;
    IO_shiftData(&yaw_input[0]);

    // calculate derivative control signal
    float derivative_signal = IO_YAW_KP*IO_lead_lag(IO_YAW_OMEGA_LEAD, IO_YAW_OMEGA_LAG, dt, &yaw_error[0], &yaw_input[0]);
    float lower_limit_int = IO_YAW_LOWER_LIMIT*IO_YAW_OMEGA_TI;
    float upper_limit_int = IO_YAW_UPPER_LIMIT*IO_YAW_OMEGA_TI;

    float yaw_control_sig = IO_PI_with_limits(1, IO_YAW_OMEGA_TI, dt, derivative_signal,
        &int_input[0], &int_output_[0], lower_limit_int, upper_limit_int);

    return (SPEKTRUM_getRudderNominal() - yaw_control_sig);
}

void IO_position_control(float x_ref, float y_ref, float * roll_cmd, float * pitch_cmd, float dt, bool initialise){
    static float error_x[3] = {0}, pitch_x[3] = {0};
    static float error_y[3] = {0}, roll_y[3] = {0};
    static float error_x2[3] = {0}, pitch_x2[3] = {0};
    static float error_y2[3] = {0}, roll_y2[3] = {0};
    static float x_int_input[3] = {0}, x_int_output[3] = {0};
    static float y_int_input[3] = {0}, y_int_output[3] = {0};
    static float x_lpf_input[3] = {0}, x_lpf_output[3] = {0};
    static float y_lpf_input[3] = {0}, y_lpf_output[3] = {0};

    float position_err_body[3] = {0};
    float position_err_inertial[3] = {x_ref - RN131_get_tx(),y_ref - RN131_get_ty(),0};

    // use conjugate of quaternion to find body frame errors
    float q_conjugate[4] = {IMU_getQuaternion_q0(),-IMU_getQuaternion_q1(),-IMU_getQuaternion_q2(),-IMU_getQuaternion_q3()};
    Reb(&q_conjugate[0], &position_err_inertial[0], &position_err_body[0]);

    IO_shiftData(&error_x[0]);
    error_x[0] = position_err_body[0];
    IO_shiftData(&pitch_x[0]);

    IO_shiftData(&error_y[0]);
    error_y[0] = position_err_body[1];
    IO_shiftData(&roll_y[0]);

    if (initialise){
        memset(&error_x[0],0,sizeof(float)*3);
        memset(&pitch_x[0],0,sizeof(float)*3);
        memset(&error_y[0],0,sizeof(float)*3);
        memset(&roll_y[0],0,sizeof(float)*3);

        memset(&error_x2[0],0,sizeof(float)*3);
        memset(&pitch_x2[0],0,sizeof(float)*3);
        memset(&error_y2[0],0,sizeof(float)*3);
        memset(&roll_y2[0],0,sizeof(float)*3);

        memset(&x_int_input[0],0,sizeof(float)*3);
        memset(&x_int_output[0],0,sizeof(float)*3);
        memset(&y_int_input[0],0,sizeof(float)*3);
        memset(&y_int_output[0],0,sizeof(float)*3);

        memset(&x_lpf_input[0],0,sizeof(float)*3);
        memset(&x_lpf_output[0],0,sizeof(float)*3);
        memset(&y_lpf_input[0],0,sizeof(float)*3);
        memset(&y_lpf_output[0],0,sizeof(float)*3);

        *pitch_cmd = 0;
        *roll_cmd = 0;
        return;
    }

    // Calculate lead output
    float pitch_lead = IO_X_KP*IO_lead_lag(IO_X_OMEGA_LEAD_1, IO_X_OMEGA_LAG_1, dt, &error_x[0], &pitch_x[0]);
    float roll_lead = IO_Y_KP*IO_lead_lag(IO_Y_OMEGA_LEAD_1, IO_Y_OMEGA_LAG_1, dt, &error_y[0], &roll_y[0]);

    // add further lpf for sensor noise attenuation
    IO_shiftData(&x_lpf_input[0]);
    x_lpf_input[0] = pitch_lead;
    IO_shiftData(&x_lpf_output[0]);

    IO_shiftData(&y_lpf_input[0]);
    y_lpf_input[0] = roll_lead;
    IO_shiftData(&y_lpf_output[0]);

    pitch_lead = IO_LPF_1st(&x_lpf_input[0], &x_lpf_output[0], dt, IO_X_LPF_WN);
    roll_lead = IO_LPF_1st(&y_lpf_input[0], &y_lpf_output[0], dt, IO_Y_LPF_WN);

//    IO_shiftData(&error_x2[0]);
//    error_x2[0] = pitch_lead;
//    IO_shiftData(&pitch_x2[0]);
//
//    IO_shiftData(&error_y2[0]);
//    error_y2[0] = roll_lead;
//    IO_shiftData(&roll_y2[0]);
//
//    pitch_lead = IO_lead_lag(IO_X_OMEGA_LEAD_2, IO_X_OMEGA_LAG_2, dt, &error_x2[0], &pitch_x2[0]);
//    roll_lead = IO_lead_lag(IO_Y_OMEGA_LEAD_2, IO_Y_OMEGA_LAG_2, dt, &error_y2[0], &roll_y2[0]);

    float pitch_lower_limit_int = IO_PITCH_ANGLE_LOWER_LIMIT*IO_X_OMEGA_TI;
    float pitch_upper_limit_int = IO_PITCH_ANGLE_UPPER_LIMIT*IO_X_OMEGA_TI;
    float roll_lower_limit_int = IO_ROLL_ANGLE_LOWER_LIMIT*IO_Y_OMEGA_TI;
    float roll_upper_limit_int = IO_ROLL_ANGLE_UPPER_LIMIT*IO_Y_OMEGA_TI;

    *pitch_cmd = IO_PI_with_limits(1, IO_X_OMEGA_TI, dt, pitch_lead,
        &x_int_input[0], &x_int_output[0], pitch_lower_limit_int, pitch_upper_limit_int);
    *roll_cmd = IO_PI_with_limits(1, IO_Y_OMEGA_TI, dt, roll_lead,
        &y_int_input[0], &y_int_output[0], roll_lower_limit_int, roll_upper_limit_int);
}


void IO_shiftData(float data[3]){
    data[2] = data[1];
    data[1] = data[0];
}

// When most accurate current time is required, call this function
// Resolution of 1 microseconds
// Maximum count equivalent to 71 minutes
volatile UINT32 IO_updateCoreTime_us(void){
    static UINT32 previous_time = 0;

    // If this function is running by some process do not interrupt
    if (!IO_coreupdate_mutex){
        IO_coreupdate_mutex = true;
        UINT32 current_time = ReadCoreTimer();
        UINT32 dt = (UINT32)((UINT32)current_time - (INT32)previous_time);
        previous_time = current_time;
        IO_time_raw += dt;
        IO_coreupdate_mutex = false;
    }
    
    return (UINT32)(IO_time_raw / 40);
}

volatile UINT32 IO_getTime_us(void){
    return (UINT32)(IO_time_raw / 40);
}


// Used by RN131 to offset initial estimate
void IO_adjIO_time_us(INT32 offset_us){
    IO_time_raw = IO_time_raw + (INT64)offset_us*40;
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

    // Must be called regularly to keep time updated (atleast every 107s)
    IO_updateCoreTime_us();

    IO_time_ms++;
    IO_DATA_TIME++;

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

    if ((IO_time_ms % IO_SD_FLUSH) == 0){
        IO_sdFlush = true;
    }

    SPEKTRUM_timeoutInc();
    RN131_timeoutInc();

    // When rotor does not get updated, speed should be set to zero
    if (IO_timeoutSpeed()){
        IO_rotor_speed = 0;
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
    if (!PORTReadBits(IOPORT_A, BIT_1)){
        IO_closeFiles = true;
    }
}

/*
 * This interrupt is used for measuring motor speed
 * PRIORITY: 7
 * SUBPRIORITY: 3
 */
void __ISR(_CHANGE_NOTICE_VECTOR, IPL7SRS) changeNotificationHandler(void)
{
    static int prev_time_stamp[10] = {0};
    static bool first_run[10] = {true};
    static bool pin_prev_state[10] = {low};

    int temp_time;

    // clear the mismatch condition
    unsigned int port_val_D = mPORTDRead();
    unsigned int port_val_B = mPORTBRead();

    // clear the interrupt flag
    mCNClearIntFlag();

#if defined(SETUP_RADIO_DETECT)
    // Measuring the period of the pulse...not the mark time
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

    if ((port_val_D & IO_GYROGAIN_PIND) && (pin_prev_state[1] == low))
    {
        pin_prev_state[1] = high;
        prev_time_stamp[1] = ReadCoreTimer();
    }
    if(!(port_val_D & IO_GYROGAIN_PIND) && (pin_prev_state[1] == high))
    {
        temp_time = ReadCoreTimer();
        IO_gyrogain_dt = (unsigned int) ReadCoreTimer() - (int) prev_time_stamp[1];
        prev_time_stamp[1] = temp_time;
        pin_prev_state[1] = low;

        if ((IO_gyrogain_dt > CN_PWM_MAX) || (IO_gyrogain_dt < CN_PWM_MIN)){
            IO_valid_PWM_data &= 0b11101111;
        }
        else{
            IO_valid_PWM_data |= 0b00010000;
        }
    }

    if ((port_val_D & IO_ULTRASONIC_PIND) && (pin_prev_state[2] == low))
    {
        pin_prev_state[2] = high;
        prev_time_stamp[2] = ReadCoreTimer();
    }
    if(!(port_val_D & IO_ULTRASONIC_PIND) && (pin_prev_state[2] == high))
    {
        temp_time = ReadCoreTimer();
        IO_ultrasonic_dt = (unsigned int) ReadCoreTimer() - (int) prev_time_stamp[2];
        prev_time_stamp[2] = temp_time;
        pin_prev_state[2] = low;

        ultrasonic_updated = true;

        if (IO_ultrasonic_dt > IO_ULTRASONIC_DT_MAX)
            IO_ultrasonic_dt = IO_ULTRASONIC_DT_MAX;
    }

    if ((port_val_B & IO_ESC_PINB) && (pin_prev_state[3] == low))
    {
        pin_prev_state[3] = high;
        prev_time_stamp[3] = ReadCoreTimer();
    }
    if(!(port_val_B & IO_ESC_PINB) && (pin_prev_state[3] == high))
    {
        temp_time = ReadCoreTimer();
        IO_esc_dt = (unsigned int) ReadCoreTimer() - (int) prev_time_stamp[3];
        prev_time_stamp[3] = temp_time;
        pin_prev_state[3] = low;

        if ((IO_esc_dt > CN_PWM_MAX) || (IO_esc_dt < CN_PWM_MIN)){
            IO_valid_PWM_data &= 0b11111110;
        }
        else{
            IO_valid_PWM_data |= 0b00000001;
        }
    }

    if ((port_val_B & IO_RIGHT_PINB) && (pin_prev_state[4] == low))
    {
        pin_prev_state[4] = high;
        prev_time_stamp[4] = ReadCoreTimer();
    }
    if(!(port_val_B & IO_RIGHT_PINB) && (pin_prev_state[4] == high))
    {
        temp_time = ReadCoreTimer();
        IO_right_dt = (unsigned int) ReadCoreTimer() - (int) prev_time_stamp[4];
        prev_time_stamp[4] = temp_time;
        pin_prev_state[4] = low;

        if ((IO_right_dt > CN_PWM_MAX) || (IO_right_dt < CN_PWM_MIN)){
            IO_valid_PWM_data &= 0b11111101;
        }
        else{
            IO_valid_PWM_data |= 0b00000010;
        }
    }

    if ((port_val_B & IO_REAR_PINB) && (pin_prev_state[5] == low))
    {
        pin_prev_state[5] = high;
        prev_time_stamp[5] = ReadCoreTimer();
    }
    if(!(port_val_B & IO_REAR_PINB) && (pin_prev_state[5] == high))
    {
        temp_time = ReadCoreTimer();
        IO_rear_dt = (unsigned int) ReadCoreTimer() - (int) prev_time_stamp[5];
        prev_time_stamp[5] = temp_time;
        pin_prev_state[5] = low;

        if ((IO_rear_dt > CN_PWM_MAX) || (IO_rear_dt < CN_PWM_MIN)){
            IO_valid_PWM_data &= 0b11111011;
        }
        else{
            IO_valid_PWM_data |= 0b00000100;
        }
    }

    if ((port_val_B & IO_RUDDER_PINB) && (pin_prev_state[6] == low))
    {
        pin_prev_state[6] = high;
        prev_time_stamp[6] = ReadCoreTimer();
    }
    if(!(port_val_B & IO_RUDDER_PINB) && (pin_prev_state[6] == high))
    {
        temp_time = ReadCoreTimer();
        IO_rudder_dt = (unsigned int) ReadCoreTimer() - (int) prev_time_stamp[6];
        prev_time_stamp[6] = temp_time;
        pin_prev_state[6] = low;

        if ((IO_rudder_dt > CN_PWM_MAX) || (IO_rudder_dt < CN_PWM_MIN)){
            IO_valid_PWM_data &= 0b11110111;
        }
        else{
            IO_valid_PWM_data |= 0b00001000;
        }
    }

    if ((port_val_B & IO_LEFT_PINB) && (pin_prev_state[7] == low))
    {
        pin_prev_state[7] = high;
        prev_time_stamp[7] = ReadCoreTimer();
    }
    if(!(port_val_B & IO_LEFT_PINB) && (pin_prev_state[7] == high))
    {
        temp_time = ReadCoreTimer();
        IO_left_dt = (unsigned int) ReadCoreTimer() - (int) prev_time_stamp[7];
        prev_time_stamp[7] = temp_time;
        pin_prev_state[7] = low;

        if ((IO_left_dt > CN_PWM_MAX) || (IO_left_dt < CN_PWM_MIN)){
            IO_valid_PWM_data &= 0b11011111;
        }
        else{
            IO_valid_PWM_data |= 0b00100000;
        }
    }

    // if any pin is invalid --> radio error true
    if (IO_valid_PWM_data == 0b00111111){
        IO_RADIO_ERROR = false;
    }
    else{
        IO_RADIO_ERROR = true;
    }
    
#else
    // Measuring the period of the pulse...not the mark time
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

    if ((port_val_D & IO_ULTRASONIC_PIND) && (pin_prev_state[2] == low))
    {
        pin_prev_state[2] = high;
        prev_time_stamp[2] = ReadCoreTimer();
    }
    if(!(port_val_D & IO_ULTRASONIC_PIND) && (pin_prev_state[2] == high))
    {
        temp_time = ReadCoreTimer();
        IO_ultrasonic_dt = (unsigned int) ReadCoreTimer() - (int) prev_time_stamp[2];
        prev_time_stamp[2] = temp_time;
        pin_prev_state[2] = low;
    }
#endif

}

// Set control loop as lowest priority
void __ISR(_TIMER_3_VECTOR, ipl1) _Timer3Handler(void){
    IO_control_exec();
    // clear the interrupt flag
    mT3ClearIntFlag();

}