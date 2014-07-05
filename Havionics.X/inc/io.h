
#ifndef IO_H
#define	IO_H

#include <plib.h>
#include <stdbool.h>
#include <string.h>
#include "HardwareProfile.h"
#include "GenericTypeDefs.h"
#include <math.h>
#include "MDD File System/FSIO.h"
#include "spektrumRX.h"

#ifdef IO_H_IMPORT
	#define IO_EXTERN
#else
	#define IO_EXTERN extern
#endif

typedef enum{
            BOOT,
            IDLE,
            OKAY,
            ERROR,
            FILE_CLOSED,
            MANUAL
} IO_SYSTEM_STATE;

typedef enum{
            NO_ERROR,
            WRITE_ERROR,
            READ_ERROR,
            OPEN_ERROR,
            CLOSE_ERROR
} IO_FILE_ERROR;

#define SD_WRITE_BUFFER_SIZE 512
typedef struct {
    int file_id;
    bool file_opened;
    bool close_file;
    FSFILE * fsFile;
    char file_name[20];
    char data_buffer[SD_WRITE_BUFFER_SIZE];
    char overflow_buffer[256];
    UINT16 overflow_length;
    bool buffer_overflow;
    UINT16 write_length;
    unsigned int data_buffer_len;
    UINT64 time;
    IO_FILE_ERROR error;
} IO_file;

#define IO_OUTPUT_TRIS              0
#define IO_INPUT_TRIS               1

#define IO_HEARTBEAT_TRIS()         PORTSetPinsDigitalOut(IOPORT_C, BIT_1)
#define IO_TOGGLE_HEARTBEAT()       PORTToggleBits(IOPORT_C, BIT_1)
#define IO_SET_HEARTBEAT()          PORTSetBits(IOPORT_C, BIT_1)
#define IO_CLEAR_HEARTBEAT()        PORTClearBits(IOPORT_C, BIT_1)
#define IO_PROGRAM_SWITCH_TRIS()    PORTSetPinsDigitalIn(IOPORT_A, BIT_1)
#define IO_DEBUG_TRIS()             PORTSetPinsDigitalOut(IOPORT_E, BIT_7)
#define IO_LOW                      0
#define IO_HIGH                     1
#define IO_DEBUG_LAT_LOW()          PORTClearBits(IOPORT_E, BIT_7)
#define IO_DEBUG_LAT_HIGH()         PORTSetBits(IOPORT_E, BIT_7)
#define IO_DEBUG_TOGGLE()           PORTToggleBits(IOPORT_E, BIT_7)
#define IO_LEVELSHIFT1TRIS()        PORTSetPinsDigitalOut(IOPORT_A, BIT_4)
#define IO_LEVELSHIFT2TRIS()        PORTSetPinsDigitalOut(IOPORT_A, BIT_5)

#define IO_PWM_PERIOD               (500)

#define TOGGLE_PER_SEC              (1000)
#define CORE_TICK_RATE              (GetSystemClock()/2/TOGGLE_PER_SEC)

#define IO_ERROR_BLINK              (1000)

#define IO_ADC_READ                 (100)
#define IO_SD_FLUSH                 (50)
#define IO_DATA_PERIOD              (20)
#define IO_STATE_PROJECT            (20)
#define IO_RX_TIMEOUT_CHECK         (5)

#define IO_SPEED_TIMEOUT            (100)

#define IO_RPM_MAX                  (3500)
#define IO_RPM_MIN                  (500)
#define IO_PWM_SAT_MAX              (0xFF)
#define IO_PWM_SAT_MIN              (0x00)

// RADIO input capture pins
#define IO_OPTO_ENCODER_PIND           (0x40)
#define IO_GYROGAIN_PIND               (0x80)
#define IO_ULTRASONIC_PIND             (0x2000)

// PGEC1/AN1/CN3/RB1 (24): ESC
// AN2/C2IN-/CN4/RB2 (23): RIGHT
// AN3/C2IN+/CN5/RB3 (22): REAR
// AN4/C1IN-/CN6/RB4 (21): RUDDER
// AN5/C1IN+/V BUSON /CN7/RB5 (20): LEFT
#define IO_ESC_PINB                    (0x02)
#define IO_RIGHT_PINB                  (0x04)
#define IO_REAR_PINB                   (0x08)
#define IO_RUDDER_PINB                 (0x10)
#define IO_LEFT_PINB                   (0x20)

#define CN_PWM_MAX                      (84000) // equivalent to 2.2ms
#define CN_PWM_MIN                      (32000) // equivalent to 0.8ms

#define IO_ULTRASONIC_DT_MAX            (1774800)
#define IO_ULTRASONIC_DT_MIN            (37120)

#define IO_RPM_2_RAD_PER_SEC        (2*M_PI/60)

// File System parameters
#define IO_FS_DMESG "dmesg.txt"
#define IO_FS_LOG "log.txt"

typedef enum IO_PID_TYPE_t{
    IO_PID,
    IO_PD
} IO_PID_TYPE;

IO_EXTERN void IO_setup(void);
IO_EXTERN void IO_setupPWM_default(void);
IO_EXTERN UINT16 IO_PWM_dc(float dc);
IO_EXTERN IO_SYSTEM_STATE IO_getSystemState(void);
IO_EXTERN void IO_setSystemState(IO_SYSTEM_STATE state);
IO_EXTERN UINT16 IO_getBatteryVoltage(void);
IO_EXTERN void IO_delayms(unsigned int num_ms_delay);
IO_EXTERN void IO_initialize_FS(void);
IO_EXTERN bool IO_initializeFile(IO_file * IO_file_ds, const char * IO_file_name);
IO_EXTERN void IO_terminate_FS(void);
IO_EXTERN bool IO_flush_FS(void);
IO_EXTERN void IO_logMessage(char * str, UINT16 len);
IO_EXTERN bool IO_getSDFlush(void);
IO_EXTERN void IO_setSDFlush(bool val);
IO_EXTERN bool IO_getCloseFiles(void);
IO_EXTERN void IO_setCloseFiles(bool val);
IO_EXTERN void IO_LED_init(void);
IO_EXTERN void IO_leds_on(float power);
IO_EXTERN volatile UINT32 IO_get_time_ms(void);
IO_EXTERN void IO_setSendDataTime(UINT16 delta_t);
IO_EXTERN void IO_set_change_LED(bool val);
IO_EXTERN bool IO_get_change_LED(void);
IO_EXTERN bool IO_getSendData(void);
IO_EXTERN void IO_setSendData(bool val);
IO_EXTERN bool IO_getLocalControl(void);
IO_EXTERN void IO_setLocalControl(bool val);
IO_EXTERN void IO_changeNotificationSetup(void);
IO_EXTERN void IO_logSensorData(void);
IO_EXTERN void IO_generalLog(float a1, float a2, float a3);
IO_EXTERN void IO_logRadio(void);
IO_EXTERN void IO_logControl(void);
IO_EXTERN void IO_logQuaternion(void);
IO_EXTERN bool IO_getRadioError(void);
IO_EXTERN bool IO_getLEDState(void);
IO_EXTERN volatile UINT32 IO_getLEDON_time(void);
IO_EXTERN void IO_Control(void);

IO_EXTERN float IO_getGyroGain_dt(void);
IO_EXTERN float IO_getUltrasonic_dt(void);
IO_EXTERN unsigned short int IO_getRotorSpeed(void);
IO_EXTERN float IO_getFilteredRotorSpeed(void);
IO_EXTERN  char IO_limit_PWM(int sig);

IO_EXTERN UINT16 IO_getGyroGain(void);
IO_EXTERN UINT16 IO_getUltrasonic(void);
IO_EXTERN UINT16 IO_getESC(void);
IO_EXTERN UINT16 IO_getRight(void);
IO_EXTERN UINT16 IO_getRear(void);
IO_EXTERN UINT16 IO_getRudder(void);
IO_EXTERN UINT16 IO_getLeft(void);
IO_EXTERN float IO_getAltitude(void);

IO_EXTERN float IO_limits(float lower, float upper, float input);
IO_EXTERN void IO_filter_1st_order(float n1, float n2, float d1, float d2, float input[3], float output[3]);
IO_EXTERN void IO_filter_2nd_order(float n0, float n1, float n2, float d0, float d1, float d2, float input[3], float output[3]);
IO_EXTERN float IO_filter_notch(float wn, float damp_p, float damp_z, float Ts, float input[3], float output[3]);
IO_EXTERN float IO_PI_with_limits(const float Kp, const float omega_Ti, const float Ts, const float current_error,
    float filter_input[3], float filter_output[3], const float lower_limit, const float upper_limit);
IO_EXTERN float IO_lead_lag(float omega_lead, float omega_lag, float Ts, float filter_input[3], float filter_output[3]);
IO_EXTERN float IO_complex_lead_lag(float wn_z, float damp_z, float wn_p, float damp_p, float Ts, float input[3], float output[3]);
IO_EXTERN float IO_heave_control(float heave_reference, float height_sensor, float lower_ctrl_limit, float upper_ctrl_limit, float dt);
IO_EXTERN float IO_yaw_control(float dt, float yaw_ref);
IO_EXTERN float IO_LPF_1st(float input[3], float output[3], float dt, float wn);

IO_EXTERN void IO_PIDComputation(IO_PID_TYPE pidtype, float Kp, float omega_Td, float omega_Ti, float alpha, float Ts,float u[3], float e[3]);
IO_EXTERN void IO_PID_with_limits(float Kp, float omega_lead, float omega_Ti, float alpha, float Ts,
        float int_inputs[3], float int_outputs[3], float current_error,
        float int_lower_limit, float int_upper_limit,
        float diff_inputs[3], float diff_outputs[3]);
IO_EXTERN void IO_PID_rateAux(IO_PID_TYPE pidtype, float Kp, float omega_Td, float omega_Ti, float alpha, float Ts,float u[3], float e[3]);
IO_EXTERN float IO_roll_control(float roll_reference, float dt, bool initialise);
IO_EXTERN float IO_pitch_control(float pitch_reference, float dt, bool initialise);
IO_EXTERN void IO_position_control(float x_ref, float y_ref, float * roll_cmd, float * pitch_cmd, float dt, bool initialise);
IO_EXTERN void IO_shiftData(float data[3]);
IO_EXTERN float IO_landingRefGen(float dt, bool reset);
IO_EXTERN float IO_filter_speed(float speed_in, float dt, float wn);
IO_EXTERN float IO_filter_altitude_prefilter(float alt_in, float dt, float wn);
IO_EXTERN float IO_filter_yaw_prefilter(float yaw_in, float dt, float wn);

IO_EXTERN bool IO_getStateProject(void);
IO_EXTERN void IO_setStateProject(bool val);

IO_EXTERN void IO_logVital(bool ack, float motor_speed, float altitude_ref,
        float altitude_sense);

float IO_longitudinal;
float IO_lateral;
float IO_collective;
float IO_roll_ref;
float IO_pitch_ref;
float IO_yaw_ref;
float IO_x_ref;
float IO_y_ref;
float IO_z_ref;

#endif	/* IO_H */

