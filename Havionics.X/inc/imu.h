#ifndef IMU_H
#define	IMU_H

#ifdef IMU_H_IMPORT
    #define IMU_EXTERN
#else
	#define IMU_EXTERN extern
#endif

#include "../inc/common.h"
#include "../inc/itg3200.h"
#include "../inc/adxl345.h"
#include "../inc/rn131.h"
#include "../inc/io.h"

#define KP_DRIFT 1.8
#define KI_DRIFT 10e-3

#define TO_RAD_PER_SEC(x)   (x*M_PI/180)

#define IMU_OMEGA_KP      (0.01)
#define IMU_OMEGA_TI      (0.01*2*M_PI)
#define IMU_BIAS_UPPER_LIMIT    (1)   // (rad/s)
#define IMU_BIAS_LOWER_LIMIT    (-1)

float gravity_accel[3];
float gravity_estimate[3];

IMU_EXTERN void IMU_openI2C(void);
IMU_EXTERN void IMU_writeI2C1(uchar reg, uchar data, uchar ADDRESS);
IMU_EXTERN uchar IMU_readI2C1(uchar reg, uchar ADDRESS);
IMU_EXTERN uchar IMU_writeI2C1Read(uchar reg, uchar data, uchar ADDRESS);
IMU_EXTERN bool IMU_tryConfig(uchar reg, uchar data, uchar ADDRESS);

IMU_EXTERN void IMU_propagateState(float dt);
IMU_EXTERN void IMU_initializeQuaternion(float * quat);
IMU_EXTERN void IMU_quaternion2euler(float q[4], float * roll, float * pitch, float * yaw);
IMU_EXTERN float IMU_getRoll(void);
IMU_EXTERN float IMU_getPitch(void);
IMU_EXTERN float IMU_getYaw(void);
IMU_EXTERN void IMU_quaternionRotate(const float q1[4], const float q2[4], float q[4]);
IMU_EXTERN void IMU_correctGyro(float dt);
IMU_EXTERN void IMU_normalizeVector(float array[], int len);
IMU_EXTERN void IMU_quaternion_error(const float p[4], const float q[4], float q_err[4]);
IMU_EXTERN float * IMU_getQuaternion(void);
IMU_EXTERN void IMU_rotateXby180(float q_in[4], float q_out[4]);

IMU_EXTERN INT16 IMU_getRoll16BIT(void);
IMU_EXTERN INT16 IMU_getPitch16BIT(void);
IMU_EXTERN INT16 IMU_getYaw16BIT(void);
#endif	/* IMU_H */

