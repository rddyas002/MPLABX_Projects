#define IMU_H_IMPORT

#include "imu.h"

//#define gyro_wx_rad angular_rate[0]
//#define gyro_wy_rad angular_rate[1]
//#define gyro_wz_rad angular_rate[2]

#define gyro_wx_rad (itg3200_data.x_float*M_PI/180)
#define gyro_wy_rad (itg3200_data.y_float*M_PI/180)
#define gyro_wz_rad (itg3200_data.z_float*M_PI/180)

bool IMU_i2cOpen = false;

float IMU_roll = 0;
float IMU_pitch = 0;
float IMU_yaw = 0;

static float angular_rate[3] = {0};

// attitude estimation
float IMU_q[4] = {0,1,0,0};

void IMU_openI2C(void){
    if (!IMU_i2cOpen){
        // Use I2C1 & set baud rate to 400kHz
        OpenI2C1( I2C_EN | I2C_SLW_EN , I2C1_BAUD );
        IMU_i2cOpen = true;
    }
}

bool IMU_tryConfig(uchar reg, uchar data, uchar ADDRESS){
    uchar reg_data = 0, counter = 0;

    while(counter++ < IMU_TRYCONFIG){
        reg_data =  IMU_writeI2C1Read(reg, data, ADDRESS);
        if (reg_data == data)
            return true;
    }

    return false;
}

uchar IMU_writeI2C1Read(uchar reg, uchar data, uchar ADDRESS){
    IMU_writeI2C1(reg, data, ADDRESS);
    return IMU_readI2C1(reg, ADDRESS);
}

void IMU_writeI2C1(uchar reg, uchar data, uchar ADDRESS){
    unsigned char WRITE_ADDRESS;
    WRITE_ADDRESS = (ADDRESS << 1);

    StartI2C1();
    IdleI2C1();

    MasterWriteI2C1(WRITE_ADDRESS);
    IdleI2C1();

    MasterWriteI2C1(reg);
    IdleI2C1();

    MasterWriteI2C1(data);
    IdleI2C1();

    StopI2C1();
    IdleI2C1();
}

uchar IMU_readI2C1(uchar reg, uchar ADDRESS){
    uchar WRITE_ADDRESS, READ_ADDRESS, data;
    WRITE_ADDRESS = (ADDRESS << 1);
    READ_ADDRESS = ((ADDRESS << 1) | 0x01);

    StartI2C1();
    IdleI2C1();

    MasterWriteI2C1(WRITE_ADDRESS);
    IdleI2C1();

    MasterWriteI2C1(reg);
    IdleI2C1();

    RestartI2C1();
    IdleI2C1();

    MasterWriteI2C1(READ_ADDRESS);
    IdleI2C1();

    data = MasterReadI2C1();
    IdleI2C1();

    NotAckI2C1();
    IdleI2C1();

    StopI2C1();
    IdleI2C1();

    return data;
}

void IMU_initializeQuaternion(float * quat){
    memcpy(&IMU_q[0], quat,4*sizeof(float));
}

void IMU_propagateState(float dt){
    static int normalize_counter = 0;
    float q_next[4] = {0};

    q_next[0] = IMU_q[0] - dt*0.5*(IMU_q[1]*gyro_wx_rad + IMU_q[2]*gyro_wy_rad + IMU_q[3]*gyro_wz_rad);
    q_next[1] = IMU_q[1] + dt*0.5*(IMU_q[0]*gyro_wx_rad - IMU_q[3]*gyro_wy_rad + IMU_q[2]*gyro_wz_rad);
    q_next[2] = IMU_q[2] + dt*0.5*(IMU_q[3]*gyro_wx_rad + IMU_q[0]*gyro_wy_rad - IMU_q[1]*gyro_wz_rad);
    q_next[3] = IMU_q[3] - dt*0.5*(IMU_q[2]*gyro_wx_rad - IMU_q[1]*gyro_wy_rad - IMU_q[0]*gyro_wz_rad);

    memcpy(&IMU_q[0], &q_next[0],4*sizeof(float));

    if (normalize_counter++ == 10){
        IMU_normalizeVector(IMU_q,4);
        normalize_counter = 0;
    }

    float flip[4] = {0,1,0,0};
    float quat_ret[4];
    IMU_quaternionRotate(&IMU_q[0], &flip[0], &quat_ret[0]);
    IMU_quaternion2euler(quat_ret, &IMU_roll, &IMU_pitch, &IMU_yaw);
}

void IMU_quaternion2euler(float q[4], float * roll, float * pitch, float * yaw){
  *roll = atan2(2.0 * (q[0] * q[1] + q[2] * q[3]), 1.0 - 2.0 *
                        (pow(q[1], 2.0) + pow(q[2], 2.0))) * 180.0 / 3.1415926535897931;
  *pitch = asin(2.0 * (q[0] * q[2] - q[3] * q[1])) * 180.0 / 3.1415926535897931;
  *yaw = atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 *
                       (pow(q[2], 2.0) + pow(q[3], 2.0))) * 180.0 / 3.1415926535897931;
}

void IMU_quaternionRotate(const float q1[4], const float q2[4], float q[4]){
  float b_q2[16];
  int i9;
  int i10;

  /*  rotation q1 by q2 */
  b_q2[0] = q2[0];
  b_q2[4] = -q2[1];
  b_q2[8] = -q2[2];
  b_q2[12] = -q2[3];
  b_q2[1] = q2[1];
  b_q2[5] = q2[0];
  b_q2[9] = q2[3];
  b_q2[13] = q2[2];
  b_q2[2] = q2[2];
  b_q2[6] = q2[3];
  b_q2[10] = q2[0];
  b_q2[14] = -q2[1];
  b_q2[3] = q2[3];
  b_q2[7] = -q2[2];
  b_q2[11] = q2[1];
  b_q2[15] = q2[0];
  for (i9 = 0; i9 < 4; i9++) {
    q[i9] = 0.0;
    for (i10 = 0; i10 < 4; i10++) {
      q[i9] += b_q2[i9 + (i10 << 2)] * q1[i10];
    }
  }
}

float IMU_getRoll(void){
    return IMU_roll;
}

float IMU_getPitch(void){
    return IMU_pitch;
}

float IMU_getYaw(void){
    return IMU_yaw;
}

void IMU_correctGyro(float dt){
    static float integrated_error[3] = {0}, prev_error[3] = {0};
    float error[3] = {0}, correction[3] = {0};

    float g_readings[3] = {ADXL345_getxG_f(),ADXL345_getyG_f(),ADXL345_getzG_f()};

    IMU_normalizeVector(g_readings,3);

    gravity_accel[0]=g_readings[0];
    gravity_accel[1]=g_readings[1];
    gravity_accel[2]=g_readings[2];

    // PLEASE check whether this is last column of rotation mat
    float gravity_est[3] = {2*(IMU_q[1]*IMU_q[3] - IMU_q[0]*IMU_q[2]),
                            2*(IMU_q[2]*IMU_q[3] + IMU_q[0]*IMU_q[1]),
                            pow(IMU_q[0],2)-pow(IMU_q[1],2)-pow(IMU_q[2],2)+pow(IMU_q[3],2)};

    gravity_estimate[0]=gravity_est[0];
    gravity_estimate[1]=gravity_est[1];
    gravity_estimate[2]=gravity_est[2];

    // Now get cross product of gravity_estimate and accel_reading
    // error = g_readings X gravity_est
    error[0] = g_readings[1]*gravity_est[2] - g_readings[2]*gravity_est[1];
    error[1] = g_readings[2]*gravity_est[0] - g_readings[0]*gravity_est[2];
    error[2] = g_readings[0]*gravity_est[1] - g_readings[1]*gravity_est[0];

    integrated_error[0] += 0.5*dt*(error[0] + prev_error[0]);
    integrated_error[1] += 0.5*dt*(error[1] + prev_error[1]);
    integrated_error[2] += 0.5*dt*(error[2] + prev_error[2]);

    prev_error[0] = error[0];
    prev_error[1] = error[1];
    prev_error[2] = error[2];

    correction[0] = error[0]*KP_DRIFT + integrated_error[0]*KI_DRIFT;
    correction[1] = error[1]*KP_DRIFT + integrated_error[1]*KI_DRIFT;
    correction[2] = error[2]*KP_DRIFT + integrated_error[2]*KI_DRIFT;

    angular_rate[0] = TO_RAD_PER_SEC(itg3200_data.x_float) + correction[0];
    angular_rate[1] = TO_RAD_PER_SEC(itg3200_data.y_float) + correction[1];
    angular_rate[2] = TO_RAD_PER_SEC(itg3200_data.z_float) + correction[2];
}

void IMU_normalizeVector(float array[], int len){
    unsigned char k;
    float norm = 0.0;

    for (k = 0; k < len; k++){
        norm += pow(array[k],2);
    }

    norm = sqrt(norm);

    for (k = 0; k < len; k++){
        array[k] = array[k]/norm;
    }
}