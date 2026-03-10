#ifndef PID_H
#define PID_H

#include "motor.h"
#include "cmsis_os2.h"

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float maxIntegral;
    float integral;
    float prev_error;
    float output;
} PidConstant;

extern PidConstant pidSpeedConstant[4][2];
extern PidConstant pidOrientationConstant;

// Define a structure to hold all sensor data in one place
typedef struct {
    // float roll;
    // float pitch;
    float yaw;
    int quat_accuracy;
    // float gyroX;
    // float gyroY;
    float gyroZ;
    int gyro_accuracy;
} BNO_Data;

typedef struct {
    // float roll;
    // float pitch;
    float yaw;
} Desired_Data;

extern BNO_Data bno_data;
extern Desired_Data desired_data;

extern osMutexId_t EulerAngleMutexHandle;
extern osMutexId_t PIDValueMutexHandle; //for motor pwm

void PID_Data_Refresh(void);
void PID_Balancing(void);
void PID_Speed(void);
void PID_Orientation(void);

#endif // PID_H
