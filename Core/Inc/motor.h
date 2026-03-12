// motor.h
#pragma once
#include "main.h"
#include "kinematics.h"
#include "cmsis_os2.h"

typedef struct {
    TIM_HandleTypeDef* htim;
    uint32_t channel;
	float coefficient;
    GPIO_TypeDef* dir1_port;
    uint16_t dir1_pin;
    GPIO_TypeDef* dir2_port;
    uint16_t dir2_pin;
} MotorConfig;

typedef struct {
    int16_t pwm;
	float rpmDesired;
	float rpmMeasured;
	float rpmFiltered;
} Motor;

extern Motor motor[4];

//extern MotorSpeeds motorSpeeds;
extern osMutexId_t motorDataMutexHandle; //for motor pwm

void motor_INIT ();
void controlMotor(Motor *motor, int count);
void stopAllMotors();
//void testingMotors();
