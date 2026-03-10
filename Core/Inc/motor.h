// motor.h
#pragma once
#include "main.h"
#include "kinematics.h"
#include "cmsis_os2.h"

typedef struct {
	int8_t id;
    int16_t pwm;
	float rpmDesired;
	float rpmMeasured;
	float rpmFiltered;
} Motors;

extern Motors motor[4];

//extern MotorSpeeds motorSpeeds;
extern osMutexId_t motorDataMutexHandle; //for motor pwm

void motor_INIT ();
void controlMotor(Motors *motors, int count);
void stopAllMotors();
//void testingMotors();
