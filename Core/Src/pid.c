#include "pid.h"
#include "motor.h"
#include "main.h"
#include "functions.h"
#include "controller_input.h"
#include "bno08x.h"

#define SAMPLE_TIME 0.01f  // 100 Hz (10ms)

BNO_Data bno_data = {0};
Desired_Data desired_data = {
	// .roll = 0.0f,
	// .pitch = 0.0f,
	.yaw = 0.0f,
};

// ARR = 8399
// #define PID_LOW_RPM  { .Kp = 50.0f, .Ki = 800.0f, .Kd = 0.0f }
// #define PID_HIGH_RPM { .Kp = 20.0f, .Ki = 420.0f, .Kd = 0.0f }

// ARR = 999
#define PID_LOW_RPM  { .Kp = 6.0f, .Ki = 95.0f, .Kd = 0.0f }
#define PID_HIGH_RPM { .Kp = 2.4f, .Ki = 50.0f, .Kd = 0.0f }

PidConstant pidSpeedConstant[4][2] = {
	{ PID_LOW_RPM, PID_HIGH_RPM }, // Motor 0
	{ PID_LOW_RPM, PID_HIGH_RPM }, // Motor 1
	{ PID_LOW_RPM, PID_HIGH_RPM }, // Motor 2
	{ PID_LOW_RPM, PID_HIGH_RPM }  // Motor 3
};

PidConstant pidOrientationConstant = {
	.Kp = 0.03f,
	.Ki = 0.0f,
	.Kd = 0.06f,
	.maxIntegral = 1.0f,
	.integral = 0.0f,
	.prev_error = 0.0f,
	.output = 0.0f //Differential PWM
};

BNO_Data local_bno = {0};
Desired_Data local_desired_data = {0};
// Motors localmotor[4] = {0};

void PID_Data_Refresh(void) {
	if(osMutexAcquire(EulerAngleMutexHandle, 10) == osOK) {
		memcpy(&local_bno, &bno_data, sizeof(BNO_Data));
		memcpy(&local_desired_data, &desired_data, sizeof(Desired_Data));
		osMutexRelease(EulerAngleMutexHandle);
	}

	// if(osMutexAcquire(motorDataMutexHandle, 1) == osOK) {
	// 	memcpy(localmotor, motor, sizeof(Motors) * 4);
	// 	osMutexRelease(motorDataMutexHandle);
	// }
}

void PID_Speed(void) {
	if(osMutexAcquire(PIDValueMutexHandle, 10) != osOK) return;

	if(osMutexAcquire(motorDataMutexHandle, 10) != osOK) {
		osMutexRelease(PIDValueMutexHandle);
		return;
	}

    for(uint8_t i = 0; i < 4; i++) {
        float desired = motor[i].rpmDesired;
    	float actual = motor[i].rpmFiltered;

    	PidConstant* config = &pidSpeedConstant[i][0];// We only use the integral and prev error from low speed set for simplicity

    	float Kp, Ki, Kd;
    	if (fabsf(desired) < 30) {
    		Kp = pidSpeedConstant[i][0].Kp;
    		Ki = pidSpeedConstant[i][0].Ki;
    		Kd = pidSpeedConstant[i][0].Kd;
    	} else {
    		Kp = pidSpeedConstant[i][1].Kp;
    		Ki = pidSpeedConstant[i][1].Ki;
    		Kd = pidSpeedConstant[i][1].Kd;
    	}

        if (desired == 0.0f) {
            motor[i].pwm = 0;               // Set PWM to 0 directly
            config->integral = 0.0f;         // Reset integral to prevent wind-up
            config->prev_error = 0.0f;       // Reset previous error to avoid derivative kick
            continue;                        // Skip PID computation for this motor
        }

        // PID computation for non-zero desired RPM
        float error = desired - actual;

        // Integral term update
        config->integral += error * SAMPLE_TIME;

        // Derivative term
        float derivative = (error - config->prev_error) / SAMPLE_TIME;

        // PID output
        float output = (Kp * error) +
                      (Ki * config->integral) +
                      (Kd * derivative);

        // Clamp output to PWM range
        output = constrain(output, -MAX_PWM, MAX_PWM);

        // Anti-windup: reverse-integrate if output saturates
        if(output >= MAX_PWM || output <= -MAX_PWM) {
            config->integral -= error * SAMPLE_TIME;
        }

        // Update motor PWM
        motor[i].pwm = (int16_t)output;

        // Store previous error
        config->prev_error = error;
    }

    osMutexRelease(motorDataMutexHandle);
	osMutexRelease(PIDValueMutexHandle);
}

void PID_Orientation(void) {
	if (!enableOrientationPID || isDrift || enableBrake || enableSpinningTopMode) { //can add isSelfRotate if u want
		if (osMutexAcquire(EulerAngleMutexHandle, 10) == osOK) {
			desired_data.yaw = bno_data.yaw; // just an indication of orientation pid is not activated
			osMutexRelease(EulerAngleMutexHandle);
		}
		if(osMutexAcquire(PIDValueMutexHandle, 10) == osOK) {
			pidOrientationConstant.output = 0;
			pidOrientationConstant.integral = 0;
			pidOrientationConstant.prev_error = 0;
			osMutexRelease(PIDValueMutexHandle);
		}
		return;
	}

	if(osMutexAcquire(PIDValueMutexHandle, 10) != osOK) return;

	PidConstant* config = &pidOrientationConstant;

	float error = local_desired_data.yaw - local_bno.yaw;
	error = fabs(error) < 0.3 ? 0 : error;
	// Handle angle wrap-around from -180 to +180 degrees
	if (error > 180.0f) {
		error -= 360.0f;
	} else if (error < -180.0f) {
		error += 360.0f;
	}

	// Integral term
	config->integral += error * SAMPLE_TIME;

	//Anti-windup
	if (config->integral >= config->maxIntegral) config->integral = config->maxIntegral;
	if (config->integral <= -config->maxIntegral) config->integral = -config->maxIntegral;

	// Derivative term
	float derivative = local_bno.gyroZ;
	// PID output
	float output = (config->Kp * error) +
					   (config->Ki * config->integral) +
					   (config->Kd * derivative);

	config->output = output;
	// config->prev_error = error;

	osMutexRelease(PIDValueMutexHandle);
}
