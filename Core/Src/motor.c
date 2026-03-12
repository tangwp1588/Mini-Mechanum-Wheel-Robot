#include "motor.h"
#include "functions.h"
#include "kinematics.h"
#include "main.h"
#include "tim.h"

// Tune PWM direction first before encoder direction.
// If motor run very fast at low speed, change encoder's coefficient.
// If motor run very fast at low speed in reversed direction, change PWM coefficient.
// If motor runs normally in reverse direction, change both PWM and encoder's coefficient.

MotorConfig MOTOR_CONFIG[4];
Motor motor[4] = {0};

void motor_INIT(void) {
	MOTOR_CONFIG[0] = (MotorConfig){&htim8, TIM_CHANNEL_1, -1.0f, M0_DIR1_GPIO_Port, M0_DIR1_Pin, M0_DIR2_GPIO_Port, M0_DIR2_Pin}; // M0
	MOTOR_CONFIG[1] = (MotorConfig){&htim8, TIM_CHANNEL_2, -1.0f, M1_DIR1_GPIO_Port, M1_DIR1_Pin, M1_DIR2_GPIO_Port, M1_DIR2_Pin}; // M1
	MOTOR_CONFIG[2] = (MotorConfig){&htim9, TIM_CHANNEL_1,  1.0f, M2_DIR1_GPIO_Port, M2_DIR1_Pin, M2_DIR2_GPIO_Port, M2_DIR2_Pin}; // M2
	MOTOR_CONFIG[3] = (MotorConfig){&htim9, TIM_CHANNEL_2, -1.0f, M3_DIR1_GPIO_Port, M3_DIR1_Pin, M3_DIR2_GPIO_Port, M3_DIR2_Pin}; // M3
	for(int i = 0; i < 4; i++) {
		HAL_TIM_PWM_Start(MOTOR_CONFIG[i].htim, MOTOR_CONFIG[i].channel);
	}
}

void controlMotor(Motor *motor, int count) {
	// Thread safety: Copy input states to local buffer
	Motor local_motor[4];
	if(osMutexAcquire(motorDataMutexHandle, 10) == osOK) {
		memcpy(local_motor, motor, sizeof(Motor) * 4);
		osMutexRelease(motorDataMutexHandle);
	} else {
		return;
	}

	for (int i = 0; i < count; i++) {
		const MotorConfig *cfg = &MOTOR_CONFIG[i];
		int pwm = (int)roundf((float)local_motor[i].pwm * cfg->coefficient);

		if (pwm > 0) {
			HAL_GPIO_WritePin(cfg->dir1_port, cfg->dir1_pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(cfg->dir2_port, cfg->dir2_pin, GPIO_PIN_RESET);
		} else if (pwm < 0) {
			HAL_GPIO_WritePin(cfg->dir1_port, cfg->dir1_pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(cfg->dir2_port, cfg->dir2_pin, GPIO_PIN_SET);
		} else {
			// Brake mode
			HAL_GPIO_WritePin(cfg->dir1_port, cfg->dir1_pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(cfg->dir2_port, cfg->dir2_pin, GPIO_PIN_SET);
		}

		__HAL_TIM_SET_COMPARE(cfg->htim, cfg->channel, constrain(abs(pwm), 0, MAX_PWM));
	}
}