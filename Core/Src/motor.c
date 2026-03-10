#include "motor.h"
#include "functions.h"
#include "kinematics.h"
#include "main.h"       // For HAL/GPIO functions
#include "tim.h"

// When pid is activate, the pins are correct but the motor runs forever once start, exchange the two ENCODER pins to fix it!
// This is because motor runs +ve rpm but encoder reads -ve rpm, and this cause the pid to windup.
// Always tune PWM direction first, then encoder direction !

void motor_INIT (void) {
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1); // M0
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2); // M1
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1); // M2
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2); // M3
}

void controlMotor(Motors *motors, int count) {
	if(osMutexAcquire(motorDataMutexHandle, 10) != osOK) return;

	// Copy motor data
	Motors local_motors[4];
	memcpy(local_motors, motors, sizeof(Motors) * 4);

	osMutexRelease(motorDataMutexHandle);

	int id, speed;
	for (id = 0; id < count; id++) {
		switch(id) {
			case 0:
				speed = coefficcient_M0 * local_motors[0].pwm;
				if (speed > 0) {
					HAL_GPIO_WritePin(M0_DIR1_GPIO_Port, M0_DIR1_Pin, SET);
					HAL_GPIO_WritePin(M0_DIR2_GPIO_Port, M0_DIR2_Pin, RESET);
				} else if (speed < 0) {
					HAL_GPIO_WritePin(M0_DIR1_GPIO_Port, M0_DIR1_Pin, RESET);
					HAL_GPIO_WritePin(M0_DIR2_GPIO_Port, M0_DIR2_Pin, SET);
				} else {
					HAL_GPIO_WritePin(M0_DIR1_GPIO_Port, M0_DIR1_Pin, SET);
					HAL_GPIO_WritePin(M0_DIR2_GPIO_Port, M0_DIR2_Pin, SET);
				}
				__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, constrain(abs(speed), 0, MAX_PWM));
				break;
			case 1:
				speed = coefficcient_M1 * local_motors[1].pwm;
				if (speed > 0) {
					HAL_GPIO_WritePin(M1_DIR1_GPIO_Port, M1_DIR1_Pin, SET);
					HAL_GPIO_WritePin(M1_DIR2_GPIO_Port, M1_DIR2_Pin, RESET);
				} else if (speed < 0) {
					HAL_GPIO_WritePin(M1_DIR1_GPIO_Port, M1_DIR1_Pin, RESET);
					HAL_GPIO_WritePin(M1_DIR2_GPIO_Port, M1_DIR2_Pin, SET);
				} else {
					HAL_GPIO_WritePin(M1_DIR1_GPIO_Port, M1_DIR1_Pin, SET);
					HAL_GPIO_WritePin(M1_DIR2_GPIO_Port, M1_DIR2_Pin, SET);
				}
				__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, constrain(abs(speed), 0, MAX_PWM));
				break;
			case 2:
				speed = coefficcient_M2 * local_motors[2].pwm;
				if (speed > 0) {
					HAL_GPIO_WritePin(M2_DIR1_GPIO_Port, M2_DIR1_Pin, SET);
					HAL_GPIO_WritePin(M2_DIR2_GPIO_Port, M2_DIR2_Pin, RESET);
				} else if (speed < 0) {
					HAL_GPIO_WritePin(M2_DIR1_GPIO_Port, M2_DIR1_Pin, RESET);
					HAL_GPIO_WritePin(M2_DIR2_GPIO_Port, M2_DIR2_Pin, SET);
				} else {
					HAL_GPIO_WritePin(M2_DIR1_GPIO_Port, M2_DIR1_Pin, SET);
					HAL_GPIO_WritePin(M2_DIR2_GPIO_Port, M2_DIR2_Pin, SET);
				}
				__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, constrain(abs(speed), 0, MAX_PWM));
				break;
			case 3:
				speed = coefficcient_M3 * local_motors[3].pwm;
				if (speed > 0) {
					HAL_GPIO_WritePin(M3_DIR1_GPIO_Port, M3_DIR1_Pin, SET);
					HAL_GPIO_WritePin(M3_DIR2_GPIO_Port, M3_DIR2_Pin, RESET);
				} else if (speed < 0) {
					HAL_GPIO_WritePin(M3_DIR1_GPIO_Port, M3_DIR1_Pin, RESET);
					HAL_GPIO_WritePin(M3_DIR2_GPIO_Port, M3_DIR2_Pin, SET);
				} else {
					HAL_GPIO_WritePin(M3_DIR1_GPIO_Port, M3_DIR1_Pin, SET);
					HAL_GPIO_WritePin(M3_DIR2_GPIO_Port, M3_DIR2_Pin, SET);
				}
				__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, constrain(abs(speed), 0, MAX_PWM));
				break;
			default:
				speed = 0;
				HAL_GPIO_WritePin(M0_DIR1_GPIO_Port, M0_DIR1_Pin, RESET);
				HAL_GPIO_WritePin(M0_DIR2_GPIO_Port, M0_DIR2_Pin, RESET);
				HAL_GPIO_WritePin(M1_DIR1_GPIO_Port, M1_DIR1_Pin, RESET);
				HAL_GPIO_WritePin(M1_DIR2_GPIO_Port, M1_DIR2_Pin, RESET);
				HAL_GPIO_WritePin(M2_DIR1_GPIO_Port, M2_DIR1_Pin, RESET);
				HAL_GPIO_WritePin(M2_DIR2_GPIO_Port, M2_DIR2_Pin, RESET);
				HAL_GPIO_WritePin(M3_DIR1_GPIO_Port, M3_DIR1_Pin, RESET);
				HAL_GPIO_WritePin(M3_DIR2_GPIO_Port, M3_DIR2_Pin, RESET);
				__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, speed);
				__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, speed);
				__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, speed);
				__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, speed);
				break;
		}
	}

	// TIM_HandleTypeDef *htim_1, *htim_2;
	// uint32_t channel_1, channel_2;
	// int id, speed;
	// for (id = 0; id < count; id++) {
	// 	switch(id) {
	// 		case 3:
	// 			htim_1 = &htim12;
	// 			htim_2 = &htim12;
	// 			channel_1 = TIM_CHANNEL_1;
	// 			channel_2 = TIM_CHANNEL_2;
	// 			speed = coefficcient_M3 * local_motors[3].pwm;
	// 			break;
	// 		case 1:
	// 			htim_1 = &htim8;
	// 			htim_2 = &htim8;
	// 			channel_1 = TIM_CHANNEL_1;
	// 			channel_2 = TIM_CHANNEL_2;
	// 			speed = coefficcient_M1 * local_motors[1].pwm;
	// 			break;
	// 		case 2:
	// 			htim_1 = &htim8;
	// 			htim_2 = &htim8;
	// 			channel_1 = TIM_CHANNEL_3;
	// 			channel_2 = TIM_CHANNEL_4;
	// 			speed = coefficcient_M2 * local_motors[2].pwm;
	// 			break;
	// 		case 0:
	// 			htim_1 = &htim9;
	// 			htim_2 = &htim9;
	// 			channel_1 = TIM_CHANNEL_1;
	// 			channel_2 = TIM_CHANNEL_2;
	// 			speed = coefficcient_M0 * local_motors[0].pwm;
	// 			break;
	// 		default:
	// 			htim_1 = NULL;
	// 			htim_2 = NULL;
	// 			channel_1 = 0;
	// 			channel_2 = 0;
	// 			speed = 0;
	// 			break;
	// 	}
	//
	// 		if (speed > 0) {
	// 			// speed = map(speed, 0, MAX_PWM, MIN_PWM, MAX_PWM);
	// 			__HAL_TIM_SET_COMPARE(htim_1, channel_1, constrain(abs(speed), 0, MAX_PWM));
	// 			__HAL_TIM_SET_COMPARE(htim_2, channel_2, 0);
	// 		} else if (speed < 0) {
	// 			// speed = map(speed, -MAX_PWM, 0, -MAX_PWM, -MIN_PWM);
	// 			__HAL_TIM_SET_COMPARE(htim_1, channel_1, 0);
	// 			__HAL_TIM_SET_COMPARE(htim_2, channel_2, constrain(abs(speed), 0, MAX_PWM));
	// 		} else {
	// 			__HAL_TIM_SET_COMPARE(htim_1, channel_1, MAX_PWM);
	// 			__HAL_TIM_SET_COMPARE(htim_2, channel_2, MAX_PWM);
	// 		}
	// 	}
}


//void stopAllMotors(void) {
//	controlMotor(0, 0);
//	controlMotor(1, 0);
//	controlMotor(2, 0);
//	controlMotor(3, 0);
//}

