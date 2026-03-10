#include "encoder.h"
#include "motor.h"
#include "main.h"
#include "tim.h"

#define ENCODER_CPR 11.0f
#define GEAR_RATIO 48.0f
#define QUADRATURE 4.0f
#define SAMPLE_TIME_S 0.01f // 10ms

const float COUNTS_PER_REV = ENCODER_CPR * GEAR_RATIO * QUADRATURE;

#define LPF_ALPHA 0.3f
EncoderCounts prev_counts = {0};

void encoder_INIT(void) {
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL); // M0
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // M1
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); // M2
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); // M3
}

EncoderCounts GetEncoderCounts(void) {
	EncoderCounts counts = {
		.motor0 = (int16_t)__HAL_TIM_GET_COUNTER(&htim1),
		.motor1 = (int16_t)__HAL_TIM_GET_COUNTER(&htim3),
		.motor2 = (int16_t)__HAL_TIM_GET_COUNTER(&htim2),
		.motor3 = (int16_t)__HAL_TIM_GET_COUNTER(&htim4)
	};
	return counts;
}

void encoderRPMcalc(void) {
	EncoderCounts current_counts = GetEncoderCounts();

	// By casting the subtraction to int16_t, the CPU automatically handles
	// the wrap-around from 65535 to 0 (and vice versa) correctly.
	int16_t delta0 = (int16_t)(current_counts.motor0 - prev_counts.motor0);
	int16_t delta1 = (int16_t)(current_counts.motor1 - prev_counts.motor1);
	int16_t delta2 = (int16_t)(current_counts.motor2 - prev_counts.motor2);
	int16_t delta3 = (int16_t)(current_counts.motor3 - prev_counts.motor3);

	prev_counts = current_counts;

	// RPM = (delta_ticks / counts_per_rev) * (60 / sample_time)
	float rpm0 = (delta0 * 60.0f) / (COUNTS_PER_REV * SAMPLE_TIME_S);
	float rpm1 = (delta1 * 60.0f) / (COUNTS_PER_REV * SAMPLE_TIME_S);
	float rpm2 = (delta2 * 60.0f) / (COUNTS_PER_REV * SAMPLE_TIME_S);
	float rpm3 = (delta3 * 60.0f) / (COUNTS_PER_REV * SAMPLE_TIME_S);

	if(osMutexAcquire(motorDataMutexHandle, 10) == osOK) {
		motor[0].rpmMeasured = rpm0;
		motor[1].rpmMeasured = rpm1;
		motor[2].rpmMeasured = rpm2;
		motor[3].rpmMeasured = rpm3;

		for(uint8_t i = 0; i < 4; i++) {
			motor[i].rpmFiltered = LPF_ALPHA * motor[i].rpmMeasured + (1.0f - LPF_ALPHA) * motor[i].rpmFiltered;
		}
		osMutexRelease(motorDataMutexHandle);
	}
}