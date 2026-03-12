#include "encoder.h"
#include "motor.h"
#include "main.h"
#include "tim.h"

#define LPF_ALPHA 0.3f

#define ENCODER_CPR 11.0f
#define GEAR_RATIO 48.0f
#define QUADRATURE 4.0f
#define SAMPLE_TIME_S 0.01f // 10ms
const float COUNTS_PER_REV = ENCODER_CPR * GEAR_RATIO * QUADRATURE;

EncoderConfig ENCODER_CONFIG[4];
Encoder encoder[4];

void encoder_INIT(void) {
	ENCODER_CONFIG[0] =	(EncoderConfig){&htim1, TIM_CHANNEL_ALL, 1};
	ENCODER_CONFIG[1] =	(EncoderConfig){&htim3, TIM_CHANNEL_ALL, 1};
	ENCODER_CONFIG[2] =	(EncoderConfig){&htim2, TIM_CHANNEL_ALL, 1};
	ENCODER_CONFIG[3] =	(EncoderConfig){&htim4, TIM_CHANNEL_ALL, 1};
	for(int i = 0; i < 4; i++) {
		HAL_TIM_Encoder_Start(ENCODER_CONFIG[i].htim, ENCODER_CONFIG[i].channel);
	}
}

void encoderRPM(void) {
	for(int i = 0; i < 4; i++) {
		const uint32_t counts = __HAL_TIM_GET_COUNTER(ENCODER_CONFIG[i].htim);
		const int16_t delta = (int16_t)(counts - encoder[i].prev_count);
		encoder[i].prev_count = counts;

		const float rpm = ((float)delta / COUNTS_PER_REV) * (60.0f / SAMPLE_TIME_S);
		if(osMutexAcquire(motorDataMutexHandle, 10) == osOK) {
			motor[i].rpmMeasured = rpm * ENCODER_CONFIG[i].coefficient;
			motor[i].rpmFiltered = LPF_ALPHA * motor[i].rpmMeasured + (1.0f - LPF_ALPHA) * motor[i].rpmFiltered;
			osMutexRelease(motorDataMutexHandle);
		}
	}
}