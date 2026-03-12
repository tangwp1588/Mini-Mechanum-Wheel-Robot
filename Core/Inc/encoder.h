#pragma once
#include <stdint.h>
#include "main.h"
#include "cmsis_os2.h"

typedef struct {
    TIM_HandleTypeDef* htim;
    uint32_t channel;
    float coefficient;
    uint32_t prev_count;
} EncoderConfig;

typedef struct {
    uint32_t prev_count;
} Encoder;

void encoder_INIT(void);
void encoderRPM(void);
