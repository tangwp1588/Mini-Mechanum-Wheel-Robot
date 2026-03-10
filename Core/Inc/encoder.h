#pragma once
#include <stdint.h>
#include "cmsis_os.h"

typedef struct {
    int16_t motor0;
    int16_t motor1;
    int16_t motor2;
    int16_t motor3;
} EncoderCounts;

void encoder_INIT(void);
EncoderCounts GetEncoderCounts(void);
void encoderRPMcalc(void);
