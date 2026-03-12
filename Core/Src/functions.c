#include "functions.h"

#include <tgmath.h>

int map(int x, int in_min, int in_max, int out_min, int out_max) {
    if (in_max == in_min) return out_min;  // avoid division by zero

    if (x > in_max) return out_max;
    if (x < in_min) return out_min;

    float ratio = (float)(x - in_min) / (float)(in_max - in_min);
    float result = (float)out_min + ratio * (float)(out_max - out_min);
    return round(result);
}

double constrain(double value, double min, double max) {
    if (value < min) return min;
    else if (value > max) return max;
    else return value;
}
