// kinematics.h
#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "main.h"

extern const int coefficcient_M0;
extern const int coefficcient_M1;
extern const int coefficcient_M2;
extern const int coefficcient_M3;

extern const int MAX_PWM;
extern const int MIN_PWM;
extern const int MAX_RPM;
extern const int MIN_RPM;
extern const int SELF_ROTATION_RPM;
extern const int DEADZONE_JOYSTICK_X;
extern const int DEADZONE_JOYSTICK_Y;
extern const int DEADZONE_TRIGGER;

extern float spinningTopSetpoint;

// #define PWM_ONLY

void InverseKinematics(int LX, int LY, int RX, int Throttle);

void driveStandard(double power, double theta, double rotation);
void driveDrift(int x, int y, double power, double rotation);
void driveSpinningTop(double power, double theta, double rotation);

void applySpeedMultiplier(double s0, double s1, double s2, double s3);



#endif // KINEMATICS_H
