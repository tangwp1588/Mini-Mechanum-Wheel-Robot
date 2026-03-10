#include "kinematics.h"
#include "controller_input.h"
#include "functions.h"
#include "motor.h"
#include "main.h"
#include "pid.h"

const int coefficcient_M0 = -1;
const int coefficcient_M1 = -1;
const int coefficcient_M2 = 1;
const int coefficcient_M3 = -1;

const int MAX_PWM = 999;
const int MIN_PWM = 99; // avoid using this in PID calculation
const float DEADZONE_PWM_RATIO = (float)MIN_PWM / (float)MAX_PWM; //

const int MAX_RPM = 240;
const int MIN_RPM = 0;
// const int SELF_ROTATION_RPM = 120;
const int DEADZONE_JOYSTICK_X = 50;
const int DEADZONE_JOYSTICK_Y = 25;
const int DEADZONE_TRIGGER = 100;

const int UP_MAX_JOYSTICK_LY = -512;
const int DN_MAX_JOYSTICK_LY = 508;
const int L_MAX_JOYSTICK_LX = -512;
const int R_MAX_JOYSTICK_LX = 508;
const int L_MAX_JOYSTICK_RX = -512;
const int R_MAX_JOYSTICK_RX = 508;
const int MAX_TRIGGER = 1020;

Motors motor[4] = {0};// the actual location of this array

void InverseKinematics(int LX, int LY, int RX, int Throttle) {
	int x = (abs(LX) < DEADZONE_JOYSTICK_X) ? 0 : LX;
	int y = (abs(LY) < DEADZONE_JOYSTICK_Y) ? 0 : -LY;
	int rx = (abs(RX) < DEADZONE_JOYSTICK_X) ? 0 : RX;
	int rt = (Throttle < DEADZONE_TRIGGER) ? 0 : Throttle;

	double theta = (x == 0 && y == 0) ? M_PI_2 : atan2(y, x);
	double power = (rt == 0) ? 0 : (double)(rt - DEADZONE_TRIGGER) / (MAX_TRIGGER - DEADZONE_TRIGGER); // 0 to 1
	double rotation = (rx == 0) ? 0 : RX > 0 ? (double)(rx - DEADZONE_JOYSTICK_X) / (R_MAX_JOYSTICK_RX - DEADZONE_JOYSTICK_X)
	                                         : (double)(rx + DEADZONE_JOYSTICK_X) / -(L_MAX_JOYSTICK_RX + DEADZONE_JOYSTICK_X); // -1 to 1
	rotation *= rotation * rotation; // non-linear increase to allow precise control in low speed without sacrifice high speed control
#ifdef PWM_ONLY
	if (power != 0 && fabs(power) < DEADZONE_PWM_RATIO) power = power > 0 ? DEADZONE_PWM_RATIO : -DEADZONE_PWM_RATIO;
	if (rotation != 0 && fabs(rotation) < DEADZONE_PWM_RATIO) rotation = rotation > 0 ? DEADZONE_PWM_RATIO : -DEADZONE_PWM_RATIO;
#endif
	if (isDrift) {
		driveDrift(x, y, power, rotation);
		return;
	}
	// if (enableSlowRotation && !isDrift) rotation /= 4;

	if (enableSpinningTopMode) {
		driveSpinningTop(power, theta, rotation);
	} else {
		driveStandard(power, theta, rotation);
	}
}

void driveStandard(double power, double theta, double rotation) {
	double Sin = sin(theta - M_PI_4);
	double Cos = cos(theta - M_PI_4);
	double Max = (fabs(Sin) > fabs(Cos)) ? fabs(Sin) : fabs(Cos);

	double s0 = power * Cos / Max + rotation;
	double s1 = power * Sin / Max - rotation;
	double s2 = power * Cos / Max - rotation;
	double s3 = power * Sin / Max + rotation;

	applySpeedMultiplier(s0, s1, s2, s3);
}

void driveDrift(int x, int y, double power, double rotation) {
	double s0, s1, s2, s3;

	double forward = y >= 0 ? power : -power;
	double yaw = rotation;
	double strafe = 2.0f;

	// strafe =  (float)rx / (float)512;
	// yaw = 0.3f;
	//
	// if (strafe != 0) {
	// 	forward /= 4;
	// 	if (strafe > 0) yaw *= 1;
	// 	else if (strafe < 0) yaw *= -1;
	// } else yaw = 0;

	if (yaw != 0) {
		forward /= 4;
		if (yaw > 0) strafe *= 1;
		else if (yaw < 0) strafe *= -1;
	} else strafe = 0;

	s0 = s3 = forward + yaw;
	s1 = s2 = forward - yaw;

	if (strafe < 0) {s0 -= strafe; s2 -= strafe;}
	if (strafe > 0) {s1 += strafe; s3 += strafe;}

	float maxVal = fabs(s0);
	if (fabs(s1) > maxVal) maxVal = fabs(s1);
	if (fabs(s2) > maxVal) maxVal = fabs(s2);
	if (fabs(s3) > maxVal) maxVal = fabs(s3);

	if (maxVal > 1) {
		float scalar = (float)1 / maxVal;
		s0 *= scalar; s1 *= scalar; s2 *= scalar; s3 *= scalar;
	}

	applySpeedMultiplier(s0, s1, s2, s3);
}

float spinningTopSetpoint = 0.0f;
void driveSpinningTop(double power, double theta, double rotation) {
	float currentYaw = 0;
	float gyroZ_rad = 0;

	if(osMutexAcquire(EulerAngleMutexHandle, 10) == osOK) {
		currentYaw = bno_data.yaw;
		gyroZ_rad = bno_data.gyroZ; // Angular velocity in degrees per second
		osMutexRelease(EulerAngleMutexHandle);
	}

	// Latency compensation
	float leadAngle_deg = (gyroZ_rad * 57.295779515f) * -0.08f;

	// Use the PREDICTED yaw instead of the CURRENT yaw
	float predictedYaw = currentYaw + leadAngle_deg;

	// Shortest-path angle normalization (using predictedYaw)
	float diff = predictedYaw - spinningTopSetpoint;
	while (diff > 180.0f)  diff -= 360.0f;
	while (diff < -180.0f) diff += 360.0f;

	double yawErrorRad = diff * (M_PI / 180.0f);
	theta -= yawErrorRad;

	double Sin = sin(theta - M_PI_4);
	double Cos = cos(theta - M_PI_4);

	double s0 = (power * Cos) + rotation;
	double s1 = (power * Sin) - rotation;
	double s2 = (power * Cos) - rotation;
	double s3 = (power * Sin) + rotation;

	// [Proportional Scaling Logic from previous step]
	double maxVal = fabs(s0);
	if (fabs(s1) > maxVal) maxVal = fabs(s1);
	if (fabs(s2) > maxVal) maxVal = fabs(s2);
	if (fabs(s3) > maxVal) maxVal = fabs(s3);

	if (maxVal > 1) {
		s0 /= maxVal; s1 /= maxVal; s2 /= maxVal; s3 /= maxVal;
	}

	applySpeedMultiplier(s0, s1, s2, s3);
}

void applySpeedMultiplier(double s0, double s1, double s2, double s3) {
	float yawCorrection = 0;
	if(osMutexAcquire(PIDValueMutexHandle, 10) == osOK) {
		yawCorrection = enableOrientationPID ? pidOrientationConstant.output : 0.0f;
		osMutexRelease(PIDValueMutexHandle);
	}
	if(osMutexAcquire(motorDataMutexHandle, 10) == osOK) {
#ifdef PWM_ONLY
		// double m[4];
		// m[0] = s0 - yawCorrection;
		// m[1] = s1 + yawCorrection;
		// m[2] = s2 + yawCorrection;
		// m[3] = s3 - yawCorrection;
		// for (int i = 0; i < 4; i++) {
		// 	if(m[i] > 0) motor[i].pwm = (int16_t)constrain(m[i] * (MAX_PWM - MIN_PWM) + MIN_PWM, MIN_PWM, MAX_PWM);
		// 	else if (m[i] < 0) motor[i].pwm = (int16_t)constrain(m[i] * (MAX_PWM - MIN_PWM) - MIN_PWM, -MAX_PWM, -MIN_PWM);
		// 	else motor[i].pwm = 0;
		// }
		motor[0].pwm = (int16_t)constrain((s0 - yawCorrection) * MAX_PWM, -MAX_PWM, MAX_PWM);
		motor[1].pwm = (int16_t)constrain((s1 + yawCorrection) * MAX_PWM, -MAX_PWM, MAX_PWM);
		motor[2].pwm = (int16_t)constrain((s2 + yawCorrection) * MAX_PWM, -MAX_PWM, MAX_PWM);
		motor[3].pwm = (int16_t)constrain((s3 - yawCorrection) * MAX_PWM, -MAX_PWM, MAX_PWM);
#else
		motor[0].rpmDesired = (float)constrain((s0 - yawCorrection) * MAX_RPM, -MAX_RPM, MAX_RPM);
		motor[1].rpmDesired = (float)constrain((s1 + yawCorrection) * MAX_RPM, -MAX_RPM, MAX_RPM);
		motor[2].rpmDesired = (float)constrain((s2 + yawCorrection) * MAX_RPM, -MAX_RPM, MAX_RPM);
		motor[3].rpmDesired = (float)constrain((s3 - yawCorrection) * MAX_RPM, -MAX_RPM, MAX_RPM);
#endif
		osMutexRelease(motorDataMutexHandle);
	}

}
// void holonomicXDrive(int LX, int LY, int RX, int Throttle, bool isTurningLeft, bool isTurningRight) {
// 	int x = (abs(LX) < JOYSTICK_DEADZONE_X) ? 0 : LX;
// 	int y = (abs(LY) < JOYSTICK_DEADZONE_Y) ? 0 : -LY;
// 	int rx = (abs(RX) < JOYSTICK_DEADZONE_X) ? 0 : map(RX, -512, 508, -MAX_RPM, MAX_RPM);
// 	// int rx = (abs(RX) < JOYSTICK_DEADZONE_X) ? 0 : map(RX, -512, 508, -6000, 6000); // for direct-pwm
//
// 	// Calculate angle and power from controller inputs
// 	double theta;
// 	if (x == 0 && y == 0) theta = 0.5 * M_PI;
// 	else theta = atan2(y, x);
//
// 	int power = (Throttle < TRIGGER_DEADZONE) ? 0 : map(Throttle, TRIGGER_DEADZONE, 1020,
// 														MIN_RPM, MAX_RPM);
// 	// int power = (Throttle < TRIGGER_DEADZONE) ? 0 : map(Throttle, TRIGGER_DEADZONE, 1020, 0, MAX_PWM-2000); // for direct-pwm
//
// 	// Convert from polar to mechanum drive equations
// 	double Sin = sin(theta - M_PI / 4);
// 	double Cos = cos(theta - M_PI / 4);
// 	double Max = (fabs(Sin) > fabs(Cos)) ? fabs(Sin) : fabs(Cos);
//
// 	double speed_M0 = round(power * Cos / Max) + rx; // FRONT_LEFT
// 	double speed_M1 = round(power * Sin / Max) - rx; // FRONT_RIGHT
// 	double speed_M2 = round(power * Cos / Max) - rx; // BACK_RIGHT
// 	double speed_M3 = round(power * Sin / Max) + rx; // BACK_LEFT
//
// // Drifting
// 	// Controls forward speed when drifting
// 	speed_M0 *= (isTurningLeft)  ?  0.7 : 1;
// 	speed_M1 *= (isTurningRight) ?  0.7 : 1;
//
// 	// Controls rotate speed when drifting
// 	speed_M2 *= (isTurningRight) ? -1 : 1; // BACK_RIGHT
// 	speed_M3 *= (isTurningLeft)  ? -1 : 1; // BACK_LEFT
//
// 	float yawCorrection = enableOrientationPID ? pidOrientationConstant.output : 0.0f;
//
// 	if(osMutexAcquire(PIDValueMutexHandle, 10) == osOK) {
// 		if(osMutexAcquire(motorDataMutexHandle, 10) == osOK) {
// 			motor[0].rpmDesired = (float)constrain(speed_M0, -MAX_RPM, MAX_RPM) - yawCorrection;
// 			motor[1].rpmDesired = (float)constrain(speed_M1, -MAX_RPM, MAX_RPM) + yawCorrection;
// 			motor[2].rpmDesired = (float)constrain(speed_M2, -MAX_RPM, MAX_RPM) + yawCorrection;
// 			motor[3].rpmDesired = (float)constrain(speed_M3, -MAX_RPM, MAX_RPM) - yawCorrection;
//
// 			// motor[0].pwm = constrain(speed_M0, -MAX_PWM, MAX_PWM) - yawCorrection;
// 			// motor[1].pwm = constrain(speed_M1, -MAX_PWM, MAX_PWM) + yawCorrection;
// 			// motor[2].pwm = constrain(speed_M2, -MAX_PWM, MAX_PWM) + yawCorrection;
// 			// motor[3].pwm = constrain(speed_M3, -MAX_PWM, MAX_PWM) - yawCorrection; // for direct pwm control
// 			osMutexRelease(motorDataMutexHandle);
// 		}
// 		osMutexRelease(PIDValueMutexHandle);
// 	}
// }