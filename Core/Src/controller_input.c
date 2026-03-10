#include "controller_input.h"
#include "motor.h"
#include "kinematics.h"
#include "uart_handler.h"
#include "main.h"
#include "pid.h"

bool enableBrake = false;
// bool isTurningLeft = false;
// bool isTurningRight = false;
bool isDrift = false;
bool isSelfRotate = false;
// bool enableSlowRotation = true;
bool enableOrientationPID = false;
bool enableSpinningTopMode = false;

// #define DOUBLE_CLICK_ENABLED

int checkButtonEvent(ButtonHandler* h, uint16_t currentButtonsValue) {
	int event = EVENT_NONE;
	uint8_t reading = (currentButtonsValue & h->mask) ? 1 : 0;
	uint32_t now = osKernelGetTickCount();

	// 1. Debounce Logic
	if (reading != h->lastRawState) {
		h->lastDebounceTime = now;
	}

	if ((now - h->lastDebounceTime) > DEBOUNCE_MS) {
		if (reading != h->lastSteadyState) {
			h->lastSteadyState = reading;

			if (h->lastSteadyState == 1) { // Transition to PRESSED
				h->pressStartTime = now;
				h->longPressTriggered = 0;
			} else { // Transition to RELEASED
				if (!h->longPressTriggered) {
					h->clickCount++;
					h->lastClickTime = now; // Mark exactly when the click finished
				}
			}
		}
	}

	// 2. Long Press Detection (While held)
	if (h->lastSteadyState == 1 && !h->longPressTriggered) {
		if ((now - h->pressStartTime) > LONG_PRESS_MS) {
			h->longPressTriggered = 1;
			h->clickCount = 0; // Cancel any pending double-clicks
			event = EVENT_LONG;
		}
	}

	// 3. Click Evaluation (Only runs when button is released)
	if (h->lastSteadyState == 0 && h->clickCount > 0) {
	#ifdef DOUBLE_CLICK_ENABLED
		if ((now - h->lastClickTime) > DOUBLE_CLICK_MS) {
			if (h->clickCount == 1) {
				event = EVENT_SINGLE;
			} else if (h->clickCount >= 2) {
				event = EVENT_DOUBLE;
			}
			h->clickCount = 0;
		}
	#else
		if (h->clickCount >= 1) {
			event = EVENT_SINGLE;
		}
		h->clickCount = 0;
	#endif
	}

	h->lastRawState = reading;
	return event;
}

ButtonHandler handlerLS = { .mask = BUTTON_LS };
ButtonHandler handlerRS = { .mask = BUTTON_RS };
ButtonHandler handlerX = { .mask = BUTTON_X };
ButtonHandler handlerY = { .mask = BUTTON_Y };

void processGamepad(void) {
	static bool motorTestM0 = false;
	static bool motorTestM1 = false;
	static bool motorTestM2 = false;
	static bool motorTestM3 = false;

	ControllerData* data = UART_GetControllerData();


	int btnLSEvent = checkButtonEvent(&handlerLS, data->buttons);
	int btnRSEvent = checkButtonEvent(&handlerRS, data->buttons);
	int btnXEvent = checkButtonEvent(&handlerX, data->buttons);
	int btnYEvent = checkButtonEvent(&handlerY, data->buttons);

	if (btnXEvent == EVENT_SINGLE) {
		motorTestM0 = !motorTestM0;
		if (osMutexAcquire(motorDataMutexHandle, 10) == osOK) {
			motor[0].rpmDesired = motorTestM0 ? 120 : 0;
			osMutexRelease(motorDataMutexHandle);
		}
	}

	if (btnYEvent == EVENT_SINGLE) {
		motorTestM1 = !motorTestM1;
		if (osMutexAcquire(motorDataMutexHandle, 10) == osOK) {
			motor[1].rpmDesired = motorTestM1 ? 120 : 0;
			osMutexRelease(motorDataMutexHandle);
		}
	}

	if (btnXEvent == EVENT_LONG) {
		motorTestM3 = !motorTestM3;
		if (osMutexAcquire(motorDataMutexHandle, 10) == osOK) {
			motor[3].rpmDesired = motorTestM3 ? 120 : 0;
			osMutexRelease(motorDataMutexHandle);
		}
	}

	if (btnYEvent == EVENT_LONG) {
		motorTestM2 = !motorTestM2;
		if (osMutexAcquire(motorDataMutexHandle, 10) == osOK) {
			motor[2].rpmDesired = motorTestM2 ? 120 : 0;
			osMutexRelease(motorDataMutexHandle);
		}
	}

	if (btnRSEvent == EVENT_SINGLE) {
		// Toggle Orientation PID
		enableOrientationPID = !enableOrientationPID;
		if (enableOrientationPID) {
			if(osMutexAcquire(EulerAngleMutexHandle, 10) == osOK) {
				desired_data.yaw = bno_data.yaw;
				osMutexRelease(EulerAngleMutexHandle);
			}
		}
	}

	// if (btnRSEvent == EVENT_LONG) {
	// 	enableSlowRotation = !enableSlowRotation;
	// }

	if (btnLSEvent == EVENT_SINGLE) {
		isDrift = !isDrift;
	}

	if (data->brake > 300) {
		if (!enableSpinningTopMode) {
			if(osMutexAcquire(EulerAngleMutexHandle, 10) == osOK) {
				spinningTopSetpoint = bno_data.yaw;
				osMutexRelease(EulerAngleMutexHandle);
			}
		}
		enableSpinningTopMode = true;
	} else enableSpinningTopMode = false;

	int targetX = data->axisX;
	int targetY = data->axisY;
	int targetRX = data->axisRX;
	int targetRT = data->throttle;

	// isDrift = data->buttons & BUTTON_LB;
	isSelfRotate = abs(data->axisRX) > DEADZONE_JOYSTICK_X;

	if (data->dpad != 0) {
		targetX = 0;
		targetY = 0;

		if (data->dpad & DPAD_UP)    targetY -= 508;
		if (data->dpad & DPAD_DOWN)  targetY += 508;
		if (data->dpad & DPAD_LEFT)  targetX -= 508;
		if (data->dpad & DPAD_RIGHT) targetX += 508;
	}

	if (!motorTestM0 && !motorTestM1 && !motorTestM2 && !motorTestM3 ) {
		InverseKinematics(targetX, targetY, targetRX, targetRT);
	}
}
