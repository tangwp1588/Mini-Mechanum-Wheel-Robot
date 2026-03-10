// controller_input.h
#pragma once
#include "main.h"       // For HAL/GPIO functions
#include "cmsis_os2.h"

#pragma pack(push, 1)
typedef struct {
    int16_t axisX, axisY, axisRX, axisRY;
    uint16_t dpad;
    uint16_t buttons;
    uint16_t brake, throttle;
} ControllerData;
#pragma pack(pop)

// D-pad bitmasks
#define DPAD_UP    0x01
#define DPAD_DOWN  0x02
#define DPAD_LEFT  0x08
#define DPAD_RIGHT 0x04

// Button bitmasks
#define BUTTON_A   0x0001
#define BUTTON_B   0x0002
#define BUTTON_X   0x0004
#define BUTTON_Y   0x0008
#define BUTTON_LB  0x0010
#define BUTTON_RB  0x0020
#define BUTTON_LS  0x0100
#define BUTTON_RS  0x0200

// Button Event Types
#define EVENT_NONE   0
#define EVENT_SINGLE 1
#define EVENT_DOUBLE 2
#define EVENT_LONG   3

#define DEBOUNCE_MS     30
#define DOUBLE_CLICK_MS 300
#define LONG_PRESS_MS   800

typedef struct {
    uint16_t mask;              // e.g., BUTTON_A
    uint8_t lastRawState;
    uint8_t lastSteadyState;    // 0 or 1
    uint32_t lastDebounceTime;
    uint32_t pressStartTime;
    uint32_t lastClickTime;
    int clickCount;
    uint8_t longPressTriggered;
    uint8_t waitingForDoubleClick;
} ButtonHandler;

extern bool enableBrake;
// extern bool isTurningLeft;
// extern bool isTurningRight;
extern bool isDrift;
extern bool isSelfRotate;
extern bool enableSlowRotation;
extern bool enableOrientationPID;
extern bool enableSpinningTopMode;

int checkButtonEvent(ButtonHandler* h, uint16_t currentButtonsValue);
void processGamepad(void);
