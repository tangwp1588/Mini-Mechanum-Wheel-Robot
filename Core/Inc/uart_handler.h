// uart_handler.h
#pragma once
#include "controller_input.h"
#include "main.h"

// Initialize UART6 and DMA
void UART_Init(void);

// Get latest controller data
ControllerData* UART_GetControllerData(void);
extern volatile ControllerData controllerData;
