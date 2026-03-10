// uart_handler.c
#include "uart_handler.h"
#include "usart.h"
#include <string.h>

//extern UART_HandleTypeDef huart5;

// Controller data buffer
volatile ControllerData controllerData;
static uint8_t uartRxBuffer[sizeof(ControllerData)];
// Analog data might look weird when inspecting at live expression window when you run debug,
// but it's okay as later on it will be remapped into int16_t again

// Initialize UART6 and start DMA reception
void UART_Init(void) {
  HAL_UART_Receive_DMA(&huart2, uartRxBuffer, sizeof(ControllerData));
}

// UART RX Complete Callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart2) {
    memcpy((void*)&controllerData, uartRxBuffer, sizeof(ControllerData));
    HAL_UART_Receive_DMA(&huart2, uartRxBuffer, sizeof(ControllerData)); // Restart DMA
  }
}

// Get pointer to controller data
ControllerData* UART_GetControllerData(void) {
  return (ControllerData*)&controllerData;
}
