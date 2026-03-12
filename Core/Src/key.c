#include "key.h"

uint8_t Key_GetNum(void) {
	uint8_t KeyNum = 0;

	if(HAL_GPIO_ReadPin(KEY_0_GPIO_Port, KEY_0_Pin) == 1) {
		osDelay(100);
		while(HAL_GPIO_ReadPin(KEY_0_GPIO_Port, KEY_0_Pin) == 1);
		osDelay(10);
		KeyNum = 4;
	} // only KEY_0 is pull down

	if(HAL_GPIO_ReadPin(KEY_1_GPIO_Port, KEY_1_Pin) == 0) {
		osDelay(100);
		while(HAL_GPIO_ReadPin(KEY_1_GPIO_Port, KEY_1_Pin) == 0);
		osDelay(10);
		KeyNum = 3;
	}

	if(HAL_GPIO_ReadPin(KEY_2_GPIO_Port, KEY_2_Pin) == 0) {
		osDelay(100);
		while(HAL_GPIO_ReadPin(KEY_2_GPIO_Port, KEY_2_Pin) == 0);
		osDelay(10);
		KeyNum = 2;
	}

	if(HAL_GPIO_ReadPin(KEY_3_GPIO_Port, KEY_3_Pin) == 0) {
		osDelay(100);
		while(HAL_GPIO_ReadPin(KEY_3_GPIO_Port, KEY_3_Pin) == 0);
		osDelay(10);
		KeyNum = 1;
	}
	// if(HAL_GPIO_ReadPin(KEY_0_GPIO_Port, KEY_0_Pin) == 1) {
	// 	osDelay(20);
	// 	while(HAL_GPIO_ReadPin(KEY_0_GPIO_Port, KEY_0_Pin) == 1);
	// 	osDelay(10);
	// 	KeyNum = 1;
	// } // only KEY_0 is pull down
	//
	// if(HAL_GPIO_ReadPin(KEY_1_GPIO_Port, KEY_1_Pin) == 0) {
	// 	osDelay(20);
	// 	while(HAL_GPIO_ReadPin(KEY_1_GPIO_Port, KEY_1_Pin) == 0);
	// 	osDelay(10);
	// 	KeyNum = 2;
	// }
	//
	// if(HAL_GPIO_ReadPin(KEY_2_GPIO_Port, KEY_2_Pin) == 0) {
	// 	osDelay(20);
	// 	while(HAL_GPIO_ReadPin(KEY_2_GPIO_Port, KEY_2_Pin) == 0);
	// 	osDelay(10);
	// 	KeyNum = 3;
	// }
	//
	// if(HAL_GPIO_ReadPin(KEY_3_GPIO_Port, KEY_3_Pin) == 0) {
	// 	osDelay(20);
	// 	while(HAL_GPIO_ReadPin(KEY_3_GPIO_Port, KEY_3_Pin) == 0);
	// 	osDelay(10);
	// 	KeyNum = 4;
	// }

	return KeyNum;
}
