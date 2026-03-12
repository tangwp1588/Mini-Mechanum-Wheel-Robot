/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "uart_handler.h"
#include "usart.h"
#include "controller_input.h"
#include "motor.h"
#include "encoder.h"
#include "kinematics.h"
//#include "oled.h"
#include "st7789.h"
#include "pid.h"
#include "bno08x.h"
#include "menu.h"
#include "key.h"
#include "adc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for EncoderTask */
osThreadId_t EncoderTaskHandle;
const osThreadAttr_t EncoderTask_attributes = {
  .name = "EncoderTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for ControllerTask */
osThreadId_t ControllerTaskHandle;
const osThreadAttr_t ControllerTask_attributes = {
  .name = "ControllerTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for OledTask */
osThreadId_t OledTaskHandle;
const osThreadAttr_t OledTask_attributes = {
  .name = "OledTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for PIDTask */
osThreadId_t PIDTaskHandle;
const osThreadAttr_t PIDTask_attributes = {
  .name = "PIDTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for IMUTask */
osThreadId_t IMUTaskHandle;
const osThreadAttr_t IMUTask_attributes = {
  .name = "IMUTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for motorDataMutex */
osMutexId_t motorDataMutexHandle;
const osMutexAttr_t motorDataMutex_attributes = {
  .name = "motorDataMutex"
};
/* Definitions for EulerAngleMutex */
osMutexId_t EulerAngleMutexHandle;
const osMutexAttr_t EulerAngleMutex_attributes = {
  .name = "EulerAngleMutex"
};
/* Definitions for PIDValueMutex */
osMutexId_t PIDValueMutexHandle;
const osMutexAttr_t PIDValueMutex_attributes = {
  .name = "PIDValueMutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartEncoderTask(void *argument);
void StartMotorTask(void *argument);
void StartControllerTask(void *argument);
void StartOledTask(void *argument);
void StartPIDTask(void *argument);
void StartIMUTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
/* USER CODE END 1 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of motorDataMutex */
  motorDataMutexHandle = osMutexNew(&motorDataMutex_attributes);

  /* creation of EulerAngleMutex */
  EulerAngleMutexHandle = osMutexNew(&EulerAngleMutex_attributes);

  /* creation of PIDValueMutex */
  PIDValueMutexHandle = osMutexNew(&PIDValueMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of EncoderTask */
  EncoderTaskHandle = osThreadNew(StartEncoderTask, NULL, &EncoderTask_attributes);

  /* creation of MotorTask */
  MotorTaskHandle = osThreadNew(StartMotorTask, NULL, &MotorTask_attributes);

  /* creation of ControllerTask */
  ControllerTaskHandle = osThreadNew(StartControllerTask, NULL, &ControllerTask_attributes);

  /* creation of OledTask */
  OledTaskHandle = osThreadNew(StartOledTask, NULL, &OledTask_attributes);

  /* creation of PIDTask */
  PIDTaskHandle = osThreadNew(StartPIDTask, NULL, &PIDTask_attributes);

  /* creation of IMUTask */
  IMUTaskHandle = osThreadNew(StartIMUTask, NULL, &IMUTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartEncoderTask */
/**
* @brief Function implementing the EncoderTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartEncoderTask */
void StartEncoderTask(void *argument)
{
  /* USER CODE BEGIN StartEncoderTask */
	uint32_t previousWakeTime = osKernelGetTickCount();
//	EncoderCounts prev_counts = {0};
	const uint32_t delayTime = 10; // 10ms
  /* Infinite loop */
  for(;;)
  {
	  encoderRPM();
	  previousWakeTime += delayTime;
	  osDelayUntil(previousWakeTime);
  }
  /* USER CODE END StartEncoderTask */
}

/* USER CODE BEGIN Header_StartMotorTask */
/**
  * @brief  Function implementing the MotorTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMotorTask */
void StartMotorTask(void *argument)
{
  /* USER CODE BEGIN StartMotorTask */
	uint32_t previousWakeTime = osKernelGetTickCount();
	const uint32_t delayTime = 10; // 10ms period
	/* Infinite loop */
	for(;;)
	{
		controlMotor(motor, 4);
		previousWakeTime += delayTime;
		osDelayUntil(previousWakeTime);
	}
  /* USER CODE END StartMotorTask */
}

/* USER CODE BEGIN Header_StartControllerTask */
/**
* @brief Function implementing the ControllerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartControllerTask */
void StartControllerTask(void *argument)
{
  /* USER CODE BEGIN StartControllerTask */
	uint32_t previousWakeTime = osKernelGetTickCount();
	const uint32_t delayTime = 10;
  /* Infinite loop */
  for(;;)
  {
	  processGamepad();
	  previousWakeTime += delayTime;
	  osDelayUntil(previousWakeTime);
  }
  /* USER CODE END StartControllerTask */
}

/* USER CODE BEGIN Header_StartOledTask */
/**
* @brief Function implementing the OledTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartOledTask */
void StartOledTask(void *argument)
{
  /* USER CODE BEGIN StartOledTask */
	uint32_t previousWakeTime = osKernelGetTickCount();
	const uint32_t delayTime = 10;

	Motor local_motor[4] = {0};
	char MessageMotor0[40], MessageMotor1[40], MessageMotor2[40], MessageMotor3[40];

	BNO_Data local_bno = {0};
	Desired_Data local_desired_data = {0};
	char MessageYaw[40], MessageQuatAccuracy[40];
	char MessageGyroZ[40], MessageDesiredYaw[40];

	// ADC: Variables for voltage calculation
	uint32_t raw_adc_value = 0;
	float calculated_voltage = 0.0f;
	char MessageVoltage[40];

	// screen rotation modify in st7789.h
  /* Infinite loop */
  for(;;)
  {
	  if(osMutexAcquire(motorDataMutexHandle, 10) == osOK) {
		  memcpy(local_motor, motor, sizeof(Motor) * 4);
	      osMutexRelease(motorDataMutexHandle);
	  }

	  if(osMutexAcquire(EulerAngleMutexHandle, 10) == osOK) {
	  	  memcpy(&local_bno, &bno_data, sizeof(BNO_Data));
	  	  memcpy(&local_desired_data, &desired_data, sizeof(Desired_Data));
	      osMutexRelease(EulerAngleMutexHandle);
	  }

  	// HAL_ADC_Start(&hadc1);
  	// if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) // Poll with a 10ms timeout
  	// {
  	// 	raw_adc_value = HAL_ADC_GetValue(&hadc1);
  	// }


  	// --- Calculations & Formatting ---
  	// ADC: Calculate voltage from the raw ADC value
  	// calculated_voltage = ((float)raw_adc_value / 4095.0f) * 3.4f * 11.0f;
  	// calculated_voltage = raw_adc_value;
//	  sprintf(MessageMotor0, "M0:%4d  %4d  %3d",(int)localmotor[0].rpmDesired, (int)localmotor[0].rpmMeasured, (int)localmotor[0].pwm);
//	  sprintf(MessageMotor1, "M1:%4d  %4d  %3d",(int)localmotor[1].rpmDesired, (int)localmotor[1].rpmMeasured, (int)localmotor[1].pwm);
//	  sprintf(MessageMotor2, "M2:%4d  %4d  %3d",(int)localmotor[2].rpmDesired, (int)localmotor[2].rpmMeasured, (int)localmotor[2].pwm);
//	  sprintf(MessageMotor3, "M3:%4d  %4d  %3d",(int)localmotor[3].rpmDesired, (int)localmotor[3].rpmMeasured, (int)localmotor[3].pwm);

  	// space after every value so that no left decimal place that are not refresh
  	sprintf(MessageMotor0, "M0:%4d  %5d", (int)local_motor[0].rpmFiltered, (int)local_motor[0].pwm);
  	sprintf(MessageMotor1, "M1:%4d  %5d", (int)local_motor[1].rpmFiltered, (int)local_motor[1].pwm);
  	sprintf(MessageMotor2, "M2:%4d  %5d", (int)local_motor[2].rpmFiltered, (int)local_motor[2].pwm);
  	sprintf(MessageMotor3, "M3:%4d  %5d", (int)local_motor[3].rpmFiltered, (int)local_motor[3].pwm);

	sprintf(MessageYaw,          "Yaw:      %5.1f ", local_bno.yaw);
  	sprintf(MessageQuatAccuracy, "Accuracy: %2d ", local_bno.quat_accuracy);

  	sprintf(MessageGyroZ,        "GyroZ:    %5.1f ", local_bno.gyroZ);
  	sprintf(MessageDesiredYaw,   "DesYaw:   %5.1f ", local_desired_data.yaw);
	// sprintf(MessageVoltage,    "Batt:    %3.2fV", calculated_voltage);
	// OLED_ShowString(0, 0, "    RPM1 RPM  PWM", OLED_7X12);
	//
	// OLED_ShowString(0, 12+3, MessageMotor0, OLED_7X12);
	// OLED_ShowString(0, 24+3, MessageMotor1, OLED_7X12);
	// OLED_ShowString(0, 36+3, MessageMotor2, OLED_7X12);
	// OLED_ShowString(0, 48+3, MessageMotor3, OLED_7X12);
	//
	// OLED_Update();
	ST7789_DrawRectangle(1,1,238,238,WHITE);
	ST7789_WriteString(10, 10, "    RPM    PWM", Font_11x18, WHITE, DARKBLUE);

  	ST7789_WriteString(10, 10+20+5, MessageMotor0, Font_11x18, MAGENTA, DARKBLUE);
  	ST7789_WriteString(10, 10+20*2+5, MessageMotor1, Font_11x18, YELLOW, DARKBLUE);
  	ST7789_WriteString(10, 10+20*3+5, MessageMotor2, Font_11x18, CYAN, DARKBLUE);
  	ST7789_WriteString(10, 10+20*4+5, MessageMotor3, Font_11x18, LIGHTGREEN, DARKBLUE);


  	ST7789_WriteString(10, 10+20*6+5, MessageYaw, Font_11x18, GREEN, DARKBLUE);
  	ST7789_WriteString(10, 10+20*7+5, MessageQuatAccuracy, Font_11x18, LGREY, DARKBLUE);

  	ST7789_WriteString(10, 10+20*8+5, MessageGyroZ, Font_11x18, WHITE, DARKBLUE);
  	ST7789_WriteString(10, 10+20*9+5, MessageDesiredYaw, Font_11x18, BRRED, DARKBLUE);
	// ST7789_WriteString(10, 10+20*9+10, MessageVoltage, Font_11x18, RED, DARKBLUE);

	uint8_t KeyNum = Key_GetNum();
	if (KeyNum > 0) {
	  ST7789_Fill_Color(BGCOLOUR);
	  mainMenu();
	}

	previousWakeTime += delayTime;
	osDelayUntil(previousWakeTime);
  }
  /* USER CODE END StartOledTask */
}

/* USER CODE BEGIN Header_StartPIDTask */
/**
* @brief Function implementing the PIDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPIDTask */
void StartPIDTask(void *argument)
{
  /* USER CODE BEGIN StartPIDTask */
	uint32_t previousWakeTime = osKernelGetTickCount();
	const uint32_t delayTime = 10;  // 10ms period (100Hz)

	/* Infinite loop */
	for(;;)
	{
		PID_Data_Refresh();
#ifndef PWM_ONLY
		PID_Speed();
#endif
		PID_Orientation();

		previousWakeTime += delayTime;
		osDelayUntil(previousWakeTime);
	}
  /* USER CODE END StartPIDTask */
}

/* USER CODE BEGIN Header_StartIMUTask */
/**
* @brief Function implementing the IMUTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartIMUTask */
void StartIMUTask(void *argument)
{
  /* USER CODE BEGIN StartIMUTask */

	float q0, q1, q2, q3;
	float roll, pitch, yaw;

	// ADDED: Flag to track if we've successfully saved calibration to flash
	bool calibrationSaved = false;
	// ADDED: Track time to ensure the sensor has been stable at high accuracy for a moment
	uint32_t highAccuracyStartTime = 0;

	HAL_GPIO_WritePin(IMU_RST_GPIO_Port, IMU_RST_Pin, GPIO_PIN_RESET);
	osDelay(10);
	HAL_GPIO_WritePin(IMU_RST_GPIO_Port, IMU_RST_Pin, GPIO_PIN_SET);
	osDelay(190);

	// Reset Sensor
	softReset();
	// enableARVRStabilizedRotationVector(5);
	// enableRotationVector(5);
	enableGameRotationVector(5);
	// enableAccelerometer(100);
	// enableLinearAccelerometer(100);
	enableGyro(5);
	// enableMagnetometer(100);
	// enableStepCounter(100);
	// enableStabilityClassifier(100);

	// Start calibration background process
	// calibrateAll();

	uint32_t previousWakeTime = osKernelGetTickCount();
	const uint32_t delayTime = 5;

  /* Infinite loop */
	for(;;)
	{
		while ( dataAvailable() )
		{
			q0 = getQuatReal();
			q1 = getQuatI();
			q2 = getQuatJ();
			q3 = getQuatK();

			// roll = atan2( 2 * ( q0 * q1 + q2 * q3 ) ,  1- 2 * ( q1 * q1 + q2 * q2 ) ) * 57.295779515f;
			// pitch = asin( 2 * ( q0 * q2 - q3 * q1 ) ) * 57.295779515f;
			yaw = atan2( 2 * ( q0 * q3 + q1 * q2 ) ,  1 - 2 * ( q2 * q2 + q3 * q3 ) ) * 57.295779515f;
            uint8_t current_accuracy = getQuatAccuracy();

			if(osMutexAcquire(EulerAngleMutexHandle, 10) == osOK) {
				// bno_data.roll  = roll;
				// bno_data.pitch = pitch;
				bno_data.yaw = yaw;
				bno_data.quat_accuracy = current_accuracy;
				// bno_data.gyroX = getGyroX();
				// bno_data.gyroY = getGyroY();
				bno_data.gyroZ = getGyroZ();

				osMutexRelease(EulerAngleMutexHandle);
			}

			// --- SAFE CALIBRATION SAVE LOGIC ---
			// If we haven't saved yet, and accuracy is high (3)
			if (!calibrationSaved && current_accuracy == 3) {
				if (highAccuracyStartTime == 0) {
					// Start a 2-second timer to ensure it's STABLE at accuracy 3
					highAccuracyStartTime = osKernelGetTickCount();
				}
				else if ((osKernelGetTickCount() - highAccuracyStartTime) >= 2000) {
					// It has been stable at accuracy 3 for 2 seconds. Save it to flash.
					saveCalibration();
					calibrationSaved = true; // Never save again this power cycle
				}
			}
			else if (current_accuracy < 3) {
				// Reset the stability timer if accuracy drops
				highAccuracyStartTime = 0;
			}
			// -----------------------------------
		}
		previousWakeTime += delayTime;
		osDelayUntil(previousWakeTime);
	}
  /* USER CODE END StartIMUTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

