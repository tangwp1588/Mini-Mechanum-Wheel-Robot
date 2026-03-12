#include "menu.h"
#include "main.h"
#include "key.h"
//#include "OLED.h"
#include "st7789.h"
#include "pid.h"

uint8_t KeyNum;
int menu2;
float stepSize = 10;
char MessageStepSize1[10], MessageKp1[10], MessageKi1[10], MessageKd1[10]; // for orientation PID
char MessageStepSize2[10], MessageKpLow2[10], MessageKiLow2[10], MessageKdLow2[10], MessageKpHigh2[10], MessageKiHigh2[10], MessageKdHigh2[10]; // for speed PID

void mainMenu(void) {
	while(1) {
		menu2 = menu_lvl1();
		if (menu2 == 1) return;
		if (menu2 == 2) menu_lvl2_orientationPID();
		if (menu2 == 3) menu_lvl2_speedPID();
	}
}

void menu_lvl1_display(int LineNum) {
	// OLED_ShowString(0, 0, "<-             ", OLED_8X16);
	// OLED_ShowString(0, 16, "Option 1        ", OLED_8X16);
	// OLED_ShowString(0, 32, "Option 2        ", OLED_8X16);
	// OLED_ShowString(0, 48, "Option 3        ", OLED_8X16);
	//
	// if (line_no >= 1 && line_no <= 4) OLED_ReverseArea(0, (LineNum - 1) * 16, 64, 16);
	// OLED_Update();

	if (LineNum == 1)
		ST7789_WriteString(0,  0, "<---          ", FONT, TXTCOLOUR, SELCOLOUR);
	else ST7789_WriteString(0,  0, "<---          ", FONT, TXTCOLOUR, BGCOLOUR);

	if (LineNum == 2)
		ST7789_WriteString(0, SPACING, "OrientationPID", FONT, TXTCOLOUR, SELCOLOUR);
	else ST7789_WriteString(0, SPACING, "OrientationPID", FONT, TXTCOLOUR, BGCOLOUR);

	if (LineNum == 3)
		ST7789_WriteString(0, SPACING*2, "Speed PID     ", FONT, TXTCOLOUR, SELCOLOUR);
    else ST7789_WriteString(0, SPACING*2, "Speed PID     ", FONT, TXTCOLOUR, BGCOLOUR);
}

int menu_lvl1(void) {
	uint8_t flag = 1;
	menu_lvl1_display(0);

	while(1) {
		KeyNum = Key_GetNum();
		if(KeyNum == 1) {
			flag = 1;
			ST7789_Fill_Color(BGCOLOUR);
			return flag;
		}
		if(KeyNum == 3) {
			flag++;
			if(flag == 4) flag = 1;
		}
		if(KeyNum == 2) {
			flag--;
			if(flag == 0) flag = 3;
		}
		if(KeyNum == 4) {
//			OLED_Clear();
//			OLED_Update();
			ST7789_Fill_Color(BGCOLOUR);
			return flag;
		}

		menu_lvl1_display(flag);
	}
}

void orientationPID_printf() {
	sprintf(MessageStepSize1, "%9.4f", stepSize);
	sprintf(MessageKp1, "%9.4f", pidOrientationConstant.Kp);
	sprintf(MessageKi1, "%9.4f", pidOrientationConstant.Ki);
	sprintf(MessageKd1, "%9.4f", pidOrientationConstant.Kd);
}

void speedPID_printf() {
	sprintf(MessageStepSize2, "%9.4f", stepSize);
	sprintf(MessageKpLow2, "%9.4f", pidSpeedConstant[0][0].Kp);
	sprintf(MessageKiLow2, "%9.4f", pidSpeedConstant[0][0].Ki);
	sprintf(MessageKdLow2, "%9.4f", pidSpeedConstant[0][0].Kd);
	sprintf(MessageKpHigh2, "%9.4f", pidSpeedConstant[0][1].Kp);
	sprintf(MessageKiHigh2, "%9.4f", pidSpeedConstant[0][1].Ki);
	sprintf(MessageKdHigh2, "%9.4f", pidSpeedConstant[0][1].Kd);
}

void menu_lvl2_orientationPID_display(int LineNum) {
	orientationPID_printf();

	if (LineNum == 1)
		ST7789_WriteString(0,  0, "<---          ", FONT, TXTCOLOUR, SELCOLOUR);
	else ST7789_WriteString(0,  0, "<---          ", FONT, TXTCOLOUR, BGCOLOUR);

	if (LineNum == 2) {
		ST7789_WriteString(0, SPACING, "SS:", FONT, TXTCOLOUR, SELCOLOUR);
		ST7789_WriteString(64, SPACING, MessageStepSize1, FONT, TXTCOLOUR, BGCOLOUR);
	} else {
		ST7789_WriteString(0, SPACING, "SS:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(64, SPACING, MessageStepSize1, FONT, TXTCOLOUR, BGCOLOUR);
	}

	if (LineNum == 3) {
		ST7789_WriteString(0, SPACING*2, "Kp:", FONT, TXTCOLOUR, SELCOLOUR);
		ST7789_WriteString(64, SPACING*2, MessageKp1, FONT, TXTCOLOUR, BGCOLOUR);
	} else {
		ST7789_WriteString(0, SPACING*2, "Kp:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(64, SPACING*2, MessageKp1, FONT, TXTCOLOUR, BGCOLOUR);
	}

	if (LineNum == 4) {
		ST7789_WriteString(0, SPACING*3, "Ki:", FONT, TXTCOLOUR, SELCOLOUR);
		ST7789_WriteString(64, SPACING*3, MessageKi1, FONT, TXTCOLOUR, BGCOLOUR);
	} else {
		ST7789_WriteString(0, SPACING*3, "Ki:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(64, SPACING*3, MessageKi1, FONT, TXTCOLOUR, BGCOLOUR);
	}

	if (LineNum == 5) {
		ST7789_WriteString(0, SPACING*4, "Kd:", FONT, TXTCOLOUR, SELCOLOUR);
		ST7789_WriteString(64, SPACING*4, MessageKd1, FONT, TXTCOLOUR, BGCOLOUR);
	} else {
		ST7789_WriteString(0, SPACING*4, "Kd:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(64, SPACING*4, MessageKd1, FONT, TXTCOLOUR, BGCOLOUR);
	}
}

int menu_lvl2_orientationPID(void) {
	uint8_t flag_lvl2 = 1, flag_lvl3 = 0;
	menu_lvl2_orientationPID_display(0);

	while(1) {
		KeyNum = Key_GetNum();
		if(KeyNum == 1) {
			// ST7789_Fill_Color(BLACK);
			flag_lvl3 = 1;
		}
		if(KeyNum == 3) {
			flag_lvl2++;
			if(flag_lvl2 == 6) flag_lvl2 = 1;
		}
		if(KeyNum == 2) {
			flag_lvl2--;
			if(flag_lvl2 == 0) flag_lvl2 = 5;
		}
		if(KeyNum == 4) {
//			OLED_Clear();
//			OLED_Update();
			// ST7789_Fill_Color(BLACK);
			flag_lvl3 = flag_lvl2;
		}

		if(flag_lvl3 == 1) {
			ST7789_Fill_Color(BGCOLOUR);
			return 0;
		}
		if(flag_lvl3 == 2) flag_lvl3 = menu_lvl3_orientationPID_stepSize();
		if(flag_lvl3 == 3) flag_lvl3 = menu_lvl3_orientationPID_constants(1);
		if(flag_lvl3 == 4) flag_lvl3 = menu_lvl3_orientationPID_constants(2);
		if(flag_lvl3 == 5) flag_lvl3 = menu_lvl3_orientationPID_constants(3);

		menu_lvl2_orientationPID_display(flag_lvl2);
	}
}

void menu_lvl2_speedPID_display(int LineNum) {
	speedPID_printf();

	if (LineNum == 1)
		ST7789_WriteString(0,  0, "<---          ", FONT, TXTCOLOUR, SELCOLOUR);
	else ST7789_WriteString(0,  0, "<---          ", FONT, TXTCOLOUR, BGCOLOUR);

	if (LineNum == 2) {
		ST7789_WriteString(0, SPACING, "SS:", FONT, TXTCOLOUR, SELCOLOUR);
		ST7789_WriteString(80, SPACING, MessageStepSize2, FONT, TXTCOLOUR, BGCOLOUR);
	} else {
		ST7789_WriteString(0, SPACING, "SS:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(80, SPACING, MessageStepSize2, FONT, TXTCOLOUR, BGCOLOUR);
	}

	if (LineNum == 3) {
		ST7789_WriteString(0, SPACING*2, "LKp:", FONT, TXTCOLOUR, SELCOLOUR);
		ST7789_WriteString(80, SPACING*2, MessageKpLow2, FONT, TXTCOLOUR, BGCOLOUR);
	} else {
		ST7789_WriteString(0, SPACING*2, "LKp:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(80, SPACING*2, MessageKpLow2, FONT, TXTCOLOUR, BGCOLOUR);
	}

	if (LineNum == 4) {
		ST7789_WriteString(0, SPACING*3, "LKi:", FONT, TXTCOLOUR, SELCOLOUR);
		ST7789_WriteString(80, SPACING*3, MessageKiLow2, FONT, TXTCOLOUR, BGCOLOUR);
	} else {
		ST7789_WriteString(0, SPACING*3, "LKi:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(80, SPACING*3, MessageKiLow2, FONT, TXTCOLOUR, BGCOLOUR);
	}

	if (LineNum == 5) {
		ST7789_WriteString(0, SPACING*4, "LKd:", FONT, TXTCOLOUR, SELCOLOUR);
		ST7789_WriteString(80, SPACING*4, MessageKdLow2, FONT, TXTCOLOUR, BGCOLOUR);
	} else {
		ST7789_WriteString(0, SPACING*4, "LKd:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(80, SPACING*4, MessageKdLow2, FONT, TXTCOLOUR, BGCOLOUR);
	}

	if (LineNum == 6) {
		ST7789_WriteString(0, SPACING*5, "HKp:", FONT, TXTCOLOUR, SELCOLOUR);
		ST7789_WriteString(80, SPACING*5, MessageKpHigh2, FONT, TXTCOLOUR, BGCOLOUR);
	} else {
		ST7789_WriteString(0, SPACING*5, "HKp:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(80, SPACING*5, MessageKpHigh2, FONT, TXTCOLOUR, BGCOLOUR);
	}

	if (LineNum == 7) {
		ST7789_WriteString(0, SPACING*6, "HKi:", FONT, TXTCOLOUR, SELCOLOUR);
		ST7789_WriteString(80, SPACING*6, MessageKiHigh2, FONT, TXTCOLOUR, BGCOLOUR);
	} else {
		ST7789_WriteString(0, SPACING*6, "HKi:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(80, SPACING*6, MessageKiHigh2, FONT, TXTCOLOUR, BGCOLOUR);
	}

	if (LineNum == 8) {
		ST7789_WriteString(0, SPACING*7, "HKd:", FONT, TXTCOLOUR, SELCOLOUR);
		ST7789_WriteString(80, SPACING*7, MessageKdHigh2, FONT, TXTCOLOUR, BGCOLOUR);
	} else {
		ST7789_WriteString(0, SPACING*7, "HKd:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(80, SPACING*7, MessageKdHigh2, FONT, TXTCOLOUR, BGCOLOUR);
	}
}

int menu_lvl2_speedPID(void) {
	uint8_t flag_lvl2 = 1, flag_lvl3 = 0;
	menu_lvl2_speedPID_display (0);

	while(1) {
		KeyNum = Key_GetNum();
		if(KeyNum == 1) {
			flag_lvl3 = 1;
		}
		if(KeyNum == 3) {
			flag_lvl2++;
			if(flag_lvl2 == 9) flag_lvl2 = 1;
		}
		if(KeyNum == 2) {
			flag_lvl2--;
			if(flag_lvl2 == 0) flag_lvl2 = 8;
		}
		if(KeyNum == 4) {
//			OLED_Clear();
//			OLED_Update();
			flag_lvl3 = flag_lvl2;
		}

		if(flag_lvl3 == 1) {
			ST7789_Fill_Color(BGCOLOUR);
			return 0;
		}
		if(flag_lvl3 == 2) flag_lvl3 = menu_lvl3_speedPID_stepSize();
		if(flag_lvl3 == 3) flag_lvl3 = menu_lvl3_speedPID_constants(1);
		if(flag_lvl3 == 4) flag_lvl3 = menu_lvl3_speedPID_constants(2);
		if(flag_lvl3 == 5) flag_lvl3 = menu_lvl3_speedPID_constants(3);
		if(flag_lvl3 == 6) flag_lvl3 = menu_lvl3_speedPID_constants(4);
		if(flag_lvl3 == 7) flag_lvl3 = menu_lvl3_speedPID_constants(5);
		if(flag_lvl3 == 8) flag_lvl3 = menu_lvl3_speedPID_constants(6);

		menu_lvl2_speedPID_display(flag_lvl2);
	}
}

void menu_lvl3_orientationPID_display(int LineNum) {
	orientationPID_printf();

	if (LineNum == 1)
		ST7789_WriteString(0,  0, "<---          ", FONT, TXTCOLOUR, SELCOLOUR);
	else ST7789_WriteString(0,  0, "<---          ", FONT, TXTCOLOUR, BGCOLOUR);

	if (LineNum == 2) {
		ST7789_WriteString(0, SPACING, "SS:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(64, SPACING, MessageStepSize1, FONT, TXTCOLOUR, SELCOLOUR);
	} else {
		ST7789_WriteString(0, SPACING, "SS:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(64, SPACING, MessageStepSize1, FONT, TXTCOLOUR, BGCOLOUR);
	}

	if (LineNum == 3) {
		ST7789_WriteString(0, SPACING*2, "Kp:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(64, SPACING*2, MessageKp1, FONT, TXTCOLOUR, SELCOLOUR);
	} else {
		ST7789_WriteString(0, SPACING*2, "Kp:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(64, SPACING*2, MessageKp1, FONT, TXTCOLOUR, BGCOLOUR);
	}

	if (LineNum == 4) {
		ST7789_WriteString(0, SPACING*3, "Ki:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(64, SPACING*3, MessageKi1, FONT, TXTCOLOUR, SELCOLOUR);
	} else {
		ST7789_WriteString(0, SPACING*3, "Ki:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(64, SPACING*3, MessageKi1, FONT, TXTCOLOUR, BGCOLOUR);
	}

	if (LineNum == 5) {
		ST7789_WriteString(0, SPACING*4, "Kd:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(64, SPACING*4, MessageKd1, FONT, TXTCOLOUR, SELCOLOUR);
	} else {
		ST7789_WriteString(0, SPACING*4, "Kd:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(64, SPACING*4, MessageKd1, FONT, TXTCOLOUR, BGCOLOUR);
	}
}

int menu_lvl3_orientationPID_stepSize() {
	while (1) {
		menu_lvl3_orientationPID_display(2); // Step size option is in line 2

		KeyNum = Key_GetNum();
		if (KeyNum == 1 || KeyNum == 4) {
			return 0;
		}
		if(KeyNum == 2) {
			stepSize *= 10;
			if (stepSize > 1000) stepSize = 1;
		}
		if(KeyNum == 3) {
			stepSize /= 10;
			if (stepSize <= 0.00001) stepSize = 1;
		}
		// if(KeyNum == 4) {
		// 	return 0;
		// }
	}
}

int menu_lvl3_orientationPID_constants(int id) {
	while (1) {
		KeyNum = Key_GetNum();
		if (KeyNum == 1 || KeyNum == 4) {
			return 0;
		}
		if(osMutexAcquire(PIDValueMutexHandle, 10) == osOK) {
			if(KeyNum == 2) {
				pidOrientationConstant.integral = 0;
				pidOrientationConstant.prev_error = 0;
				if (id == 1) pidOrientationConstant.Kp -= stepSize;
				if (id == 2) pidOrientationConstant.Ki -= stepSize;
				if (id == 3) pidOrientationConstant.Kd -= stepSize;
			}
			if(KeyNum == 3) {
				if (id == 1) pidOrientationConstant.Kp += stepSize;
				if (id == 2) pidOrientationConstant.Ki += stepSize;
				if (id == 3) pidOrientationConstant.Kd += stepSize;
			}
			osMutexRelease(PIDValueMutexHandle);
		}
		// if(KeyNum == 4) {
		// 	return 0;
		// }
		switch(id) {
			case 1:{
				menu_lvl3_orientationPID_display(3); // Kp option is in line 3
				break;
			}
			case 2:{
				menu_lvl3_orientationPID_display(4); // Ki option is in line 4
				break;
			}
			case 3: {
				menu_lvl3_orientationPID_display(5); // Kd option is in line 5
				break;
			}
		}
	}
}

void menu_lvl3_speedPID_display(int LineNum) {
	speedPID_printf();

	if (LineNum == 1) // not used
		ST7789_WriteString(0,  0, "<---          ", FONT, TXTCOLOUR, SELCOLOUR);
	else ST7789_WriteString(0,  0, "<---          ", FONT, TXTCOLOUR, BGCOLOUR);

	if (LineNum == 2) {
		ST7789_WriteString(0, SPACING, "SS:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(80, SPACING, MessageStepSize2, FONT, TXTCOLOUR, SELCOLOUR);
	} else {
		ST7789_WriteString(0, SPACING, "SS:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(80, SPACING, MessageStepSize2, FONT, TXTCOLOUR, BGCOLOUR);
	}

	if (LineNum == 3) {
		ST7789_WriteString(0, SPACING*2, "LKp:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(80, SPACING*2, MessageKpLow2, FONT, TXTCOLOUR, SELCOLOUR);
	} else {
		ST7789_WriteString(0, SPACING*2, "LKp:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(80, SPACING*2, MessageKpLow2, FONT, TXTCOLOUR, BGCOLOUR);
	}

	if (LineNum == 4) {
		ST7789_WriteString(0, SPACING*3, "LKi:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(80, SPACING*3, MessageKiLow2, FONT, TXTCOLOUR, SELCOLOUR);
	} else {
		ST7789_WriteString(0, SPACING*3, "LKi:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(80, SPACING*3, MessageKiLow2, FONT, TXTCOLOUR, BGCOLOUR);
	}

	if (LineNum == 5) {
		ST7789_WriteString(0, SPACING*4, "LKd:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(80, SPACING*4, MessageKdLow2, FONT, TXTCOLOUR, SELCOLOUR);
	} else {
		ST7789_WriteString(0, SPACING*4, "LKd:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(80, SPACING*4, MessageKdLow2, FONT, TXTCOLOUR, BGCOLOUR);
	}

	if (LineNum == 6) {
		ST7789_WriteString(0, SPACING*5, "HKp:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(80, SPACING*5, MessageKpHigh2, FONT, TXTCOLOUR, SELCOLOUR);
	} else {
		ST7789_WriteString(0, SPACING*5, "HKp:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(80, SPACING*5, MessageKpHigh2, FONT, TXTCOLOUR, BGCOLOUR);
	}

	if (LineNum == 7) {
		ST7789_WriteString(0, SPACING*6, "HKi:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(80, SPACING*6, MessageKiHigh2, FONT, TXTCOLOUR, SELCOLOUR);
	} else {
		ST7789_WriteString(0, SPACING*6, "HKi:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(80, SPACING*6, MessageKiHigh2, FONT, TXTCOLOUR, BGCOLOUR);
	}

	if (LineNum == 8) {
		ST7789_WriteString(0, SPACING*7, "HKd:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(80, SPACING*7, MessageKdHigh2, FONT, TXTCOLOUR, SELCOLOUR);
	} else {
		ST7789_WriteString(0, SPACING*7, "HKd:", FONT, TXTCOLOUR, BGCOLOUR);
		ST7789_WriteString(80, SPACING*7, MessageKdHigh2, FONT, TXTCOLOUR, BGCOLOUR);
	}
}

int menu_lvl3_speedPID_stepSize() {
	while (1) {
		menu_lvl3_speedPID_display(2); // Step size option is in line 2

		KeyNum = Key_GetNum();
		if (KeyNum == 1 || KeyNum == 4) {
			return 0;
		}
		if(KeyNum == 2) {
			stepSize *= 10;
			if (stepSize > 1000) stepSize = 1;
		}
		if(KeyNum == 3) {
			stepSize /= 10;
			if (stepSize <= 0.00001) stepSize = 1;
		}
		// if(KeyNum == 4) {
		// 	return 0;
		// }
	}
}

int menu_lvl3_speedPID_constants(int id) {
	while (1) {
		KeyNum = Key_GetNum();
		if (KeyNum == 1 || KeyNum == 4) {
			return 0;
		}
		if(osMutexAcquire(PIDValueMutexHandle, 10) == osOK) {
			if(KeyNum == 2) {
				for (int i = 0; i < 4; i++) {
					pidSpeedConstant[i][0].integral = 0;
					pidSpeedConstant[i][0].prev_error = 0;
					if (id == 1) {
						pidSpeedConstant[i][0].Kp -= stepSize;
					}
					if (id == 2) {
						pidSpeedConstant[i][0].Ki -= stepSize;
					}
					if (id == 3) {
						pidSpeedConstant[i][0].Kd -= stepSize;
					}
					if (id == 4) {
						pidSpeedConstant[i][1].Kp -= stepSize;
					}
					if (id == 5) {
						pidSpeedConstant[i][1].Ki -= stepSize;
					}
					if (id == 6) {
						pidSpeedConstant[i][1].Kd -= stepSize;
					}
				}
			}
			if(KeyNum == 3) {
				for (int i = 0; i < 4; i++) {
					pidSpeedConstant[i][0].integral = 0;
					pidSpeedConstant[i][0].prev_error = 0;
					if (id == 1) {
						pidSpeedConstant[i][0].Kp += stepSize;
					}
					if (id == 2) {
						pidSpeedConstant[i][0].Ki += stepSize;
					}
					if (id == 3) {
						pidSpeedConstant[i][0].Kd += stepSize;
					}
					if (id == 4) {
						pidSpeedConstant[i][1].Kp += stepSize;
					}
					if (id == 5) {
						pidSpeedConstant[i][1].Ki += stepSize;
					}
					if (id == 6) {
						pidSpeedConstant[i][1].Kd += stepSize;
					}
				}
			}
			osMutexRelease(PIDValueMutexHandle);
		}
		// if(KeyNum == 4) {
		// 	return 0;
		// }
		switch(id){
			case 1:{
				menu_lvl3_speedPID_display(3); // Low Kp option is in line 3
				break;
			}
			case 2:{
				menu_lvl3_speedPID_display(4);
				break;
			}
			case 3:{
				menu_lvl3_speedPID_display(5);
				break;
			}
			case 4:{
				menu_lvl3_speedPID_display(6);
				break;
			}
			case 5: {
				menu_lvl3_speedPID_display(7);
				break;
			}
			case 6: {
				menu_lvl3_speedPID_display(8);
				break;
			}
		}
	}
}