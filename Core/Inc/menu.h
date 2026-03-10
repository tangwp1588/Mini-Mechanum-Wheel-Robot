#pragma once

#define BGCOLOUR DARKBLUE // Background Colour
#define FONT Font_11x18   // Font
#define TXTCOLOUR WHITE   // Text Colour
#define SPACING 26        // Line Spacing

#define SELCOLOUR GREY    // Colour when the option being selected

void mainMenu();
void menu_lvl1_display(int LineNum);
int menu_lvl1();

void balancingPID_printf();
void menu_lvl2_balancingPID_display(int LineNum);
int menu_lvl2_balancingPID(); //PID1

void speedPID_printf();
void menu_lvl2_speedPID_display(int LineNum);
int menu_lvl2_speedPID(); //PID2

void orientationPID_printf();
void menu_lvl2_orientationPID_display(int LineNum);
int menu_lvl2_orientationPID(); //PID3

void menu_lvl3_balancingPID_display(int LineNum);
int menu_lvl3_balancingPID_stepSize();
int menu_lvl3_balancingPID_constants(int id);

void menu_lvl3_speedPID_display(int LineNum);
int menu_lvl3_speedPID_stepSize();
int menu_lvl3_speedPID_constants(int id);

void menu_lvl3_orientationPID_display(int LineNum);
int menu_lvl3_orientationPID_stepSize();
int menu_lvl3_orientationPID_constants(int id);