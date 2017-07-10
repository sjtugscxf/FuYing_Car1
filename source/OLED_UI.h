#ifndef OLED_UI_H
#define OLED_UI_H

#define Rows 30
#define Pages 6

//=== Global variables===
extern u8 oled_menu;


void UI_SystemInfo();
void displayMenu();//menu==0
void displayParameters();//menu==1
void displayCamera();//menu==2
void displayDebug();//menu==3

void drawCam(bool(*isTarget)(u8 x));//二值化
//void drawCam2(bool(*isTarget)(u8 x));//透视变换
bool isWhite(u8 x);

void drawRoad();//画road_B的mid left right
void drawJump();//画jump[][]以及其延长线

#endif