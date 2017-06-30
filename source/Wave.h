#ifndef WAVE_H
#define WAVE_H

// ===== Global Variables =====
extern int distance;   //  mm
extern U32 wavetimef;
extern U32 wavetime;
extern U32 wavetimeus;
extern int distance_tmp;
extern int distance_last;

// ======= APIs =======
void StartUltrasound(u8 x);  //1开启，0关闭

  // Init
void Wave_Init();       //在main最前面初始化


#endif