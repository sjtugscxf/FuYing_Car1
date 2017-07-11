#include "includes.h"

// ===== Variables ======
//---- GLOBAL ----

U32 wavetimef;
U32 wavetime;
U32 wavetimeus;
int distance;
int distance_tmp;
int distance_last;
int distance_buffer[5];
int distance_ave;
int distance_diff;
int distance_sum;

WaveState waveState;
uint8 wave_lost_cnt = 0;

void StartUltrasound(u8 x){
  if(x)
    PTC->PSOR |= 1<<1;
  else
    PTC->PCOR |= 1<<1;
}

void Wave_Init()
{
  PORTC->PCR[0] |= PORT_PCR_MUX(1) |PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(11);    // CCD2AO接口用作输入，超声波接收
  PORTC->PCR[1] |= PORT_PCR_MUX(1);    // CCD1AO接口用作输出，超声波发送
  PTC->PDDR |= (0x1<<1);
  NVIC_EnableIRQ(PORTC_IRQn);
  NVIC_SetPriority(PORTC_IRQn, NVIC_EncodePriority(NVIC_GROUP, 2, 1)); //PORTC中断服务程序在cam.c中
  waveState = LOST;
}


