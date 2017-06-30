#include "includes.h"

// ===== Variables ======
//---- GLOBAL ----

U32 wavetimef;
U32 wavetime;
U32 wavetimeus;
int distance;
int distance_tmp;
int distance_last;

void StartUltrasound(u8 x){
  if(x)
    PTB->PSOR |= 1<<23;
  else
    PTB->PCOR |= 1<<23;
}

void Wave_Init()
{
  PORTB->PCR[11] |= PORT_PCR_MUX(1) |PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(11);    // CCD2AO接口用作输入，超声波接收
  PORTB->PCR[23] |= PORT_PCR_MUX(1);    // CCD1AO接口用作输出，超声波发送
  PTB->PDDR |= (0x1<<23);
  NVIC_EnableIRQ(PORTB_IRQn);
  NVIC_SetPriority(PORTB_IRQn, NVIC_EncodePriority(NVIC_GROUP, 2, 1)); //PORTC中断服务程序在cam.c中
  distance = 400;
  distance_last = 500;
}

void PORTB_IRQHandler(){
  if((PORTB->ISFR)&PORT_ISFR_ISF(1 << 11)){  //超声波接收中断
    PORTB->ISFR |= PORT_ISFR_ISF(1 << 11);
    if((PTB->PDIR>>11)&1)  
    {
      wavetimef=PIT2_VAL();
    }
    else 
    {
        wavetime=wavetimef-PIT2_VAL();
        wavetimeus = wavetime / (g_bus_clock/1000000); //1us
        distance_tmp=wavetimeus*34/200;    //距离单位毫米
        
        //if(distance_tmp > 500)
          //distance_tmp = 500;
        if((distance_tmp - distance_last) >= 80)
          distance_tmp = distance_last;

        distance=distance_tmp;
        distance_last = distance;
    }
  }
}
