/*
Arthor : Qian Qiyang (KisaragiAyanoo@twitter)
Date : 2015/12/01
License : MIT
*/

#include "includes.h"



// === Receive ISR ===
void UART3_IRQHandler(void){
   uint8 tmp = UART_GetChar();
  /*
   if (tmp == 'a'){
     remote_state = 2;
     UART_SendString("turn left");//
   }else if (tmp == 'd'){
     remote_state = 3;
     UART_SendString("turn right");//
   }else if (tmp == 'w'){
     remote_state = 1;
     UART_SendString("go ahead");
   }else if (tmp == 's'){
     remote_state = 0;
     UART_SendString("stop");
   }
   */
  /* if(tmp=='0')
     remote_state=bt_no;
   else if(tmp=='1')
     remote_state=bt_prepare;
   else if(tmp=='2')
     remote_state=bt_start;
   else if(tmp=='3')
     remote_state=bt_finish;
   else if(tmp=='4')
     remote_state=bt_forbid;
   */
  switch(car_type){
  case leader:
    switch(tmp){
    case 'A':
      bt_ok=1;
      overtake_state=in_overtake;
      break;
    case 'b':
      car_type=follower;
      overtake_state=no_overtake;
      UART_SendChar('B');
      break;
    case 'C':
      bt_ok=1;
      //bt_stop=1;其实不需要反馈就可以停车，如果想要缩短两车到终点的时间差，则需要利用这个反馈！！！！！！！！
    }
    break;
    
  case follower:
    switch(tmp){
    case 'a':
      if(bt_ok==0){            //只有后车完成了之前的state才可以接受新的超车请求
        overtake_state=in_overtake;
        UART_SendChar('A');
        bt_ok=1;
      }
    case 'B':
      car_type==leader;
      overtake_state=no_overtake;
      break;
    case 'c':
      bt_ok=1;
      bt_stop=1;
      UART_SendChar('C');
      break;
    default:break;
    }
    break;
    
  default:break;
  /*
   else if(tmp=='b')
     overtake_state=0;
   else if(tmp=='c')
     bt_stop=1;
   */
  }
}

// ======== APIs ======

// ---- Ayano's Format ----

void UART_SendDataHead(){
  while(!(UART3->S1 & UART_S1_TDRE_MASK));
  UART3->D = 127;
}
void UART_SendData(s16 data){
  uint8 neg=0;
  uint8 num;
  if(data<0){
    neg = 1;
    data = -data;
  }
  num = data%100;
  
  while(!(UART3->S1 & UART_S1_TDRE_MASK));
  UART3->D = neg?(49-data/100):(50+data/100);
  
  while(!(UART3->S1 & UART_S1_TDRE_MASK));
  UART3->D = num;
}

// ----- Basic Functions -----

// Read 
int8 UART_GetChar(){
  while(!(UART3->S1 & UART_S1_RDRF_MASK));
  return UART3->D;
}

// Send
void UART_SendChar(uint8 data){
  while(!(UART3->S1 & UART_S1_TDRE_MASK));
  UART3->D = data;
}

void UART_SendString(const char *p) {
  while (*p)
    UART_SendChar(*p++);
}

// Init
void UART_Init(u32 baud){
  //UART3
  
  uint16 sbr;
  uint32 busclock = g_bus_clock;

  SIM->SCGC4 |= SIM_SCGC4_UART3_MASK;
  
  PORTE->PCR[4] = PORT_PCR_MUX(3);
  PORTE->PCR[5] = PORT_PCR_MUX(3);
  UART3->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK );  // DISABLE UART FIRST
  UART3->C1 = 0;  // NONE PARITY CHECK
  
  //UART baud rate = UART module clock / (16 × (SBR[12:0] + BRFD))
  // BRFD = BRFA/32; fraction
  sbr = (uint16)(busclock/(16*baud));
  UART3->BDH = (UART3->BDH & 0XC0)|(uint8)((sbr & 0X1F00)>>8);
  UART3->BDL = (uint8)(sbr & 0XFF);
  
  UART3->C4 = (UART3->C4 & 0XE0)|(uint8)((32*busclock)/(16*baud)-sbr*32); 

  UART3->C2 |=  UART_C2_RIE_MASK;
  
  UART3->C2 |= UART_C2_RE_MASK;
  
  UART3->C2 |= UART_C2_TE_MASK; 
  
  NVIC_EnableIRQ(UART3_RX_TX_IRQn); 
  NVIC_SetPriority(UART3_RX_TX_IRQn, NVIC_EncodePriority(NVIC_GROUP, 2, 2));
}