#include "stm32f10x.h"
#include <time.h>

#define RCC_APB2_ENR *(volatile unsigned int *)0x40021018

#define GPIOA_CRL *(volatile unsigned int *)0x40010800 //PortA 
#define GPIOA_IDR *(volatile unsigned int *)0x40010808 //PortA + offset

#define GPIOB_CRH *(volatile unsigned int *)0x40010C04 //PortB
#define GPIOB_IDR *(volatile unsigned int *)0x40010C08 //PortB + offset

#define GPIOC_CRL *(volatile unsigned int *)0x40011000 //PortC = LOW
#define GPIOC_CRH *(volatile unsigned int *)0x40011004 //PortC = HIGH
#define GPIOC_IDR *(volatile unsigned int *)0x40011008 //PortC + offset

#define GPIOD_CRL *(volatile unsigned int *)0x40011400 //PortD
#define GPIOD_BSRR *(volatile unsigned int *)0x40011410 
#define GPIOD_BRR *(volatile unsigned int *)0x40011414


int main(void)
{
  RCC_APB2_ENR |= 0x3C;//포트 연결을 위해서 A,B,C,D를 설정해준다
  
  //key1
  GPIOC_CRL &= 0xFFF0FFFF;
  GPIOC_CRL |= 0x00080000;
  
  //key2
  GPIOB_CRH &= 0xFFFFF0FF;
  GPIOB_CRH |= 0x00000800;
    
  //key3
  GPIOC_CRH &= 0xFF0FFFFF;
  GPIOC_CRH |= 0x00800000;
  
  //key4
  GPIOA_CRL &= 0xFFFFFFF0;  
  GPIOA_CRL |= 0x00000008;
  
  //LED
  GPIOD_CRL &= 0x0FF000FF;
  GPIOD_CRL |= 0x10011100; // PORT D 2,3,4,7 LED 포트 접근
    
  GPIOD_BSRR = 0x00000000; // LED 2,3,4,7 다 끔
  GPIOD_BSRR |= 0x0000009C; // LED 2,3,4,7
  
  int mode = 0;
    
  while(1){
    
    // mode1
    if (!(GPIOC_IDR & (1<<4))){
      mode = 1;
    }
    
    // mode2
    if(!(GPIOB_IDR & (1<<10))){
      mode = 2;
    }
     
    // mode3
    if(!(GPIOC_IDR & (1<<13))){
      mode = 3;
    }
    
    // mode4
    if(!(GPIOA_IDR & (1<<0))){
      mode = 4;
    }
      
    
    switch(mode)
    {
    case 1:
      //GPIOD_BRR |= 0x0000009C;
      GPIOD_BSRR |= 0x00840000;
      break;
    case 2:
      //GPIOD_BRR |= 0x0000009C;
      GPIOD_BSRR |= 0x00000084;
      break;
    case 3:
      //GPIOD_BRR |= 0x000009C;
      GPIOD_BSRR |= 0x00180000;
      break;
    case 4:
      //GPIOD_BRR |= 0x0000009C;
      GPIOD_BSRR |= 0x00000018;
      break;
    default:
      //GPIOD_BRR |= 0x00000000;
    }
  }

  return 0;
}
