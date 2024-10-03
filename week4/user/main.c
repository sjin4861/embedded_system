#include "stm32f10x.h"

#define RCC_APB2ENR *(volatile unsigned int *)0x40021018
 


#define GPIOB_CRH *(volatile unsigned int *)0x40010C04
#define GPIOB_IDR *(volatile unsigned int *)0x40010C08

#define GPIOC_CRL *(volatile unsigned int *)0x40011000
#define GPIOC_CRH *(volatile unsigned int *)0x40011004
#define GPIOC_IDR *(volatile unsigned int *)0x40011008      
      
#define GPIOC_BSRR *(volatile unsigned int *)0x40011010  

void delay(){
  int i;
  for(i = 0;i<5000000; i++){}
}
int main(void){
  RCC_APB2ENR = 0x18; //port A, B, C 클럭활성화
  
  GPIOC_CRL &= 0xFFF0FFFF; //Port C 입력모드 활성화
  GPIOC_CRL |= 0x00080000;
  
  GPIOB_CRH &= 0xFFFFF0FF; //Port B 입력모드 활성화
  GPIOB_CRH |= 0x00000800;
  
  GPIOC_CRH &= 0xFF0FFF00;// 3번핀 입력
  GPIOC_CRH |= 0x00800033;
  

  int mode = 0;

    GPIOC_BSRR = 0xFFFF0000;
  
  while(1){

    if(!(GPIOC_IDR & (1<<4))){ 
      mode = 1;
    }
    if(!(GPIOB_IDR & (1<<10))){
     mode = 2;
    }
    if(!(GPIOC_IDR &(1<<13))){
     mode = 3;
    }
    
    if(mode == 1)
    {
      GPIOC_BSRR = 0x00000300;
      delay();
      GPIOC_BSRR = 0x03000000;
       
    }
    if(mode == 2)
    {
       GPIOC_BSRR = 0x00000100;
       delay();
       GPIOC_BSRR = 0x01000000;
   
    }
      
    if(mode == 3)
    {
       GPIOC_BSRR = 0x00000200;
       delay();
       GPIOC_BSRR = 0x02000000;
    }
    mode = 0;
      
    
  }
  
}