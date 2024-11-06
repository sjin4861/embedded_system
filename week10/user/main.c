#include "stm32f10x.h"
 #include "core_cm3.h"
 #include "misc.h"
 #include "stm32f10x_gpio.h"
 #include "stm32f10x_rcc.h"
 #include "stm32f10x_usart.h"
 #include "stm32f10x_adc.h"
 #include "touch.h"
 #include "lcd.h"
 //색상 배열 정의

uint16_t value=0;
uint16_t x;
uint16_t y;
uint16_t con_x;
uint16_t con_y;



 int color[12] = 
  {WHITE,CYAN,BLUE,RED,MAGENTA,LGRAY,GREEN,YELLOW,BROWN,BRRED,GRAY};


void RCC_Configure(void)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
}

void GPIO_Configure()
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
}

void ADC_Configure(void)
{
  ADC_InitTypeDef ADC_InitStructure;
  
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  
  ADC_Init(ADC1, &ADC_InitStructure);
  
  ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 1, ADC_SampleTime_239Cycles5);
  ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
  ADC_Cmd(ADC1, ENABLE);
  ADC_ResetCalibration(ADC1);
  while(ADC_GetResetCalibrationStatus(ADC1));
  
  ADC_StartCalibration(ADC1);
  while(ADC_GetCalibrationStatus(ADC1));
  
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
                       
}
void NVIC_Configure(void){
  NVIC_InitTypeDef NVIC_InitStructure;
  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  
  NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
  NVIC_Init(&NVIC_InitStructure);
}
void ADC1_2_IRQHandler() {
  
  if(ADC_GetITStatus(ADC1, ADC_IT_EOC) != RESET){
    value = ADC_GetConversionValue(ADC1);
  }
  ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
}

int main() {

SystemInit();
 RCC_Configure();
 GPIO_Configure();
 ADC_Configure();
 NVIC_Configure();
 // -----------------------------------
 LCD_Init();
 Touch_Configuration();
 Touch_Adjust();
 LCD_Clear(WHITE);
 
 
 
 while(1){
 
 Touch_GetXY(&x, &y, 1);
 Convert_Pos(x, y, &con_x, &con_y);
 ADC_SoftwareStartConvCmd(ADC1, ENABLE);
 LCD_DrawCircle(con_x,con_y,5);
 LCD_ShowString(30, 30, "WED_Team10", BLACK, WHITE);
 LCD_ShowNum(0, 50, con_x, 3, BLACK, WHITE);
 LCD_ShowNum(0, 100, con_y, 3, BLACK, WHITE);
 LCD_ShowNum(0, 150, value, 3, BLACK, WHITE);
 
   
}
}