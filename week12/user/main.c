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


volatile uint32_t ADC_Value[1];
 int color[12] = 
  {WHITE,CYAN,BLUE,RED,MAGENTA,LGRAY,GREEN,YELLOW,BROWN,BRRED,GRAY};


void RCC_Configure(void)
{
  
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);//DAM1활성화
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//C port 활성화
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//C port 활성화
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);//C port 활성화
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);//C port 활성화
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);//C port 활성화
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); // ADC 활성화
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
  
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_Init(ADC1, &ADC_InitStructure);
  
  ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 1, ADC_SampleTime_55Cycles5);
  ADC_DMACmd(ADC1, ENABLE);
  
  ADC_Cmd(ADC1, ENABLE);
  ADC_ResetCalibration(ADC1);
  while (ADC_GetResetCalibrationStatus(ADC1));
  ADC_StartCalibration(ADC1);
  while (ADC_GetCalibrationStatus(ADC1));
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  
}
void DMA_Configure(void)
{
  DMA_InitTypeDef DMA_InitStructure;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC_Value;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 4;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
 
  DMA_DeInit(DMA1_Channel1);
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  DMA_Cmd(DMA1_Channel1, ENABLE);
  
}


int main() {

 SystemInit();
 RCC_Configure();
 GPIO_Configure();
 ADC_Configure();
 DMA_Configure();
 // -----------------------------------
 LCD_Init();
 Touch_Configuration();
 Touch_Adjust();
 LCD_Clear(WHITE);
 
 uint32_t threshold = 3000;
 
 int flag = 0;
 
 while (1)
 {
   if(ADC_Value[0] > threshold) {
     if (!flag){
      LCD_Clear(GRAY);
      flag = 1;
     }
     LCD_ShowNum(30, 50, ADC_Value[0], 4, YELLOW, BLACK);
     
   }
   else {
     if (flag){
      LCD_Clear(WHITE);
      flag = 0;
     }
     LCD_ShowNum(30, 50, ADC_Value[0], 4, YELLOW, BLACK);
   }
 }
 return 0;
}