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

  // LED 핀 정리

  
}

void ADC_Configure(void)
{
  ADC_InitTypeDef ADC_InitStructure;
  //연속 변환 모드 비활성화
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  //변환된 데이터 오른쪽 정렬
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  //외부트리거 사용 X
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  //독립 모드로 설정
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  //조도센서를 단일 채널만 사용하기 때문에 1 로 설정
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  //단일 채널을 사용하기 때문에 비활성화 한다.
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  //초기화
  ADC_Init(ADC1, &ADC_InitStructure);
  
  ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 1, ADC_SampleTime_55Cycles5);
  //10 주차와 다른 부분 
  ADC_DMACmd(ADC1, ENABLE);
  
  ADC_Cmd(ADC1, ENABLE);
  ADC_ResetCalibration(ADC1);
  while (ADC_GetResetCalibrationStatus(ADC1));
  ADC_StartCalibration(ADC1);
  while (ADC_GetCalibrationStatus(ADC1));
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  
}

int main() {

 SystemInit();
 RCC_Configure();
 GPIO_Configure();
 ADC_Configure();
 DMA_Configure();
 // -----------------------------------함수를 이용해서 설정을 해준다
 LCD_Init();
 Touch_Configuration();
 Touch_Adjust();
 LCD_Clear(WHITE);
 
 uint32_t threshold = 3000;
 
 int flag = 0;
 
 while (1)
 {
    //조도센서로 읽어온 값이 더 크다면??
   if(ADC_Value[0] > threshold) {
     if (!flag){
      //배경색을 회색으로 변경
      LCD_Clear(GRAY);
      flag = 1;
     }
     LCD_ShowNum(30, 50, ADC_Value[0], 4, YELLOW, BLACK);
     
   }
   //조도센서로 읽어온 값이 더 작다면?
   else {
     if (flag){
      //배경색을 흰색으로 변경
      LCD_Clear(WHITE);
      flag = 0;
     }
     LCD_ShowNum(30, 50, ADC_Value[0], 4, YELLOW, BLACK);
   }
 }
 return 0;
}