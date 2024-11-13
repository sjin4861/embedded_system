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

uint16_t LCD_TEAM_NAME_X = 100;
uint16_t LCD_TEAM_NAME_Y = 100;

uint16_t LCD_BUTTON_X = 0x50;
uint16_t LCD_BUTTON_Y = 0x65;
uint16_t LCD_BUTTON_W = 0x30;
uint16_t LCD_BUTTON_H = 0x30;

uint16_t LCD_STATUS_X = 0x50;
uint16_t LCD_STATUS_Y = 0x55;

uint16_t motorAngle;
uint16_t motorDir;
uint16_t t1;
uint16_t t2;
uint16_t ledOn;

 int color[12] = 
  {WHITE,CYAN,BLUE,RED,MAGENTA,LGRAY,GREEN,YELLOW,BROWN,BRRED,GRAY};


void RCC_Configure(void)
{
   RCC_APB1PeriphClockCmd(RCC_APB1ENR_TIM2EN, ENABLE);
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE); 
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
}

void GPIO_Configure(void)
{
   GPIO_InitTypeDef GPIO_InitStructure;
   uint16_t prescale = 0;
   TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
   TIM_OCInitTypeDef TIM_OCInitStructure;
   // LED 1
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_Init(GPIOD, &GPIO_InitStructure);
   // LED 2
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_Init(GPIOD, &GPIO_InitStructure);
   // PWM motor
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOB, &GPIO_InitStructure);
   
   prescale = (uint16_t)(SystemCoreClock / 1000000)- 1;
   
   TIM_TimeBaseStructure.TIM_Period = 20000- 1;
   TIM_TimeBaseStructure.TIM_Prescaler = prescale;
   TIM_TimeBaseStructure.TIM_ClockDivision = 0;
   TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;
   
   TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
   TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
   TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
   TIM_OCInitStructure.TIM_Pulse = 1500;
   TIM_OC3Init(TIM3, &TIM_OCInitStructure); 
   
   TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
   TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Disable);
   TIM_ARRPreloadConfig(TIM3, ENABLE);
   TIM_Cmd(TIM3, ENABLE);
}

 void TIM_Configure(void)
 {
   TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
   TIM_TimeBaseStructure.TIM_Period = 10000;
   TIM_TimeBaseStructure.TIM_Prescaler = 7200;
   TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
   TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
   TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
   TIM_Cmd(TIM2, ENABLE);
   TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
 }

void Nvic_Init(void)
{
   NVIC_InitTypeDef NVIC_InitStructure;
   NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);
}
 void LED_Toggle(int num)
 {
   uint16_t pin;
   switch (num)
   {
   case 1:
     pin = GPIO_Pin_2;
     break;
   case 2:
     pin = GPIO_Pin_3;
     break;
   default:
     return;
   }
   if (GPIO_ReadOutputDataBit(GPIOD, pin) == Bit_RESET)
   {
     GPIO_SetBits(GPIOD, pin);
   }
   else
   {
     GPIO_ResetBits(GPIOD, pin);
   }
}
void moveMotor()
{
  TIM_OCInitTypeDef TIM_OCInitStructure;
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = motorAngle + 700;
  if (motorDir == 0)
  {
    motorAngle = motorAngle + 100;
    if (motorAngle == 1500)
       motorAngle = 0;
  }
  else
   {
   motorAngle = motorAngle- 100;
   if (motorAngle == 0)
     motorAngle = 1500;
   }
   TIM_OC3Init(TIM3, &TIM_OCInitStructure);
}

void TIM2_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
  {
    // 1초마다 count
    t1++;
    t2++;
    moveMotor();
   if (ledOn == 1)
   {
     // led 1 toggle
     LED_Toggle(1);
     if (t1 % 5 == 0)
     {
     // led 2 toggle
     LED_Toggle(2);
     }
   }
   TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
  }
}
int main() {

SystemInit();
 RCC_Configure();
 GPIO_Configure();
 TIM_Configure();
 Nvic_Init();
 LCD_Init();
 Touch_Configuration();
 Touch_Adjust();
 
 
 uint16_t pos_x, pos_y;
 uint16_t pix_x, pix_y;
 t1 = 0;
 t2 = 0;
 ledOn = 0;
 LCD_Clear(WHITE);
 // team name
 LCD_ShowString(LCD_TEAM_NAME_X, LCD_TEAM_NAME_Y, "WED_Team10", BLUE, WHITE);
 // button
 LCD_DrawRectangle(LCD_BUTTON_X, LCD_BUTTON_Y, LCD_BUTTON_X + LCD_BUTTON_W, LCD_BUTTON_Y + LCD_BUTTON_H);
 LCD_ShowString(LCD_BUTTON_X + (LCD_BUTTON_W / 2), LCD_BUTTON_Y + (LCD_BUTTON_H / 2), "BUT", RED, WHITE);
 while (1)
 {
 if (ledOn == 0)
 {
 LCD_ShowString(LCD_STATUS_X, LCD_STATUS_Y, "OFF", RED, WHITE);
 motorDir = 0;
 }
 else
 {
   LCD_ShowString(LCD_STATUS_X, LCD_STATUS_Y, "ON ", RED, WHITE) ;
   motorDir = 1;
 }
 // get touch coordinate
 Touch_GetXY(&pos_x, &pos_y, 1);
 Convert_Pos(pos_x, pos_y, &pix_x, &pix_y);
if( 
   pix_x >= LCD_BUTTON_X &&
 pix_x <= LCD_BUTTON_X + LCD_BUTTON_W &&
 pix_y >= LCD_BUTTON_Y &&
 pix_x <= LCD_BUTTON_Y + LCD_BUTTON_H)
{
  //button 눌림
  ledOn = !ledOn;
}
}
}