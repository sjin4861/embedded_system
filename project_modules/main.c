#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_tim.h"
#include "misc.h"
#include <stdio.h>
#include <string.h>

/******************************************************************************
 * 프로젝트 개요:
 *  - STM32F10x 기반 주차 관리 시스템
 *  - 초음파 센서, 압력 센서, 스텝모터, 블루투스 모듈 등 사용
 *  - USART1(PC 연결), USART2(블루투스)
 *  - TIM1으로 초음파 Echo 시간 측정, TIM2로 1초 주기 압력센서 폴링
 *  - EXTI, NVIC 설정(일부 생략), ADC 사용
 *  - car_presence[3][3]: 주차 공간 (3x3) 차량 유무
 *  - current_floor[3]: 각 열의 현재 층(0~2)
 ******************************************************************************/

/******************************************************************************
 * 전역 변수/매크로
 * ---------------------------------------------------------------------------
 * - bluetooth_rx_buffer[]: 블루투스에서 수신한 문자열 저장
 * - bluetooth_rx_index: 수신 버퍼 인덱스
 * - enter_trigger, out_trigger: 압력센서 임계치 넘었을 때(입구/출구) 세팅되는 플래그
 * - car_presence[][]: 주차장 3x3 공간에 대한 차량 유무(1:차있음/0:없음)
 * - current_floor[]: 각 열(column)이 지상0층~2층 중 어디에 있는지
 * - Motor_SetSteps(): 스텝모터 제어
 * - Ultrasonic_MeasureDistance(): 초음파 센서 거리 계산
 * - HandleCarEnter(), HandleOutTrigger(): 입차/출차 처리 함수
 * - HandleCarOut(): “OUT r c” 명령 시 차량 제거 및 모터 이동
 * - SendCarPresenceStatus(): “SHOW” 명령 시 주차 현황을 PC/BT로 송신
 ******************************************************************************/


/******************************************************************************
 * 함수: RCC_Configure(), GPIO_Configure(), ADC_Configure(), TIM1_Configure(),
 *       TIM2_Configure(), USART1_Init(), USART2_Init(), NVIC_Configure() ...
 * ---------------------------------------------------------------------------
 * - 각종 주변장치를 초기화
 * - RCC_Configure(): 클록 활성화, GPIO & USART & TIM 등에 대한 RCC 설정
 * - GPIO_Configure(): 핀 모드 설정 (LED, 초음파 trig/echo, USART 등)
 * - ADC_Configure(): ADC 초기화, 보정
 * - TIM1_Configure(): TIM1을 활성화하고, overflow 인터럽트 설정 (Echo 측정용)
 * - TIM2_Configure(): 1초 주기(오버플로)에 인터럽트 발생 → 압력센서 폴링
 * - USART1_Init(): PC 통신용(9600bps 등)
 * - USART2_Init(): 블루투스 통신용(9600bps 등)
 * - NVIC_Configure(): USART1, USART2, ADC 등 우선순위 설정
 * - NVIC_TIM2_Configure(): TIM2 인터럽트 우선순위 설정
 ******************************************************************************/

/******************************************************************************
 * 메인 함수: main()
 * ---------------------------------------------------------------------------
 * 1) 시스템/주변장치 초기화 (SystemInit, RCC, GPIO, ADC, TIM1, TIM2, USART 등)
 * 2) LED 초기 상태(GREEN)
 * 3) 무한루프:
 *     - if (enter_trigger) → HandleCarEnter() 입차 처리
 *     - if (out_trigger)   → HandleOutTrigger() 출차 처리
 *     - 간단한 delay(1000000)
 * 4) 나머지 명령("SHOW", "TEST", "OUT")는 USART2 인터럽트에서 처리(문자열 파싱)
 ******************************************************************************/

/* =============================================================================
   하드웨어 구성 요소:
   1. 초음파 센서 11개 (3:입구, 4~14: 3x3 주차공간, 15:출구)
   2. 압력 센서 4개
   3. 신호등 LED 3개 (각각 Green/Yellow/Red)
   4. 스텝모터 4개
   5. 블루투스 모듈 1개

   핀 매핑:
   ----------------------------------------------------------------
   초음파 센서 핀 매핑
   입구 : 3
     |  4   7   8 |
     |  9   10   11 |
     |  12   13   14 |
   출구 : 15

   초음파센서 Trig 핀: PC3 ~ PC15 (C5, 6을 D와 통일하기 위해 제거)
   초음파센서 Echo 핀: PD3 ~ PD15 (D5, 6을 블루투스에 사용)
   ----------------------------------------------------------------
   신호등 LED 핀 매핑
     | 0   1   2 |

   신호등 0 GREEN : PB0
   신호등 0 YELLOW: PB1
   신호등 0 RED   : PB2

   신호등 1 GREEN : PB3
   신호등 1 YELLOW: PB4
   신호등 1 RED   : PB5

   신호등 2 GREEN : PB6
   신호등 2 YELLOW: PB7
   신호등 2 RED   : PB8
   ----------------------------------------------------------------
   블루투스 모듈 핀 매핑 (USART1 사용)
   TX 핀: PA9 (USART1_TX)
   RX 핀: PA10 (USART1_RX)

   (USART2 사용)
   TX 핀: PD5 (USART2_TX)
   RX 핀: PD6 (USART2_RX)
   ----------------------------------------------------------------
   스텝모터 모듈 핀 매핑

   입구 : 0
     | 1   2   3 |

   모터 0 IN1~IN4 : PE0~PE3
   모터 1 IN1~IN4 : PE4~PE7
   모터 2 IN1~IN4 : PE8~PE11
   모터 3 IN1~IN4 : PE12~PE15
   ----------------------------------------------------------------
   압력센서 모듈 핀 매핑 (ADC1 채널 사용)

   출구 : 0
   |1   2   3|

   압력센서 0 : PA0 (ADC_Channel_0)
   압력센서 1 : PA1 (ADC_Channel_1)

   제약사항:
   - 반드시 인터럽트 사용: 예) 초음파 에코 신호나 압력센서 변화에 대한 EXTI 인터럽트
   - 블루투스 모듈 사용

   이 코드는 전체 틀만 제공하며, 실제 동작을 위해서는 추가적인 설정과 로직 보완 필요
   ============================================================================= */

//============================ 전역 변수 및 매크로 정의 ==========================

// LED 핀 정의
#define LED0_GREEN_PIN GPIO_Pin_0
#define LED0_YELLOW_PIN GPIO_Pin_1
#define LED0_RED_PIN GPIO_Pin_2

#define LED1_GREEN_PIN GPIO_Pin_13
#define LED1_YELLOW_PIN GPIO_Pin_14
#define LED1_RED_PIN GPIO_Pin_15

#define LED2_GREEN_PIN GPIO_Pin_6
#define LED2_YELLOW_PIN GPIO_Pin_7
#define LED2_RED_PIN GPIO_Pin_8

#define LED_PORT GPIOB

#define LED_COLOR_GREEN 0
#define LED_COLOR_YELLOW 1
#define LED_COLOR_RED 2

// 초음파 센서 수
#define ULTRASONIC_COUNT 11
#define DISTANCE_THRESHOLD 20.0f
#define CMD_BUFFER_SIZE 100

// 블루투스 수신 버퍼
char bluetooth_rx_buffer[5000];
volatile uint8_t bluetooth_rx_index = 0;

// 차량 감지 관련 추가 전역 변수
volatile uint8_t carDetectionActive = 0;     // 입구 감지 활성화 여부
volatile uint8_t carDetectionCount = 0;      // 5초 측정을 위한 카운트(=TIM1_IRQHandler 횟수)
volatile uint8_t carDetected = 0;            // 실제로 차가 감지되었는지

// 주차공간 차 유무 저장 (3x3)
// 1: 차 있음, 0: 차 없음
uint8_t car_presence[3][3] = {0};

// 각 열의 1행이 현재 몇 층에 위치했는가를 나타내는 변수
/*
0층이 최초의 위치라고 생각할 것 / 각각 0~2의 값을 가져야함

| o o o |
| x o x |
| x x x |
-> {1, 2, 1}

| o x x |
| x x x |
| x x x |
-> {1, 0, 0}
*/
int current_floor[3] = {0, 0, 0}; 

// 스텝모터 시퀀스 (단순 예제)
const uint8_t step_sequence[8][4] = {
    {1, 0, 0, 0},
    {1, 1, 0, 0},
    {0, 1, 0, 0},
    {0, 1, 1, 0},
    {0, 0, 1, 0},
    {0, 0, 1, 1},
    {0, 0, 0, 1},
    {1, 0, 0, 1}
};

// 초음파 센서 trig/echo 핀 배열
uint16_t ultrasonic_trig_pins[ULTRASONIC_COUNT] = {
    GPIO_Pin_3, GPIO_Pin_4, GPIO_Pin_7, GPIO_Pin_8, GPIO_Pin_9,
    GPIO_Pin_10, GPIO_Pin_11, GPIO_Pin_12, GPIO_Pin_13, GPIO_Pin_14, GPIO_Pin_15
};

uint16_t ultrasonic_echo_pins[ULTRASONIC_COUNT] = {
    GPIO_Pin_3, GPIO_Pin_4, GPIO_Pin_7, GPIO_Pin_8, GPIO_Pin_9,
    GPIO_Pin_10, GPIO_Pin_11, GPIO_Pin_12, GPIO_Pin_13, GPIO_Pin_14, GPIO_Pin_15
};

#define ULTRASONIC_TRIG_PORT GPIOC
#define ULTRASONIC_ECHO_PORT GPIOD

// 모터 핀 정의 (4개 모터)
uint16_t motor_pins[4][4] = {
    {GPIO_Pin_0, GPIO_Pin_1, GPIO_Pin_2, GPIO_Pin_3},    // 모터0
    {GPIO_Pin_4, GPIO_Pin_5, GPIO_Pin_6, GPIO_Pin_7},    // 모터1
    {GPIO_Pin_8, GPIO_Pin_9, GPIO_Pin_10, GPIO_Pin_11},  // 모터2
    {GPIO_Pin_12, GPIO_Pin_13, GPIO_Pin_14, GPIO_Pin_15} // 모터3
};

// 압력센서 트리거
int enter_trigger = 0;
int out_trigger = 0;

//============================ 함수 프로토타입 선언 ============================
void RCC_Configure(void);
void GPIO_Configure(void);
void TIM1_Configure(void);
void TIM2_Configure(void);
void ADC_Configure(void);
void NVIC_Configure(void);
void EXTI_Configure(void);
void USART1_Init(void);
void USART2_Init(void);
void TIM3_Configure(void);

void LED_SetColor(uint8_t led_num, uint8_t color);
void LED_UpdateByCarPresence(void);
void Motor_SetSteps(int motor_index, int rotation, int direction);

float Ultrasonic_MeasureDistance(uint8_t sensor_index);
void Ultrasonic_Trigger(uint8_t sensor_index);

void delay(int);
void delay_us(uint32_t);
void SetColumnFloor(int col, int newFloor);
void HandleCarEnter(void);
void HandleOutTrigger(void);
void HandleCarOut(int row_in, int col_in);

// USART 통신용
void SendCarPresenceStatus(void);
void USART_SendString(USART_TypeDef* USARTx, const char* str);


//============================ 함수 구현부 ============================
void RCC_Configure(void) {

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
    
    // ADC1 (APB2), TIM1 (APB2)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    // USART1 (APB2), USART2 (APB1)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

     // AFIO (External Interrupt 등)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
}

void GPIO_Configure(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

    // LED 핀 출력 설정
    GPIO_InitStructure.GPIO_Pin = LED0_GREEN_PIN | LED0_YELLOW_PIN | LED0_RED_PIN |
                                  LED1_GREEN_PIN | LED1_YELLOW_PIN | LED1_RED_PIN |
                                  LED2_GREEN_PIN | LED2_YELLOW_PIN | LED2_RED_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(LED_PORT, &GPIO_InitStructure);

    // ADC용 압력센서 핀 설정 (PA0~PA1)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; 
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // USART1 TX (PA9), RX (PA10)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;  // TX
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // USART2 TX: PD5 (AF_PP), USART2 RX: PD6 (IN_FLOATING)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 스텝모터 핀 (PE0~PE15) 모두 출력
    GPIO_InitStructure.GPIO_Pin = 0xFFFF; // PE0~PE15
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    // 초음파 센서 Trig (PC3~PC15 : 출력)
    GPIO_InitStructure.GPIO_Pin = 0;
    for (int i = 0; i < ULTRASONIC_COUNT; i++) {
        GPIO_InitStructure.GPIO_Pin |= ultrasonic_trig_pins[i];
    }
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(ULTRASONIC_TRIG_PORT, &GPIO_InitStructure);

    // 초음파 센서 Echo (PD3~PD15 : 입력)
    GPIO_InitStructure.GPIO_Pin = 0;
    for (int i = 0; i < ULTRASONIC_COUNT; i++) {
        GPIO_InitStructure.GPIO_Pin |= ultrasonic_echo_pins[i];
    }
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(ULTRASONIC_ECHO_PORT, &GPIO_InitStructure);
}

void TIM1_Configure(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    // TIM1 설정 (1μs 단위로 동작하도록 설정)
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;       // 1MHz로 작동 (72MHz / 72)
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // 카운터 증가 모드
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;          // 최대 카운터 값 (65535)
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; // 기본 클럭 분주 없음
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    TIM_Cmd(TIM1, ENABLE); // TIM1 활성화
}

void TIM2_Configure(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    // 1) RCC에서 TIM2 클록 활성화
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    // 2) TIM2 기본 설정
    //    주파수: (APB1=36MHz) / (Prescaler+1) = 1kHz
    //    Period(ARR)=1000 → 1초마다 오버플로
    TIM_TimeBaseStructure.TIM_Prescaler = 36000 - 1;  // 프리스케일러 값
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
    TIM_TimeBaseStructure.TIM_Period = 1000 - 1;      // 1초 주기 (1kHz * 1000)
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;  // TIM2에는 사용 안 함
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    // 3) TIM 업데이트 이벤트(오버플로) 인터럽트 허용
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    // 4) TIM2 시작
    TIM_Cmd(TIM2, ENABLE);
}

void TIM3_Configure(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    // 1) TIM3 클럭 활성화
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    // 2) TIM3 기본 설정
    //    주파수: (APB1=36MHz) / (Prescaler+1) = 1kHz
    //    Period(ARR)=1000 → 1초마다 오버플로
    TIM_TimeBaseStructure.TIM_Prescaler = 36000 - 1;  // 프리스케일러 값
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
    TIM_TimeBaseStructure.TIM_Period = 1000 - 1;      // 1초 주기 (1kHz * 1000)
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;  // TIM3에는 사용 안 함
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    // 3) TIM 업데이트 이벤트(오버플로) 인터럽트 허용
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

    // 4) TIM3 시작
    TIM_Cmd(TIM3, ENABLE);
}

void NVIC_TIM3_Configure(void) {
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // 우선순위
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void USART1_Init(void)
{
    USART_InitTypeDef USART1_InitStructure;

    USART_Cmd(USART1, ENABLE);

    USART1_InitStructure.USART_BaudRate = 9600;
    USART1_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART1_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART1_InitStructure.USART_Parity = USART_Parity_No;
    USART1_InitStructure.USART_StopBits = USART_StopBits_1;
    USART1_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART1, &USART1_InitStructure);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

void USART2_Init(void)
{
    USART_InitTypeDef USART2_InitStructure;

    USART_Cmd(USART2, ENABLE);

    USART2_InitStructure.USART_BaudRate = 9600;
    USART2_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART2_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART2_InitStructure.USART_Parity = USART_Parity_No;
    USART2_InitStructure.USART_StopBits = USART_StopBits_1;
    USART2_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART2, &USART2_InitStructure);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
}

void ADC_Configure(void) {
    ADC_InitTypeDef ADC_InitStructure;

    // ADC1 초기화 설정
    ADC_InitStructure.ADC_Mode               = ADC_Mode_Independent;  
    ADC_InitStructure.ADC_ScanConvMode       = DISABLE;        // 스캔 모드
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;        // 연속 변환
    ADC_InitStructure.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign          = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel       = 1;             // 변환할 채널 수: 2
    ADC_Init(ADC1, &ADC_InitStructure);

    // 채널 순서 설정
    // (Rank=1)에 PA0 -> ADC_Channel_0
    // (Rank=2)에 PA1 -> ADC_Channel_1
    // ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_71Cycles5);
    // ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_71Cycles5);

    // **EOC(End of Conversion) 인터럽트 활성화**
    // ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);

    // ADC 활성화 및 캘리브레이션
    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));

    // ADC 변환 시작 (연속 모드이므로 자동으로 순차 변환)
    // ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void NVIC_Configure(void) {
    NVIC_InitTypeDef NVIC_InitStructure;
	
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

    // USART1 IRQ
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // USART2 IRQ
    NVIC_EnableIRQ(USART2_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // ADC1_2 IRQ
    
    NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

}

// TIM2 인터럽트 설정
void NVIC_TIM2_Configure(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // 우선순위
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


void LED_SetColor(uint8_t led_num, uint8_t color) {
    // led_num: 0,1,2 / color: 0=Green,1=Yellow,2=Red
    switch (led_num) {
        case 0:
            GPIO_ResetBits(LED_PORT, LED0_GREEN_PIN | LED0_YELLOW_PIN | LED0_RED_PIN);
            break;
        case 1:
            GPIO_ResetBits(LED_PORT, LED1_GREEN_PIN | LED1_YELLOW_PIN | LED1_RED_PIN);
            break;
        case 2:
            GPIO_ResetBits(LED_PORT, LED2_GREEN_PIN | LED2_YELLOW_PIN | LED2_RED_PIN);
            break;
    }

    uint16_t pin = 0;
    if (led_num == 0) {
        if (color == 0) pin = LED0_GREEN_PIN;
        else if (color == 1) pin = LED0_YELLOW_PIN;
        else pin = LED0_RED_PIN;
    } else if (led_num == 1) {
        if (color == 0) pin = LED1_GREEN_PIN;
        else if (color == 1) pin = LED1_YELLOW_PIN;
        else pin = LED1_RED_PIN;
    } else {
        if (color == 0) pin = LED2_GREEN_PIN;
        else if (color == 1) pin = LED2_YELLOW_PIN;
        else pin = LED2_RED_PIN;
    }
    GPIO_SetBits(LED_PORT, pin);
}

void LED_UpdateByCarPresence(void) {
    for (int col = 0; col < 3; col++) {
        int count = 0;
        for (int row = 0; row < 3; row++) {
            if (car_presence[row][col] == 1) count++;
        }
        if (count >= 2) {
            LED_SetColor(col, LED_COLOR_RED); // Red
        } else if (count == 0) {
            LED_SetColor(col, LED_COLOR_GREEN); // Green
        } else {
            LED_SetColor(col, LED_COLOR_YELLOW); // Yellow
        }
    }
}

void Motor_SetSteps(int motor_index, int rotation, int direction) {
    uint32_t microseconds_per_minute = 60000000;
    uint32_t total_steps = 4096;  // 1회전당 스텝 수
    uint32_t total_rotation = total_steps * rotation; // 총회전 수
    uint32_t rpm = 18; // 기본 RPM 설정
    uint32_t idle_time = microseconds_per_minute / (total_steps * rpm);

    for (uint32_t step = 0; step < total_rotation; step++) {
        int sequence_index = (direction == -1) ? (7 - (step % 8)) : (step % 8);
        // 단계에 맞게 핀 설정
        for (int pin = 0; pin < 4; pin++) {
            if (step_sequence[sequence_index][pin])
                GPIOE->BSRR = motor_pins[motor_index][pin];
            else
                GPIOE->BRR = motor_pins[motor_index][pin];
        }
        // 딜레이
        delay_us(idle_time);
    }
}

void Ultrasonic_Trigger(uint8_t sensor_index) {
    GPIO_SetBits(ULTRASONIC_TRIG_PORT, ultrasonic_trig_pins[sensor_index]);
    for(volatile int i=0; i<720; i++); // 약 10us 가정
    GPIO_ResetBits(ULTRASONIC_TRIG_PORT, ultrasonic_trig_pins[sensor_index]);
}

void Trig(uint8_t sensor_index) {
    GPIO_SetBits(GPIOC, ultrasonic_trig_pins[sensor_index]);  // Trig 핀 HIGH
    delay_us(10);                      // 10μs 유지
    GPIO_ResetBits(GPIOC, ultrasonic_trig_pins[sensor_index]); // Trig 핀 LOW
}

float Ultrasonic_MeasureDistance(uint8_t sensor_index) {
    // 초음파 거리 측정 로직 필요
    // 여기서는 틀만 제공

  uint16_t start_time = 0, stop_time = 0, echo_time = 0;
    float distance = 0.0;

    Trig(sensor_index); // Trig 신호 송출

    // Echo 핀 HIGH 상태 대기
    while (GPIO_ReadInputDataBit(GPIOD, ultrasonic_echo_pins[sensor_index]) == 0);

    // Echo 핀 HIGH 시작 시간 기록
    start_time = TIM_GetCounter(TIM1);

    // Echo 핀이 LOW 상태로 전환될 때까지 대기
    while (GPIO_ReadInputDataBit(GPIOD, ultrasonic_echo_pins[sensor_index]) == 1);

    // Echo 핀 HIGH 종료 시간 기록
    stop_time = TIM_GetCounter(TIM1);

    // Echo 핀의 HIGH 지속 시간 계산
    if (stop_time >= start_time) {
        echo_time = stop_time - start_time;
    } else {
        echo_time = (0xFFFF - start_time) + stop_time; // 타이머 오버플로우 처리
    }

    // 거리 계산 (단위: cm)
    distance = (float)(echo_time * 0.0343) / 2.0; // 속도: 343m/s

    Ultrasonic_Trigger(sensor_index);
    // Echo 측정 로직 필요
    // float distance = 0.0f;

    return distance;
}

// delay_us 함수 구현
void delay_us(uint32_t us) {
    uint32_t count = (SystemCoreClock / 1000000) * us / 5;
    for (; count != 0; count--);
}

// 압력센서 ADC_CHANNEL 설정
uint16_t Read_ADC_Channel(uint8_t channel)
{
    /* 원하는 채널 설정 */
    ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_71Cycles5);

    /* 변환 시작 */
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

    /* 변환 완료 대기 */
    while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));

    /* 변환 결과 반환 */
    return ADC_GetConversionValue(ADC1);
}

/******************************************************************************
 * 인터럽트: USART1_IRQHandler()
 * ---------------------------------------------------------------------------
 * - PC 쪽(USART1)에서 데이터가 들어올 때마다 RXNE 체크
 * - 현재는 단순히 받은 데이터를 USART2(블루투스)로 에코
 * - 주차 시스템 명령을 PC에서 직접 받지는 않음(필요시 로직 확장 가능)
 ******************************************************************************/
void USART1_IRQHandler() {
    uint16_t word;
    if(USART_GetITStatus(USART1,USART_IT_RXNE)!=RESET){
        word = USART_ReceiveData(USART1);
        // USART1에서 받은 데이터를 USART2로 에코
        USART_SendData(USART2, word);       
        USART_ClearITPendingBit(USART1,USART_IT_RXNE);
    }
}

/******************************************************************************
 * 인터럽트: USART2_IRQHandler()  (블루투스)
 * ---------------------------------------------------------------------------
 * - 블루투스(USART2)로부터 바이트 수신 시 RXNE 플래그
 * - '\n' or '\r' 들어오면 하나의 명령어 끝으로 보고, 문자열 파싱
 *   1) "SHOW" → SendCarPresenceStatus() 호출(주차 현황 전송)
 *   2) "TEST r d" → TEST 명령으로 열 r, 방향 d(+/-1) 모터 이동
 *   3) "OUT r c"  → row=r, col=c 차량 제거 HandleCarOut(r,c)
 * - 그 외엔 "Unknown command" or "Invalid"
 * - PC(USART1)로도 에코
 ******************************************************************************/
void USART2_IRQHandler()
{
    if(USART_GetITStatus(USART2, USART_IT_RXNE)!=RESET){
        char c = USART_ReceiveData(USART2);

        // 수신 버퍼 처리
        if (c == '\n' || c == '\r') {
            bluetooth_rx_buffer[bluetooth_rx_index] = '\0';
            bluetooth_rx_index = 0;

            // 여기서 명령 해석
            if (strcmp(bluetooth_rx_buffer, "SHOW") == 0)
            {
                // 주차 현황 전송
                SendCarPresenceStatus();
            }
            else if (strncmp(bluetooth_rx_buffer, "TEST", 4) == 0)
            {
                // TEST r d 형태 파싱
                // 예: "TEST 0 1" → r=0, d=1
                // 예: "TEST 2 -1" → r=2, d=-1
                int col_idx, direction;
                int matches = sscanf(bluetooth_rx_buffer + 4, "%d %d", &col_idx, &direction);
                if (matches == 2) {
                    // col_idx: 0,1,2
                    // direction: +1, -1
                    // 현 current_floor[col_idx]에서 direction만큼 이동
                    int newFloor = current_floor[col_idx] + direction;
                    // 범위 제한(0~2)
                    if (newFloor < 0) newFloor = 0;
                    else if (newFloor > 2) newFloor = 2;

                    SetColumnFloor(col_idx, newFloor);

                    // 메시지
                    char msg[50];
                    sprintf(msg, "TEST col=%d, direction=%d -> newFloor=%d\r\n",
                            col_idx, direction, newFloor);
                    USART_SendString(USART1, msg);
                    USART_SendString(USART2, msg);
                }
            }
            else if (strncmp(bluetooth_rx_buffer, "OUT", 3) == 0)
            {
                // 기존 OUT r c 처리 (핸들링 동일)
                int row_cmd, col_cmd;
                int matches = sscanf(bluetooth_rx_buffer+3, "%d %d", &row_cmd, &col_cmd);
                if (matches == 2) {
                    HandleCarOut(row_cmd, col_cmd);
                } else {
                    USART_SendString(USART1, "[BT] Invalid OUT command format!\r\n");
                    USART_SendString(USART2, "[BT] Invalid OUT command format!\r\n");
                }
            }
        } 
        else {
            // 아직 '\n' or '\r'이 아닐 때는 버퍼에 쌓음
            if (bluetooth_rx_index < CMD_BUFFER_SIZE - 1) {
                bluetooth_rx_buffer[bluetooth_rx_index++] = c;
            }
        }

        // PC 에코
        USART_SendData(USART1, c); 
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);    
    }
}

// USART로 문자열 전송 (PC, BT 겸용)
void USART_SendString(USART_TypeDef* USARTx, const char* str)
{
    while (*str)
    {
        while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);
        USART_SendData(USARTx, (uint16_t)*str++);
    }
}

/******************************************************************************
 * 함수: SendCarPresenceStatus()
 * ---------------------------------------------------------------------------
 * - "SHOW" 명령 처리용
 * - car_presence[row][col] (0~2,0~2)를 순회하여
 *   예) Row0: 1 0 1\r\n
 *       Row1: 0 2 0\r\n
 *       Row2: ...
 *   이런 식 문자열 만들고, PC(USART1), BT(USART2) 모두 전송
 ******************************************************************************/
void SendCarPresenceStatus(void)
{
    // 예: Row0: 1 0 1
    for(int row=0; row<3; row++) {
        char msg[50];
        sprintf(msg, "Row%d: %d %d %d\r\n", row,
                car_presence[row][0],
                car_presence[row][1],
                car_presence[row][2]);
        // PC(USART1) 전송
        USART_SendString(USART1, msg);
        // BT(USART2) 전송
        USART_SendString(USART2, msg);
    }
}

/******************************************************************************
 * 인터럽트: TIM2_IRQHandler()
 * ---------------------------------------------------------------------------
 * - 1초마다 오버플로 이벤트 → 압력센서(ADC) 값을 Read_ADC_Channel()로 읽음
 * - adc_value_0(입구), adc_value_1(출구) > 임계값 시
 *     → enter_trigger=1 / out_trigger=1
 * - 이후 메인 루프에서 HandleCarEnter() / HandleOutTrigger()를 호출
 ******************************************************************************/
void TIM2_IRQHandler(void) {
    volatile uint16_t adc_value_0 = 0, adc_value_1 = 0;

    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

        // 1초에 한 번씩 압력 센서(ADC) 읽기
        adc_value_0 = Read_ADC_Channel(ADC_Channel_0);
        adc_value_1 = Read_ADC_Channel(ADC_Channel_1);

        // 여기서 임계값 비교 등 간단한 처리도 할 수 있음
        if (adc_value_0 > 600) {
            enter_trigger = 1;
        }
        if (adc_value_1 > 600) {
            out_trigger = 1;
        }
    }
}

void delay(int step){
    for (volatile int i = 0; i < step; i++);
}


/******************************************************************************
 * 함수: Motor_SetSteps(), SetColumnFloor()
 * ---------------------------------------------------------------------------
 * - Motor_SetSteps(motor_index, rotation, direction):
 *     * rotation(=회전할 칸수×3등), direction(±1)
 *     * 1칸=rotation=3(예시), total_steps=4096
 *     * 한 스텝마다 step_sequence 인덱스에 맞춰 GPIOE에 ON/OFF → 스텝모터 회전
 * - SetColumnFloor(col, newFloor):
 *     * 현재층(current_floor[col])과 newFloor 차(diff) 계산
 *     * diff>0 → 위, diff<0 → 아래
 *     * Motor_SetSteps(col+1, 3*abs(diff), …)
 *     * current_floor[col] = newFloor
 ******************************************************************************/
void SetColumnFloor(int col, int newFloor)
{
    int diff = newFloor - current_floor[col];
    if (diff == 0) {
        // 이미 목표 층에 있으므로 동작 안 함
        return;
    }
    
    if (diff > 0) {
        // 위로 이동
        // diff칸 이동해야 하므로 rotation = 3 * diff (예시)
        Motor_SetSteps(col+1, 3 * diff, -1);
    } else {
        // 아래로 이동 (diff < 0)
        // 절댓값(-diff)만큼 칸 이동
        Motor_SetSteps(col+1, 3 * (-diff), 1);
    }
    
    // 현재 층 갱신
    current_floor[col] = newFloor;
}

/******************************************************************************
 * 함수: HandleCarEnter()
 * ---------------------------------------------------------------------------
 * - 입구 플래그(enter_trigger=1) 시 호출
 * - current_floor[col] == 0 인 열(즉, 1층인 칸)을 초음파 센서로 측정
 *   -> 거리가 5cm 미만이면 car_presence[row][col] = 1 로 주차 처리
 *   -> LED_UpdateByCarPresence() 호출
 * - 차 1대만 처리 후 종료
 * - delay(1초) 등 간단히 블로킹 (실험용)
 ******************************************************************************/
void HandleCarEnter(void)
{
    // 트리거 한번 처리 후에는 클리어
    // 각 열(col)마다 현재 1층인 행(row)를 파악
      for (int col = 0; col < 3; col++)
      {
          int row = current_floor[col]; // 이 열에서 1층에 놓여있는 행
          // 초음파 센서 인덱스 (기존 코드에서 1~9로 매핑)
          uint8_t sensor_index = row * 3 + (col + 1);

          float distance = Ultrasonic_MeasureDistance(sensor_index);
          // printf("row, col, distance : %d, %d, %.2f\n", row, col, distance);
          // 예: 5cm 이하이면 차가 들어온 것으로 간주
          if (distance < 5.0f && car_presence[row][col] == 0)
          {
              // 새 차 주차
              car_presence[row][col] = 1;
              printf("[Enter] Car detected at row=%d, col=%d\n", row, col);

              // LED 상태 갱신
              LED_UpdateByCarPresence();
              // 한 칸만 주차 처리 후 종료
              enter_trigger = 0;
          }
          // 한 1초 있다가 위로 올려버릴까?
          delay(1000000);
      
    }
    
    enter_trigger = 0;
    
    // 필요하다면 "입장 감지"용 LED 깜빡임 정도만...
    //printf("[HandleCarEnter] Enter trigger received. Car detection will run via TIM1.\n");
}

/******************************************************************************
 * 함수: HandleOutTrigger()
 * ---------------------------------------------------------------------------
 * - 출구 플래그(out_trigger=1) 시 호출
 * - 1층(=row=current_floor[col])에 차가 있으면 위로 한 칸 올림
 *   (SetColumnFloor(col, row+1) 단, row<2 일 때)
 * - LED_UpdateByCarPresence() 미사용 (원하면 추가 가능)
 ******************************************************************************/
void HandleOutTrigger(void)
{
    out_trigger = 0; // 한 번 처리 후 플래그 해제

    // 3개 열에 대해 검사
    for (int col = 0; col < 3; col++)
    {
        int row = current_floor[col];
        // 현재 1층(지상)에 있는 칸(row,col)에 차가 있다면
        if (car_presence[row][col] == 1)
        {
            // 만약 이미 가장 위(= row=2)라면 더 이상 올릴 수 없다고 가정
            // row=0 → row=1로, row=1 → row=2로 올리는 식
            if (row < 2) {
                int newFloor = row + 1; 
                SetColumnFloor(col+1, newFloor);
                printf("[OutTrigger] Moved column=%d from row=%d to row=%d\n", col, row, newFloor);
            }
            else {
                // row=2면 이미 최상층, 더 이상 올릴 필요가 없다고 가정
                printf("[OutTrigger] Column=%d is already top floor (row=2), no move.\n", col);
            }
        }
        else {
            // 차가 없으면 이동할 필요 없음
            // (원한다면 빈 칸을 더 아래로 내린다거나 다른 로직 가능)
            printf("[OutTrigger] Column=%d row=%d is empty, no move.\n", col, row);
        }
    }
}

/******************************************************************************
 * 함수: HandleCarOut(int row_in, int col_in)
 * ---------------------------------------------------------------------------
 * - 블루투스 “OUT r c” 명령 처리용
 * - row_in, col_in은 1-based → 0-based 변환
 * - 해당 위치에 차 있으면 제거, 필요시 열을 0층으로 복귀
 * - LED_UpdateByCarPresence()로 표시
 ******************************************************************************/
void HandleCarOut(int row_in, int col_in)
{
    // 1-based -> 0-based
    int row = row_in - 1;
    int col = col_in - 1;

    // 범위 체크
    if (row < 0 || row > 2 || col < 0 || col > 2) {
        printf("[OUT] Invalid row/col!\n");
        return;
    }

    // 해당 위치에 차가 있는지 확인
    if (car_presence[row][col] == 1)
    {
        // 열(col)이 현재 1층인 행과 원하는 row가 다르다면 이동
        if (current_floor[col] != row) {
            SetColumnFloor(col, row);
        }

        // 차량 제거
        car_presence[row][col] = 0;
        printf("[OUT] Car removed at row=%d, col=%d\n", row, col);

        // 필요하면 다시 0층(=row=0)으로 복귀
        // 프로젝트 요구사항에 따라 다름. 예: 기본적으로 항상 1행(=row=0)을 1층에 두고 싶다면:
        if (current_floor[col] != 0) {
            SetColumnFloor(col, 0);
        }

        // LED 갱신
        LED_UpdateByCarPresence();
    }
    else
    {
        printf("[OUT] No car at row=%d, col=%d\n", row, col);
    }
}


/******************************************************************************
 * 메인 함수: main()
 * ---------------------------------------------------------------------------
 * 1) 시스템/주변장치 초기화 (SystemInit, RCC, GPIO, ADC, TIM1, TIM2, USART 등)
 * 2) LED 초기 상태(GREEN)
 * 3) 무한루프:
 *     - if (enter_trigger) → HandleCarEnter() 입차 처리
 *     - if (out_trigger)   → HandleOutTrigger() 출차 처리
 *     - 간단한 delay(1000000)
 * 4) 나머지 명령("SHOW", "TEST", "OUT")는 USART2 인터럽트에서 처리(문자열 파싱)
 ******************************************************************************/


/******************************************************************************
 * 메인 함수: main()
 * ---------------------------------------------------------------------------
 * 1) 시스템/주변장치 초기화 (SystemInit, RCC, GPIO, ADC, TIM1, TIM2, USART 등)
 * 2) LED 초기 상태(GREEN)
 * 3) 무한루프:
 *     - if (enter_trigger) → HandleCarEnter() 입차 처리
 *     - if (out_trigger)   → HandleOutTrigger() 출차 처리
 *     - 간단한 delay(1000000)
 * 4) 나머지 명령("SHOW", "TEST", "OUT")는 USART2 인터럽트에서 처리(문자열 파싱)
 ******************************************************************************/

//============================ 메인 함수 ============================
int main() {
    uint16_t adc_value_0, adc_value_1;
  
    SystemInit();
    RCC_Configure();
    GPIO_Configure();
    ADC_Configure();
    TIM1_Configure();      // 타이머 초기화 (준식)
    USART1_Init(); // PC
    USART2_Init(); // 블루투스
    NVIC_Configure();
    TIM2_Configure(); // 1초마다 압력센서 읽기
    NVIC_TIM2_Configure();  // TIM2 NVIC 설정
    
    // 초기 LED 상태 (모두 GREEN)
    LED_SetColor(0, LED_COLOR_GREEN);
    LED_SetColor(1, LED_COLOR_GREEN);
    LED_SetColor(2, LED_COLOR_GREEN);
    
    while(1) {
    }

    while(1) {
        adc_value_0 = Read_ADC_Channel(ADC_Channel_0);
        adc_value_1 = Read_ADC_Channel(ADC_Channel_1);
        printf("adc_value_0 : %d\n", adc_value_0);
        printf("adc_value_1 : %d\n", adc_value_1);

        if (adc_value_0 > 400) {
            enter_trigger = 1;
        }

        if (adc_value_1 > 400) {
            out_trigger = 1;
        }

        // 각 초음파 센서(1~9)로 거리 측정하고 일정 거리 이하면 차 있음(1), 아니면 없음(0)
        // sensor_index: 1,2,3  / 4,5,6 / 7,8,9 => 3x3
        // row = (sensor_index-1)/3, col = (sensor_index-1)%3
        if (enter_trigger){
            // 문 개방 -> 이거 안 할거임
            enter_trigger = 0;
            // 1층에 있는 초음파 센서 트리거링
            HandleCarEnter();
        }
        if (out_trigger){
            // 사람이 출구를 나가는 것이 감지가 되는 경우
            out_trigger = 0;
            HandleOutTrigger();
            // 모터 수직 이동, 방금 들어온 차를 보고 모터 index와 방향을 결정해야함
        }
        delay(1000000);
    }
    return 0;
}