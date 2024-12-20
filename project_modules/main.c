#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_tim.h"
#include "misc.h"
#include <stdio.h>
#include <string.h>

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

#define LED1_GREEN_PIN GPIO_Pin_3
#define LED1_YELLOW_PIN GPIO_Pin_4
#define LED1_RED_PIN GPIO_Pin_5

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
volatile char bluetooth_rx_buffer[100];
volatile uint8_t bluetooth_rx_index = 0;
volatile uint8_t bluetooth_command_received = 0;

// 주차공간 차 유무 저장 (3x3)
// 1: 차 있음, 0: 차 없음
uint8_t car_presence[3][3] = {0};

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
void ADC_Configure(void);
void NVIC_Configure(void);
void EXTI_Configure(void);
void USART1_Init(void);

void LED_SetColor(uint8_t led_num, uint8_t color);
void LED_UpdateByCarPresence(void);
void Motor_SetSteps(int motor_index, int rotation, int direction);

float Ultrasonic_MeasureDistance(uint8_t sensor_index);
void Ultrasonic_Trigger(uint8_t sensor_index);

void Bluetooth_SendString(char *str);

void EXTI0_IRQHandler(void);
void EXTI1_IRQHandler(void);
void USART1_IRQHandler(void);

void delay(int);

//============================ 함수 구현부 ============================


void RCC_Configure(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
    
    // ADC1 (APB2), TIM1 (APB2)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

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
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; // RX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // USART2 TX: PD5 (AF_PP), USART2 RX: PD6 (IN_FLOATING)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

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

// 현재 분주와 period를 그냥 막 정한 상태임
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

void USART1_Init(void)
{
    USART_InitTypeDef USART1_InitStructure;

    USART_Cmd(USART1, ENABLE);

    USART1_InitStructure.USART_BaudRate = 9600;
    USART1_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART1_InitStructure.USART_StopBits = USART_StopBits_1;
    USART1_InitStructure.USART_Parity = USART_Parity_No;
    USART1_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART1_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &USART1_InitStructure);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

void USART2_Init(void)
{
    USART_InitTypeDef USART2_InitStructure;

    USART_Cmd(USART2, ENABLE);

    USART2_InitStructure.USART_BaudRate = 9600;
    USART2_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART2_InitStructure.USART_StopBits = USART_StopBits_1;
    USART2_InitStructure.USART_Parity = USART_Parity_No;
    USART2_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART2_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART2, &USART2_InitStructure);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
}

void ADC_Configure(void) {
    ADC_InitTypeDef ADC_InitStructure;

    // ADC1 초기화 설정
    ADC_InitStructure.ADC_Mode               = ADC_Mode_Independent;  
    ADC_InitStructure.ADC_ScanConvMode       = ENABLE;        // 스캔 모드
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;        // 연속 변환
    ADC_InitStructure.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign          = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel       = 2;             // 변환할 채널 수: 2
    ADC_Init(ADC1, &ADC_InitStructure);

    // 채널 순서 설정
    // (Rank=1)에 PA0 -> ADC_Channel_0
    // (Rank=2)에 PA1 -> ADC_Channel_1
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_28Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_28Cycles5);

    // **EOC(End of Conversion) 인터럽트 활성화**
    ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);

    // ADC 활성화 및 캘리브레이션
    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));

    // ADC 변환 시작 (연속 모드이므로 자동으로 순차 변환)
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void NVIC_Configure(void) {
    NVIC_InitTypeDef NVIC_InitStructure;
	
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

    // USART1 IRQ
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // USART2 IRQ
    NVIC_EnableIRQ(USART2_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00; 
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // ADC1_2 IRQ
    NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
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
        if (count == 2) {
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
    float distance = 0.0f;
    return distance;
}

void Bluetooth_SendString(char *str) {
    while (*str) {
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        USART_SendData(USART1, *str++);
    }
}

// 이런 식으로 3,4,7,8,9,10,11에 대한 EXTI 핸들러도 동일한 패턴으로 구현
// 실제로는 각 센서 echo 핀에 맞는 EXTI_LineX를 사용해야 함
// USART1 IRQ (PC와 연결)
void USART1_IRQHandler() {
    uint16_t word;
    if(USART_GetITStatus(USART1,USART_IT_RXNE)!=RESET){
        word = USART_ReceiveData(USART1);
        // USART1에서 받은 데이터를 USART2로 에코
        USART_SendData(USART2, word);       
        USART_ClearITPendingBit(USART1,USART_IT_RXNE);
    }
}
// USART2 IRQ (블루투스 모듈 연결)
void USART2_IRQHandler() {
    uint16_t word;
    if(USART_GetITStatus(USART2,USART_IT_RXNE)!=RESET){
        word = USART_ReceiveData(USART2);
        
        // 명령 버퍼에 저장
        if (word == '\n' || word == '\r') {
            // 명령 종료
            bluetooth_rx_buffer[bluetooth_rx_index] = '\0';
            bluetooth_rx_index = 0;
            bluetooth_command_received = 1;
        } else {
            if (bluetooth_rx_index < CMD_BUFFER_SIZE - 1) {
                bluetooth_rx_buffer[bluetooth_rx_index++] = (char)word;
            }
        }
        
        // 받은 데이터를 USART1로 에코 (디버깅용)
        USART_SendData(USART1, word);

        USART_ClearITPendingBit(USART2,USART_IT_RXNE);
    }
}
// ADC 변환 완료 ISR
void ADC1_2_IRQHandler(void) {
    if (ADC_GetITStatus(ADC1, ADC_IT_EOC) != RESET) {
        static uint16_t adc_values[2];
        
        // 첫 번째 변환 결과 (채널 0) 읽기
        adc_values[0] = ADC_GetConversionValue(ADC1);  
        // 두 번째 변환 결과 (채널 1) 읽기
        adc_values[1] = ADC_GetConversionValue(ADC1);  

        // 예시: 임계값 3000 초과 시 "차량 감지"로 간주
        if (adc_values[0] > 3000) {
            enter_trigger = 1;
        }
        if (adc_values[1] > 3000) {
            out_trigger = 1;
        }

        // 인터럽트 플래그 클리어
        ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
    }
}

void delay(int step){
    for (volatile int i = 0; i < step; i++);
}

//============================ 메인 함수 ============================
int main() {
    float distance;
  
    SystemInit();
    RCC_Configure();
    GPIO_Configure();
    ADC_Configure();
    TIM1_Configure();      // 타이머 초기화 (준식)
    USART1_Init(); // PC
    USART2_Init(); // 블루투스
    NVIC_Configure();
    
    // 초기 LED 상태 (모두 GREEN)
    LED_SetColor(0, LED_COLOR_GREEN);
    LED_SetColor(1, LED_COLOR_GREEN);
    LED_SetColor(2, LED_COLOR_GREEN);

    while(1) {
        // 각 초음파 센서(1~9)로 거리 측정하고 일정 거리 이하면 차 있음(1), 아니면 없음(0)
        // sensor_index: 1,2,3  / 4,5,6 / 7,8,9 => 3x3
        // row = (sensor_index-1)/3, col = (sensor_index-1)%3
        printf("%f\n",distance);
        if (enter_trigger){
            // 문 개방

            //준식
            // 초음파 센서 9개 전부 트리거 발생 시작
            for(int i = 0; i < 3; i++)
            {
              for(int j = 0; j < 1; j++)
              {
                distance = Ultrasonic_MeasureDistance(i*(3) + (j+1));
                if(distance < 10 && car_presence[i][j] == 0)
                {
                    car_presence[i][j] = 1;
                    enter_trigger = 0;
                    printf("in car\n");
                    break;
                }
              }
            }
            
            // 만약 초음파 센서 중 하나에서 차가 감지가 되면 car_presence 값 변경
            
            // 차 유무에 따라 LED 업데이트
            LED_UpdateByCarPresence();
        }
        if (out_trigger){
            // 사람이 출구를 나가는 것이 감지가 되는 경우
            
            // 모터 수직 이동, 방금 들어온 차를 보고 모터 index와 방향을 결정해야함
        }
        // 블루투스 출차 명령
        if (bluetooth_command_received) {
            bluetooth_command_received = 0;
            if (strcmp((char*)bluetooth_rx_buffer, "OUT") == 0) {
                // 일단 테스트를 위해 차량 하강
                Motor_SetSteps(1, 3, -1); // direction = -1 (역방향)

                // 실제 구현에서는 주차 공간을 토대로 판단해서 방향을 결정해야함.
            }
        }
        delay(1000000);
    }

    Motor_SetSteps(1, 3, 1);
}