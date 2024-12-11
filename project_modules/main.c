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
   TX 핀: PA9 (USART1_RX)
   RX 핀: PA10 (USART1_TX)

   (USART2 사용)
   TX 핀: PD5 (USART1_RX)
   RX 핀: PD6 (USART1_TX)
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

//============================ 함수 프로토타입 선언 ============================
void RCC_Configure(void);
void GPIO_Configure(void);
void ADC_Configure(void);
void USART_Configure(void);
void NVIC_Configure(void);
void EXTI_Configure(void);

void set_led_color(uint8_t led_num, uint8_t color);
void update_leds_based_on_car_presence(void);

uint16_t read_adc_value(uint8_t channel);

void step_motor_init(void);
void set_rpm(int motor_index, int rpm, int direction);

float measure_distance(uint8_t sensor_index);
void trigger_ultrasonic(uint8_t sensor_index);

void Bluetooth_SendString(char *str);

void EXTI0_IRQHandler(void);
void USART1_IRQHandler(void);

//============================ 함수 구현부 ============================

void RCC_Configure(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    // 타이머 등 필요시 추가
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

    // USART1 TX: PA9 (AF_PP), USART1 RX: PA10 (IN_FLOATING)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
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

    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE; 
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; 
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; 
    ADC_InitStructure.ADC_NbrOfChannel = 1; 
    ADC_Init(ADC1, &ADC_InitStructure);

    // 초기 채널 설정 (필요에 따라 변경)
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_28Cycles5);

    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));

    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void EXTI_Configure(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;

    // 공통 설정
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    // Echo 신호는 Rising 엣지에서 타이머 시작, Falling 엣지에서 타이머 정지를 위해 
    // Rising, Falling 모두 감지할 수 있도록 EXTI_Trigger_Rising_Falling 사용 가능.
    // 필요에 따라 EXTI_Trigger_Rising, EXTI_Trigger_Falling를 선택하세요.
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;

    // Sensor 1: PD3 -> EXTI_Line3
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource3);
    EXTI_InitStructure.EXTI_Line = EXTI_Line3;
    EXTI_Init(&EXTI_InitStructure);

    // Sensor 2: PD4 -> EXTI_Line4
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource4);
    EXTI_InitStructure.EXTI_Line = EXTI_Line4;
    EXTI_Init(&EXTI_InitStructure);

    // Sensor 3: PD7 -> EXTI_Line7
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource7);
    EXTI_InitStructure.EXTI_Line = EXTI_Line7;
    EXTI_Init(&EXTI_InitStructure);

    // Sensor 4: PD8 -> EXTI_Line8
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource8);
    EXTI_InitStructure.EXTI_Line = EXTI_Line8;
    EXTI_Init(&EXTI_InitStructure);

    // Sensor 5: PD9 -> EXTI_Line9
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource9);
    EXTI_InitStructure.EXTI_Line = EXTI_Line9;
    EXTI_Init(&EXTI_InitStructure);

    // Sensor 6: PD10 -> EXTI_Line10
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource10);
    EXTI_InitStructure.EXTI_Line = EXTI_Line10;
    EXTI_Init(&EXTI_InitStructure);

    // Sensor 7: PD11 -> EXTI_Line11
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource11);
    EXTI_InitStructure.EXTI_Line = EXTI_Line11;
    EXTI_Init(&EXTI_InitStructure);

    // Sensor 8: PD12 -> EXTI_Line12
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource12);
    EXTI_InitStructure.EXTI_Line = EXTI_Line12;
    EXTI_Init(&EXTI_InitStructure);

    // Sensor 9: PD13 -> EXTI_Line13
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource13);
    EXTI_InitStructure.EXTI_Line = EXTI_Line13;
    EXTI_Init(&EXTI_InitStructure);

    // Sensor 10: PD14 -> EXTI_Line14
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource14);
    EXTI_InitStructure.EXTI_Line = EXTI_Line14;
    EXTI_Init(&EXTI_InitStructure);

    // Sensor 11: PD15 -> EXTI_Line15
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource15);
    EXTI_InitStructure.EXTI_Line = EXTI_Line15;
    EXTI_Init(&EXTI_InitStructure);
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

    // 필요시 EXTI 인터럽트 추가
}

void set_led_color(uint8_t led_num, uint8_t color) {
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

// 각 열(column)별 상태:
// - 모든 칸이 차 있으면(Red)
// - 모든 칸이 비어있으면(Green)
// - 그 외(Yellow)
void update_leds_based_on_car_presence(void) {
    for (int col = 0; col < 3; col++) {
        int count = 0;
        for (int row = 0; row < 3; row++) {
            if (car_presence[row][col] == 1) count++;
        }
        if (count == 3) {
            set_led_color(col, LED_COLOR_RED); // Red
        } else if (count == 0) {
            set_led_color(col, LED_COLOR_GREEN); // Green
        } else {
            set_led_color(col, LED_COLOR_YELLOW); // Yellow
        }
    }
}

uint16_t read_adc_value(uint8_t channel) {
    ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_28Cycles5);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
    return ADC_GetConversionValue(ADC1);
}

void step_motor_init(void) {
    // 필요시 추가 초기화
}

void set_rpm(int motor_index, int rpm, int direction) {
    uint32_t microseconds_per_minute = 60000000;
    uint32_t total_steps = 4096;  // 1회전당 스텝 수
    uint32_t idle_time = microseconds_per_minute / (total_steps * rpm);

    for (uint32_t step = 0; step < total_steps; step++) {
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

void trigger_ultrasonic(uint8_t sensor_index) {
    GPIO_SetBits(ULTRASONIC_TRIG_PORT, ultrasonic_trig_pins[sensor_index]);
    for(volatile int i=0; i<720; i++); // 약 10us 가정
    GPIO_ResetBits(ULTRASONIC_TRIG_PORT, ultrasonic_trig_pins[sensor_index]);
}

float measure_distance(uint8_t sensor_index) {
    // 초음파 거리 측정 로직 필요
    // 여기서는 틀만 제공
    trigger_ultrasonic(sensor_index);
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
// EXTI 핸들러에서 사용할 헬퍼 함수: sensor_index -> (row,col)
void map_sensor_to_rc(int sensor, int *r, int *c) {
    switch(sensor) {
        case 1: *r=0;*c=0;break;
        case 2: *r=0;*c=1;break;
        case 3: *r=0;*c=2;break;
        case 4: *r=1;*c=0;break;
        case 7: *r=1;*c=1;break;
        case 8: *r=1;*c=2;break;
        case 9: *r=2;*c=0;break;
        case 10:*r=2;*c=1;break;
        case 11:*r=2;*c=2;break;
        default:*r=-1;*c=-1;break;
    }
}

// 초음파 센서 인터럽트 핸들러 예제 (센서 Echo 핀 Rising/Falling 엣지 감지)
// 실제로는 Echo 신호의 Rising 시각 기록, Falling시각 기록 후 거리 계산 필요
// 여기서는 단순히 인터럽트가 발생하면 distance를 측정했다고 가정하고
// 그 결과에 따라 car_presence 설정 예제 코드만 제시
void EXTI1_IRQHandler(void) { // 예: 센서1 Echo 핸들러
    if (EXTI_GetITStatus(EXTI_Line1) != RESET) {
        int r,c;
        float dist = measure_distance(1);
        map_sensor_to_rc(1,&r,&c);
        if (r>=0 && c>=0) {
            if (dist < 20.0f && dist > 0.0f) car_presence[r][c] = 1;
            else car_presence[r][c] = 0;
        }
        EXTI_ClearITPendingBit(EXTI_Line1);
    }
}

// 이거 EXTI 
void EXTI4_IRQHandler(void) {
    if (EXTI_GetITStatus(EXTI_Line4) != RESET) {
        float dist = measure_distance(4);
        int r, c;
        map_sensor_to_rc(7, &r, &c);
        if (r >= 0 && c >= 0) {
            if (dist < DISTANCE_THRESHOLD && dist > 0.0f) car_presence[r][c] = 1;
            else car_presence[r][c] = 0;
        }
        EXTI_ClearITPendingBit(EXTI_Line4);
    }
}

void EXTI7_IRQHandler(void) {
    if (EXTI_GetITStatus(EXTI_Line7) != RESET) {
        float dist = measure_distance(7);
        int r, c;
        map_sensor_to_rc(7, &r, &c);
        if (r >= 0 && c >= 0) {
            if (dist < DISTANCE_THRESHOLD && dist > 0.0f) car_presence[r][c] = 1;
            else car_presence[r][c] = 0;
        }
        EXTI_ClearITPendingBit(EXTI_Line7);
    }
}

void EXTI8_IRQHandler(void) {
    if (EXTI_GetITStatus(EXTI_Line8) != RESET) {
        float dist = measure_distance(8);
        int r, c;
        map_sensor_to_rc(8, &r, &c);
        if (r >= 0 && c >= 0) {
            if (dist < DISTANCE_THRESHOLD && dist > 0.0f) car_presence[r][c] = 1;
            else car_presence[r][c] = 0;
        }
        EXTI_ClearITPendingBit(EXTI_Line8);
    }
}

void EXTI9_IRQHandler(void) {
    if (EXTI_GetITStatus(EXTI_Line9) != RESET) {
        float dist = measure_distance(9);
        int r, c;
        map_sensor_to_rc(9, &r, &c);
        if (r >= 0 && c >= 0) {
            if (dist < DISTANCE_THRESHOLD && dist > 0.0f) car_presence[r][c] = 1;
            else car_presence[r][c] = 0;
        }
        EXTI_ClearITPendingBit(EXTI_Line9);
    }
}

void EXTI10_IRQHandler(void) {
    if (EXTI_GetITStatus(EXTI_Line10) != RESET) {
        float dist = measure_distance(10);
        int r, c;
        map_sensor_to_rc(10, &r, &c);
        if (r >= 0 && c >= 0) {
            if (dist < DISTANCE_THRESHOLD && dist > 0.0f) car_presence[r][c] = 1;
            else car_presence[r][c] = 0;
        }
        EXTI_ClearITPendingBit(EXTI_Line10);
    }
}

void EXTI11_IRQHandler(void) {
    if (EXTI_GetITStatus(EXTI_Line11) != RESET) {
        float dist = measure_distance(11);
        int r, c;
        map_sensor_to_rc(11, &r, &c);
        if (r >= 0 && c >= 0) {
            if (dist < DISTANCE_THRESHOLD && dist > 0.0f) car_presence[r][c] = 1;
            else car_presence[r][c] = 0;
        }
        EXTI_ClearITPendingBit(EXTI_Line11);
    }
}

// ---------------------- revise ---------------------------
// 압력센서로 인터럽트 트리거를 발생시켜야 함.
void ADC_Configure(void) {
    ADC_InitTypeDef ADC_InitStructure;

    // ADC1 쿨럭 활성화
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    // ADC1 초기화 설정
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;  // 독립모드
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;       // 단일 채널 모드
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;  // 연속 변환 모드
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // 외부 트리거 사용 안함
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; // 데이터 정렬 : 오른쪽
    ADC_InitStructure.ADC_NbrOfChannel = 1; // 변환 채널 수 : 1
    ADC_Init(ADC1, &ADC_InitStructure);

    // PA0 핀을 ADC 채널 0으로 설정
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_28Cycles5);

    // Analog Watchdog 설정
    ADC_AnalogWatchdogThresholdsConfig(ADC1, 3000, 1000); // 상한값 : 3000, 하한값 : 1000
    ADC_AnalogWatchdogSingleChannelConfig(ADC1, ADC_Channel_0); // 채널 0에 대해 Analog Watchdog 설정
    ADC_AnalogWatchdogCmd(ADC1, ADC_AnalogWatchdog_SingleRegEnable); // 단일 채널에 대해 AWD 활성화

    // ADC 인터럽트 활성화 (Analog Watchdog 인터럽트)
    ADC_ITConfig(ADC1, ADC_IT_AWD, ENABLE);

    // ADC 활성화 및 캘리브레이션
    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1));

    // ADC 변환 시작
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void NVIC_Configure(void) {
    NVIC_InitTypeDef NVIC_InitStructure;

    // ADC1 인터럽트 활성화    
    NVIC_InitStructure.NVIC_IRQChannel = ADC1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void ADC1_IRQHandler(void) {
    if (ADC_GetITStatus(ADC1, ADC_IT_AWD) != RESET) {
        // 압력 센서 값이 임계값을 벗어난 경우 초음파 센서 트리거
        trigger_ultrasonic(0);  // ex) 초음파 센서 0 번 트리거

        // 인터럽트 플래그 클리어
        ADC_ClearITPendingBit(ADC1, ADC_IT_AWD);
    }
}
// --------------------- end of revise -------------------------------


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

void update_leds_based_on_car_presence(void) {
    for (int col = 0; col < 3; col++) {
        int count = 0;
        for (int row = 0; row < 3; row++) {
            if (car_presence[row][col] == 1) count++;
        }
        if (count == 2) {
            set_led_color(col, LED_COLOR_RED); // Red
        } else if (count == 0) {
            set_led_color(col, LED_COLOR_GREEN); // Green
        } else {
            set_led_color(col, LED_COLOR_YELLOW); // Yellow
        }
    }
}

void delay(int step){
    for (volatile int i = 0; i < step; i++);
}

//============================ 메인 함수 ============================
int main(void) {
    SystemInit();
    RCC_Configure();
    GPIO_Configure();
    ADC_Configure();
    USART_Configure();
    USART1_Init(); // PC
    USART2_Init(); // 블루투스
    NVIC_Configure();
    step_motor_init();

    // 초기 LED 상태 (모두 Red)
    set_led_color(0, 2);
    set_led_color(1, 2);
    set_led_color(2, 2);

    while(1) {
        // 각 초음파 센서(1~9)로 거리 측정하고 일정 거리 이하면 차 있음(1), 아니면 없음(0)
        // sensor_index: 1,2,3  / 4,5,6 / 7,8,9 => 3x3
        // row = (sensor_index-1)/3, col = (sensor_index-1)%3
        for (int s = 1; s <= 9; s++) {
            float dist = measure_distance(s);
            // 예: dist < 20cm 이면 차 있다고 가정
            if (dist > 0 && dist < 20.0f) {
                car_presence[(s-1)/3][(s-1)%3] = 1;
            } else {
                car_presence[(s-1)/3][(s-1)%3] = 0;
            }
        }

        // 차 유무에 따라 LED 업데이트
        update_leds_based_on_car_presence();

        // 입구 초음파(0)로 차량 감지
        float dist_entrance = measure_distance(0);
        if (dist_entrance < 10.0f && dist_entrance > 0.0f) {
            // 차량 입구 도착 감지 -> 문 개방 (모터0 예)
            set_rpm(0, 10);
        }

        // 압력센서 확인 (0~3)
        uint16_t adc_val0 = read_adc_value(ADC_Channel_0); // 압력센서0
        if (adc_val0 > 500) {
            // 수직 이동 모터 동작 (예: 모터1)
            set_rpm(1, 13); 
        }

        // 블루투스 출차 명령
        if (bluetooth_command_received) {
            bluetooth_command_received = 0;
            if (strcmp((char*)bluetooth_rx_buffer, "OUT") == 0) {
                // 차량 하강 (예: 모터1 반대방향 구현 필요)
                set_rpm(1, 13, -1); // direction = -1 (역방향)
            }
        }
        delay(1000000);
    }
}