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
   1. 초음파 센서 11개 (0:입구, 1~9: 3x3 주차공간, 10:출구)
   2. 압력 센서 4개
   3. 신호등 LED 3개 (각각 Green/Yellow/Red)
   4. 스텝모터 4개
   5. 블루투스 모듈 1개

   핀 매핑:
   ----------------------------------------------------------------
   초음파 센서 핀 매핑
   입구 : 0
     |  1   2   3 |
     |  4   5   6 |
     |  7   8   9 |
   출구 : 10

   초음파센서 Trig 핀: PC0 ~ PC10
   초음파센서 Echo 핀: PD0 ~ PD10
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
   압력센서 2 : PA2 (ADC_Channel_2)
   압력센서 3 : PA3 (ADC_Channel_3)

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

// 초음파 센서 수
#define ULTRASONIC_COUNT 11
#define THRESHOLD_1 1000
#define THRESHOLD_2 1000

// 블루투스 수신 버퍼
volatile char bluetooth_rx_buffer[100];
volatile uint8_t bluetooth_rx_index = 0;
volatile uint8_t bluetooth_command_received = 0;

// 주차공간 차 유무 저장 (3x3)
// 1: 차 있음, 0: 차 없음
uint8_t car_presence[3][3] = {0};

// 센서값 배열(예: 필요시 사용)
uint16_t sensor_values[6];

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
    GPIO_Pin_0, GPIO_Pin_1, GPIO_Pin_2, GPIO_Pin_3, GPIO_Pin_4,
    GPIO_Pin_5, GPIO_Pin_6, GPIO_Pin_7, GPIO_Pin_8, GPIO_Pin_9, GPIO_Pin_10
};

uint16_t ultrasonic_echo_pins[ULTRASONIC_COUNT] = {
    GPIO_Pin_0, GPIO_Pin_1, GPIO_Pin_2, GPIO_Pin_3, GPIO_Pin_4,
    GPIO_Pin_5, GPIO_Pin_6, GPIO_Pin_7, GPIO_Pin_8, GPIO_Pin_9, GPIO_Pin_10
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

void set_led_color(uint8_t led_num, uint8_t color);
void update_leds_based_on_car_presence(void);

uint16_t read_adc_value(uint8_t channel);

void step_motor_init(void);
void set_rpm(int motor_index, int rpm);

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

    // ADC용 압력센서 핀 설정 (PA0~PA3)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
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

    // 스텝모터 핀 (PE0~PE15) 모두 출력
    GPIO_InitStructure.GPIO_Pin = 0xFFFF; // PE0~PE15
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    // 초음파 센서 Trig (PC0~PC10 : 출력)
    GPIO_InitStructure.GPIO_Pin = 0;
    for (int i = 0; i < ULTRASONIC_COUNT; i++) {
        GPIO_InitStructure.GPIO_Pin |= ultrasonic_trig_pins[i];
    }
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(ULTRASONIC_TRIG_PORT, &GPIO_InitStructure);

    // 초음파 센서 Echo (PD0~PD10 : 입력)
    GPIO_InitStructure.GPIO_Pin = 0;
    for (int i = 0; i < ULTRASONIC_COUNT; i++) {
        GPIO_InitStructure.GPIO_Pin |= ultrasonic_echo_pins[i];
    }
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(ULTRASONIC_ECHO_PORT, &GPIO_InitStructure);
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

void USART_Configure(void) {
    USART_InitTypeDef USART_InitStructure;

    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No; 
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &USART_InitStructure);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART1, ENABLE);
}

void NVIC_Configure(void) {
    NVIC_InitTypeDef NVIC_InitStructure;

    // USART1 인터럽트
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
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

// 주차공간 정보(car_presence)에 따라 LED 상태 업데이트
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
            set_led_color(col, 2); // Red
        } else if (count == 0) {
            set_led_color(col, 0); // Green
        } else {
            set_led_color(col, 1); // Yellow
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

void set_rpm(int motor_index, int rpm) {
    uint32_t microseconds_per_minute = 60000000;
    uint32_t total_steps = 4096; 
    uint32_t idle_time = microseconds_per_minute / (total_steps * rpm);

    for (uint32_t step = 0; step < total_steps; step++) {
        for (int i = 0; i < 4; i++) {
            if (step_sequence[step % 8][i])
                GPIOE->BSRR = motor_pins[motor_index][i];
            else
                GPIOE->BRR = motor_pins[motor_index][i];
        }

        // 간단한 딜레이 (정확한 us 딜레이는 타이머 기반 구현 필요)
        for(volatile uint32_t i = 0; i < idle_time; i++);
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

void EXTI0_IRQHandler(void) {
    if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}

void USART1_IRQHandler(void) {
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        char c = USART_ReceiveData(USART1);
        if(c == '\n' || c == '\r') {
            bluetooth_rx_buffer[bluetooth_rx_index] = '\0';
            bluetooth_command_received = 1;
            bluetooth_rx_index = 0;
        } else {
            bluetooth_rx_buffer[bluetooth_rx_index++] = c;
        }
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
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
                set_rpm(1, 13);
            }
        }
        delay(1000000);
    }
}