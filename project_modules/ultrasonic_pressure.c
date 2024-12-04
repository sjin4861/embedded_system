#include "stm32f10x.h"
#include "core_cm3.h"
#include "misc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_usart.h"
#include <stdio.h>

// 핀 정의
#define TRIG_PIN GPIO_Pin_8   // 초음파 센서 Trig 핀 (GPIOE 8번 핀 사용)
#define ECHO_PIN GPIO_Pin_7   // 초음파 센서 Echo 핀 (GPIOE 7번 핀 사용)
#define TRIG_PORT GPIOE       // Trig 핀이 연결된 포트
#define ECHO_PORT GPIOE       // Echo 핀이 연결된 포트

#define LED_PIN GPIO_Pin_13   // 보드의 내장 LED 핀 (GPIOC 13번 핀 사용)
#define LED_PORT GPIOC        // LED 핀이 연결된 포트

// UART 디버깅용 함수 프로토타입
void USART_Configure(void);
void USART_SendString(char *str);

// 딜레이 함수 (마이크로초 단위)
void delay_us(uint32_t us) {
    uint32_t count = us * 72; // 72MHz 기준으로 1μs당 72 사이클.
    while (count--) {
        __asm("nop"); // 'nop'은 아무 작업도 하지 않고 사이클만 소비.
    }
}

// RCC 초기화
void RCC_Configure(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE); // GPIOE 클럭 활성화
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);  // TIM1 클럭 활성화
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); // GPIOC 클럭 활성화
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);  // ADC1 클럭 활성화
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); // GPIOA 클럭 활성화
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); // USART1 클럭 활성화
}

// GPIO 초기화
void GPIO_Configure(void) {
    GPIO_InitTypeDef GPIO_InitStruct;

    // 초음파 센서 Trig 핀 설정 (출력)
    GPIO_InitStruct.GPIO_Pin = TRIG_PIN;            // GPIOE 8번 핀
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;   // 푸시풀 출력 모드
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;  // 핀 속도 50MHz
    GPIO_Init(TRIG_PORT, &GPIO_InitStruct);

    // 초음파 센서 Echo 핀 설정 (입력)
    GPIO_InitStruct.GPIO_Pin = ECHO_PIN;            // GPIOE 7번 핀
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING; // 플로팅 입력 (풀업 없음)
    GPIO_Init(ECHO_PORT, &GPIO_InitStruct);

    // LED 핀 설정 (출력)
    GPIO_InitStruct.GPIO_Pin = LED_PIN;             // GPIOC 13번 핀
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;   // 푸시풀 출력 모드
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;  // 핀 속도 50MHz
    GPIO_Init(LED_PORT, &GPIO_InitStruct);

    // UART TX (PA9) 핀 설정
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;          // GPIOA 9번 핀
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;    // Alternate Function Push-Pull
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;  // 핀 속도 50MHz
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // UART RX (PA10) 핀 설정
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;         // GPIOA 10번 핀
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING; // 플로팅 입력
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // ADC 핀 설정 (압력 센서 입력 핀: PA0)
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;          // GPIOA 0번 핀
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;      // 아날로그 입력 모드
    GPIO_Init(GPIOA, &GPIO_InitStruct);
}

// 타이머 초기화
void TIM2_Configure(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    // TIM1 설정 (1μs 단위로 동작하도록 설정)
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;       // 1MHz로 작동 (72MHz / 72)
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // 카운터 증가 모드
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;          // 최대 카운터 값 (65535)
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; // 기본 클럭 분주 없음
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    TIM_Cmd(TIM1, ENABLE); // TIM1 활성화
}

// ADC 초기화
void ADC_Configure(void) {
    ADC_InitTypeDef ADC_InitStructure;

    // ADC1 설정
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;         // 독립 모드
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;              // 단일 채널 변환
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;         // 지속적 변환
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // 외부 트리거 없음
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;     // 결과를 오른쪽 정렬
    ADC_InitStructure.ADC_NbrOfChannel = 1;                    // 변환 채널 수: 1
    ADC_Init(ADC1, &ADC_InitStructure);

    // ADC1 채널 설정 (Channel 0 -> PA0)
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_28Cycles5);

    // ADC 활성화 및 보정
    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1);               // 보정 리셋
    while (ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);               // 보정 시작
    while (ADC_GetCalibrationStatus(ADC1));

    ADC_SoftwareStartConvCmd(ADC1, ENABLE);   // 변환 시작
}

// UART 초기화
void USART_Configure(void) {
    USART_InitTypeDef USART_InitStructure;

    USART_InitStructure.USART_BaudRate = 9600;            // 보드레이트 설정
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; // 데이터 길이: 8비트
    USART_InitStructure.USART_StopBits = USART_StopBits_1; // 정지 비트: 1비트
    USART_InitStructure.USART_Parity = USART_Parity_No;   // 패리티 없음
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // 수신 및 송신 활성화
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &USART_InitStructure);

    USART_Cmd(USART1, ENABLE); // UART 활성화
}

// UART로 문자열 전송
void USART_SendString(char *str) {
    while (*str) {
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        USART_SendData(USART1, *str++);
    }
}

// 초음파 Trig 신호 송출
void Trig(void) {
    GPIO_SetBits(TRIG_PORT, TRIG_PIN);  // Trig 핀 HIGH
    delay_us(10);                      // 10μs 유지
    GPIO_ResetBits(TRIG_PORT, TRIG_PIN); // Trig 핀 LOW
}

// 초음파 거리 측정 함수
float Measure_Distance(void) {
    uint16_t start_time = 0, stop_time = 0, echo_time = 0;
    float distance = 0.0;

    Trig(); // Trig 신호 송출

    // Echo 핀 HIGH 상태 대기
    while (GPIO_ReadInputDataBit(ECHO_PORT, ECHO_PIN) == 0);

    // Echo 핀 HIGH 시작 시간 기록
    start_time = TIM_GetCounter(TIM1);

    // Echo 핀이 LOW 상태로 전환될 때까지 대기
    while (GPIO_ReadInputDataBit(ECHO_PORT, ECHO_PIN) == 1);

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

    return distance;
}

// 메인 함수
int main(void) {
    float distance;       // 초음파 거리
    uint16_t adc_value;   // 압력 센서 ADC 값
    char debug_msg[50];   // 디버깅 메시지

    // 시스템 초기화
    SystemInit();

    // 각 하드웨어 모듈 초기화
    RCC_Configure();      // 클럭 설정
    GPIO_Configure();     // GPIO 설정
    TIM2_Configure();     // 타이머 설정
    ADC_Configure();      // ADC 설정
    USART_Configure();    // UART 설정

    while (1) {
        // 초음파 거리 측정
        distance = Measure_Distance();

        // 압력 센서 ADC 값 읽기
        adc_value = ADC_GetConversionValue(ADC1);
;

        // 조건 확인: 거리 < 4cm && 압력 > 500
        if (distance < 4.0 && adc_value > 500) {
        printf("Distance: %.2f cm, ADC: %d\r\n", distance, adc_value);
        printf("LED ON\n");
        } else {
        printf("Distance: %.2f cm, ADC: %d\r\n", distance, adc_value);
        printf("LED OFF\n");
        }

        // 간단한 딜레이
        for (volatile int i = 0; i < 100000; i++);
    }
}