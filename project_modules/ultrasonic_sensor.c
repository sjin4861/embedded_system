#include "stm32f10x.h"
#include "core_cm3.h"
#include "misc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_adc.h"

// 핀 정의
#define TRIG_PIN GPIO_Pin_8
#define ECHO_PIN GPIO_Pin_7
#define TRIG_PORT GPIOE
#define ECHO_PORT GPIOE

// 딜레이 함수
void delay_us(uint32_t us) {
    uint32_t count = us * 72; // 72MHz 기준
    while (count--) {
        __asm("nop");
    }
}

// RCC 초기화
void RCC_Configure(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE); // GPIOE 클럭 활성화
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);  // TIM2 클럭 활성화
}

// GPIO 초기화
void GPIO_Configure(void) {
    GPIO_InitTypeDef GPIO_InitStruct;

    // Trig 핀 (출력)
    GPIO_InitStruct.GPIO_Pin = TRIG_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP; // 푸시풀 출력
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(TRIG_PORT, &GPIO_InitStruct);

    // Echo 핀 (입력)
    GPIO_InitStruct.GPIO_Pin = ECHO_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING; // 플로팅 입력
    GPIO_Init(ECHO_PORT, &GPIO_InitStruct);
}

// 타이머 초기화
void TIM2_Configure(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1; // 1MHz (1μs 단위)
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF; // 최대 카운터 값
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    TIM_Cmd(TIM1, ENABLE); // 타이머 활성화
}

// Trig 신호 송출
void Trig(void) {
    GPIO_SetBits(TRIG_PORT, TRIG_PIN); // Trig 핀 HIGH
    delay_us(10);                     // 10μs 대기
    GPIO_ResetBits(TRIG_PORT, TRIG_PIN); // Trig 핀 LOW
}

// 거리 측정 함수
float Measure_Distance(void) {
    uint16_t start_time = 0, stop_time = 0, echo_time = 0;
    float distance = 0.0;

    // Trig 신호 송출
    Trig();
    
    // Echo 핀 HIGH 대기
    while (GPIO_ReadInputDataBit(ECHO_PORT, ECHO_PIN) == 0);

    // Echo 핀 HIGH 시작 시간 기록
    start_time = TIM_GetCounter(TIM1);

    // Echo 핀이 LOW로 전환될 때까지 대기
    while (GPIO_ReadInputDataBit(ECHO_PORT, ECHO_PIN) == 1);

    // Echo 핀 HIGH 종료 시간 기록
    stop_time = TIM_GetCounter(TIM1);
    printf("%u", stop_time);
    // Echo 핀의 HIGH 지속 시간 계산
    if (stop_time >= start_time) {
        echo_time = stop_time - start_time;
    } else {
        echo_time = (0xFFFF - start_time) + stop_time; // 타이머 오버플로우 처리
    }

    // 거리 계산 (단위: cm)
    distance = (float)(echo_time * 0.0343) / 2.0;

    return distance;
}

// UART 초기화 (디버깅용)
void USART_Configure(void) {
    USART_InitTypeDef USART_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; // TX 핀
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; // RX 핀
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);
}

// 메인 함수
int main(void) {
    RCC_Configure();       // RCC 초기화
    GPIO_Configure();      // GPIO 초기화
    TIM2_Configure();      // 타이머 초기화
    USART_Configure();     // UART 초기화

    float distance;

    while (1) {
        distance = Measure_Distance(); // 거리 측정
        printf("Distance: %.2f cm\n", distance); // 거리 출력
        for (volatile int i = 0; i < 1000000; i++); // 지연
    }
} 
