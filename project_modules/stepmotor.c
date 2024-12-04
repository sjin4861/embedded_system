#include "stm32f10x.h"

// 스텝 모터 핀 정의
#define IN1_PIN    GPIO_Pin_11  // GPIOD Pin 11
#define IN2_PIN    GPIO_Pin_12  // GPIOD Pin 12
#define IN3_PIN    GPIO_Pin_13  // GPIOD Pin 13
#define IN4_PIN    GPIO_Pin_14  // GPIOD Pin 14

// 스텝 시퀀스 정의
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

// 딜레이 함수
void delay_us(uint32_t us) {
    // 시스템 클럭 주파수에 따라 조정해야 합니다.
    // 예를 들어, 시스템 클럭이 72MHz인 경우:
    uint32_t count = (SystemCoreClock / 1000000) * us / 5;
    for (; count != 0; count--);
}

// RPM 설정
void set_rpm(int rpm) {
    uint32_t microseconds_per_minute = 60000000;
    uint32_t total_steps = 4096;  // 1회전당 스텝 수
    uint32_t idle_time = microseconds_per_minute / (total_steps * rpm);

    for (uint32_t step = 0; step < total_steps; step++) {
        // 단계에 맞게 핀 설정
        if (step_sequence[step % 8][0])
            GPIOD->BSRR = IN1_PIN;
        else
            GPIOD->BRR = IN1_PIN;

        if (step_sequence[step % 8][1])
            GPIOD->BSRR = IN2_PIN;
        else
            GPIOD->BRR = IN2_PIN;

        if (step_sequence[step % 8][2])
            GPIOD->BSRR = IN3_PIN;
        else
            GPIOD->BRR = IN3_PIN;

        if (step_sequence[step % 8][3])
            GPIOD->BSRR = IN4_PIN;
        else
            GPIOD->BRR = IN4_PIN;

        // 딜레이
        delay_us(idle_time);
    }
}

void GPIO_Configuration(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

    // GPIOD 클럭 활성화
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

    // IN1~IN4 핀을 출력으로 설정
    GPIO_InitStructure.GPIO_Pin = IN1_PIN | IN2_PIN | IN3_PIN | IN4_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
}

int main(void) {
    // 시스템 초기화 (필요에 따라 추가)
    SystemInit();

    // GPIO 설정
    GPIO_Configuration();

    // RPM 설정 (예: 13 RPM)
    set_rpm(13);

    // 메인 루프
    while (1) {
        // 추가 기능 구현 가능
        set_rpm(13);
    }
}