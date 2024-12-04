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

// RPM 설정
void set_rpm(int rpm) {
    // 1분의 마이크로초
    uint32_t microseconds_per_minute = 60000000;
    
    // 1스텝당 대기 시간 계산
    uint32_t total_steps = 4096;  // 1회전당 스텝 수
    uint32_t idle_time = microseconds_per_minute / (total_steps * rpm);
    
    // 모터 회전
    void StepMotor(uint16_t steps, uint32_t delay) {
    for (uint16_t i = 0; i < steps; i++) {
        for (uint8_t j = 0; j < 8; j++) {
            // 각 IN 핀에 대한 출력 설정
            GPIO_WriteBit(GPIOB, GPIO_Pin_0, step_sequence[j][0] ? Bit_SET : Bit_RESET);
            GPIO_WriteBit(GPIOB, GPIO_Pin_1, step_sequence[j][1] ? Bit_SET : Bit_RESET);
            GPIO_WriteBit(GPIOB, GPIO_Pin_2, step_sequence[j][2] ? Bit_SET : Bit_RESET);
            GPIO_WriteBit(GPIOB, GPIO_Pin_3, step_sequence[j][3] ? Bit_SET : Bit_RESET);
            // 딜레이
            for (volatile uint32_t k = 0; k < delay; k++);
        }
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
