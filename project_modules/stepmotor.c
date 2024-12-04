#include "stm32f10x.h"

// 스텝 모터 핀 정의
#define IN1_PIN    GPIO_Pin_0  // GPIOA Pin 0
#define IN2_PIN    GPIO_Pin_1  // GPIOA Pin 1
#define IN3_PIN    GPIO_Pin_2  // GPIOA Pin 2
#define IN4_PIN    GPIO_Pin_3  // GPIOA Pin 3

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
    for (uint32_t step = 0; step < total_steps; step++) {
        // 단계에 맞게 핀 설정
        GPIO_WriteBit(GPIOA, IN1_PIN, step_sequence[step % 8][0]);
        GPIO_WriteBit(GPIOA, IN2_PIN, step_sequence[step % 8][1]);
        GPIO_WriteBit(GPIOA, IN3_PIN, step_sequence[step % 8][2]);
        GPIO_WriteBit(GPIOA, IN4_PIN, step_sequence[step % 8][3]);
        
        // 대기
        for (volatile uint32_t i = 0; i < idle_time; i++);  // 대기 루프
    }
}

void GPIO_Configuration(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

    // GPIOA 클럭 활성화
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    // IN1~IN4 핀을 출력으로 설정
    GPIO_InitStructure.GPIO_Pin = IN1_PIN | IN2_PIN | IN3_PIN | IN4_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

int main(void) {
    // GPIO 설정
    GPIO_Configuration();

    // RPM 설정 (예: 13 RPM)
    set_rpm(13);

    // 메인 루프
    while (1) {
        // 추가 기능 구현 가능
    }
}
