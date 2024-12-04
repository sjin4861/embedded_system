#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

// LED 핀 정의
#define LED_GREEN_PIN GPIO_Pin_0
#define LED_YELLOW_PIN GPIO_Pin_1
#define LED_RED_PIN GPIO_Pin_2
#define LED_PORT GPIOB

// 초음파 센서 임계값
#define THRESHOLD_1 1000
#define THRESHOLD_2 1000

// 초음파 센서 입력 함수 (가상의 함수로, 실제 구현 필요)
uint32_t read_ultrasonic_sensor_1(void);
uint32_t read_ultrasonic_sensor_2(void);

void RCC_Configure(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
}

void GPIO_Configure(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // LED 핀 설정
    GPIO_InitStructure.GPIO_Pin = LED_GREEN_PIN | LED_YELLOW_PIN | LED_RED_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(LED_PORT, &GPIO_InitStructure);
}

void set_led_color(uint8_t color) {
    GPIO_ResetBits(LED_PORT, LED_GREEN_PIN | LED_YELLOW_PIN | LED_RED_PIN);
    switch (color) {
        case 0: // Green
            GPIO_SetBits(LED_PORT, LED_GREEN_PIN);
            break;
        case 1: // Yellow
            GPIO_SetBits(LED_PORT, LED_YELLOW_PIN);
            break;
        case 2: // Red
            GPIO_SetBits(LED_PORT, LED_RED_PIN);
            break;
    }
}

int show_led(sensor1_value, sensor2_value) {
    if (sensor1_value > THRESHOLD_1 && sensor2_value > THRESHOLD_2) {
        set_led_color(2); // Red
    } else if (sensor1_value > THRESHOLD_1 || sensor2_value > THRESHOLD_2) {
        set_led_color(1); // Yellow
    } else {
        set_led_color(0); // Green
    }
}