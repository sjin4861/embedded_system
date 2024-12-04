#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

// LED 핀 정의
#define LED1_GREEN_PIN GPIO_Pin_0
#define LED1_YELLOW_PIN GPIO_Pin_1
#define LED1_RED_PIN GPIO_Pin_2
#define LED1_PORT GPIOB
#define LED2_GREEN_PIN GPIO_Pin_3
#define LED2_YELLOW_PIN GPIO_Pin_4
#define LED2_RED_PIN GPIO_Pin_5
#define LED2_PORT GPIOB
#define LED3_GREEN_PIN GPIO_Pin_6   
#define LED3_YELLOW_PIN GPIO_Pin_7
#define LED3_RED_PIN GPIO_Pin_8
#define LED3_PORT GPIOB
// 초음파 센서 임계값, 추후 수정 예정
#define THRESHOLD_1 1000
#define THRESHOLD_2 1000


void RCC_Configure(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
}

void set_led_color(uint8_t led_num, uint8_t color) {
    switch (led_num) {
        case 1: // LED1
            GPIO_ResetBits(GPIOB, LED1_GREEN_PIN | LED1_YELLOW_PIN | LED1_RED_PIN);
            break;
        case 2: // LED2
            GPIO_ResetBits(GPIOB, LED2_GREEN_PIN | LED2_YELLOW_PIN | LED2_RED_PIN);
            break;
        case 3: // LED3
            GPIO_ResetBits(GPIOB, LED3_GREEN_PIN | LED3_YELLOW_PIN | LED3_RED_PIN);
            break;
    }
    switch (color) {
        case 0: // Green
            GPIO_SetBits(GPIOB, led_num == 1 ? LED1_GREEN_PIN : led_num == 2 ? LED2_GREEN_PIN : LED3_GREEN_PIN);
            break;
        case 1: // Yellow
            GPIO_SetBits(GPIOB, led_num == 1 ? LED1_YELLOW_PIN : led_num == 2 ? LED2_YELLOW_PIN : LED3_YELLOW_PIN);
            break;
        case 2: // Red
            GPIO_SetBits(GPIOB, led_num == 1 ? LED1_RED_PIN : led_num == 2 ? LED2_RED_PIN : LED3_RED_PIN);
            break;
    }
}

// sensor 1~6까지 데이터를 읽고 LED 3개를 제어하는 함수
void show_led(uint16_t sensor_values[6]) {
    // sensor_values의 값을 차근차근 읽고 적절한 LED 색상을 설정
    
    if (sensor_values[0] < THRESHOLD_1 && sensor_values[1] < THRESHOLD_2) {
        set_led_color(1, 2); // Red
    } else if (sensor_values[0] < THRESHOLD_1 || sensor_values[1] < THRESHOLD_2) {
        set_led_color(1, 1); // Yellow
    } else if (sensor_values[0] >= THRESHOLD_1 && sensor_values[1] >= THRESHOLD_2) {
        set_led_color(1, 0); // Green
    }

    if (sensor_values[2] < THRESHOLD_1 && sensor_values[3] < THRESHOLD_2) {
        set_led_color(2, 2); // Red
    } else if (sensor_values[2] < THRESHOLD_1 || sensor_values[3] < THRESHOLD_2) {
        set_led_color(2, 1); // Yellow
    } else if (sensor_values[2] >= THRESHOLD_1 && sensor_values[3] >= THRESHOLD_2) {
        set_led_color(2, 0); // Green
    }

    if (sensor_values[4] < THRESHOLD_1 && sensor_values[5] < THRESHOLD_2) {
        set_led_color(3, 2); // Red
    } else if (sensor_values[4] < THRESHOLD_1 || sensor_values[5] < THRESHOLD_2) {
        set_led_color(3, 1); // Yellow
    } else if (sensor_values[4] >= THRESHOLD_1 && sensor_values[5] >= THRESHOLD_2) {
        set_led_color(3, 0); // Green
    }
}