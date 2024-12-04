#include "stm32f10x_hal.h"

/* 핀 설정 */
#define FSR_PIN ADC_CHANNEL_0 // FSR 센서가 연결된 ADC 채널 (PA0)
#define LED_PIN GPIO_PIN_1    // LED가 연결된 GPIO 핀 (PA1)
#define LED_PORT GPIOA        // LED가 연결된 GPIO 포트

/* 전역 변수 */
ADC_HandleTypeDef hadc1;

/* 함수 선언 */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);

int main(void) {
    /* HAL 및 시스템 초기화 */
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_ADC1_Init();

    uint32_t adc_value = 0;

    while (1) {
        /* 1. ADC 값 읽기 */
        HAL_ADC_Start(&hadc1);
        if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {
            adc_value = HAL_ADC_GetValue(&hadc1); // 12비트 ADC 값 (0~4095)
        }
        HAL_ADC_Stop(&hadc1);

        /* 2. 임계값 확인하여 LED 제어 */
        if (adc_value > 1000) { // 임계값 1000 이상이면 LED 켜기
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET); // LED ON
        } else {
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET); // LED OFF
        }

        HAL_Delay(100); // 100ms 딜레이
    }
}

/* GPIO 초기화 */
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIOA 클럭 활성화 */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* LED 핀(PA1) 출력 설정 */
    GPIO_InitStruct.Pin = LED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // 푸시풀 출력
    GPIO_InitStruct.Pull = GPIO_NOPULL;         // 풀업/풀다운 없음
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // 낮은 속도
    HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);
}

/* ADC1 초기화 */
static void MX_ADC1_Init(void) {
    ADC_ChannelConfTypeDef sConfig = {0};

    /* ADC1 설정 */
    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;        // 단일 채널
    hadc1.Init.ContinuousConvMode = DISABLE;          // 단일 변환 모드
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START; // 소프트웨어 시작
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;       // 오른쪽 정렬
    hadc1.Init.NbrOfConversion = 1;                  // 변환 채널 수: 1
    HAL_ADC_Init(&hadc1);

    /* ADC 채널 설정 */
    sConfig.Channel = FSR_PIN; // PA0
    sConfig.Rank = ADC_REGULAR_RANK_1; // 변환 순서: 1
    sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5; // 샘플링 시간
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

/* 시스템 클럭 설정 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* 외부 고속 클럭(HSE) 설정 */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9; // 클럭 72MHz
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    /* 버스 클럭 설정 */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; // AHB 클럭: 72MHz
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  // APB1 클럭: 36MHz
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  // APB2 클럭: 72MHz
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}
