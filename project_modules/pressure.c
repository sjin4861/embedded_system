#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_adc.h"
#include <stdio.h>
#include <string.h>

/* Function Prototypes */
void RCC_Configure(void);
void GPIO_Configure(void);
void ADC_Configure(void);

void RCC_Configure(void)
{
    /* GPIOA (ADC) 클럭 활성화 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    /* ADC1 클럭 활성화 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
}

void GPIO_Configure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* ADC 입력 핀 설정 (PA0, PA1) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; // 아날로그 입력 모드
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void ADC_Configure(void)
{
    ADC_InitTypeDef ADC_InitStructure;

    /* ADC 설정 */
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE; // 다중 채널 변환
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; // 지속적인 변환
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; // 오른쪽 정렬
    ADC_InitStructure.ADC_NbrOfChannel = 2; // 채널 수: 2
    ADC_Init(ADC1, &ADC_InitStructure);

    /* ADC1 채널 설정 (Channel 0 = PA0, Channel 1 = PA1) */
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_28Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_28Cycles5);

    /* ADC 활성화 */
    ADC_Cmd(ADC1, ENABLE);

    /* ADC 보정 */
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1));

    /* ADC 변환 시작 */
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

int main(void)
{
    uint16_t adc_value_pa0, adc_value_pa1;

    /* 시스템 초기화 */
    SystemInit();

    /* RCC, GPIO, ADC 설정 */
    RCC_Configure();
    GPIO_Configure();
    ADC_Configure();

    while (1)
    {
        /* PA0 (입구 압력센서) 값 읽기 */
        adc_value_pa0 = ADC_GetConversionValue(ADC1);

        /* 대기 후 PA1 (출구 압력센서) 값 읽기 */
        while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
        adc_value_pa1 = ADC_GetConversionValue(ADC1);

        /* 입구 센서 동작 여부 확인 */
        if (adc_value_pa0 > 2000) // 임계값 2000 설정 (예시)
        {
            // 입구 센서 트리거 처리
            // 예: 차량이 입구에 들어옴
            printf("PA0: %d\n", adc_value_pa0);
        }

        /* 출구 센서 동작 여부 확인 */
        if (adc_value_pa1 > 2000) // 임계값 2000 설정 (예시)
        {
            // 출구 센서 트리거 처리
            // 예: 차량이 출구로 나감
            printf("PA1: %d\n", adc_value_pa1);
        }

        /* 간단한 딜레이 */
        for (volatile int i = 0; i < 100000; i++);
    }
}
