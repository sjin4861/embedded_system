#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_adc.h"

/* Function Prototypes */
void RCC_Configure(void);
void GPIO_Configure(void);
void ADC_Configure(void);
void USART1_Init(void);

void RCC_Configure(void)
{
    /* GPIOA (ADC, USART1) 클럭 활성화 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    /* USART1 클럭 활성화 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    /* ADC1 클럭 활성화 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
}

void GPIO_Configure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* ADC 핀 설정 (PA0) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; // 아날로그 입력 모드
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* USART1 TX (PA9) 핀 설정 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // Alternate Function Push-Pull
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* USART1 RX (PA10) 핀 설정 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; // 입력 핀
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void ADC_Configure(void)
{
    ADC_InitTypeDef ADC_InitStructure;

    /* ADC 설정 */
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE; // 단일 채널 변환
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; // 지속적인 변환
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; // 오른쪽 정렬
    ADC_InitStructure.ADC_NbrOfChannel = 1; // 채널 수: 1
    ADC_Init(ADC1, &ADC_InitStructure);

    /* ADC1 채널 설정 (Channel 0 = PA0) */
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_28Cycles5);

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

void USART1_Init(void)
{
    USART_InitTypeDef USART_InitStructure;

    /* USART1 설정 */
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &USART_InitStructure);

    /* USART1 활성화 */
    USART_Cmd(USART1, ENABLE);
}

void USART1_SendData(uint16_t data)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); // 전송 가능 대기
    USART_SendData(USART1, data); // 데이터 전송
}

int main(void)
{
    uint16_t adc_value;
    char buffer[20];

    /* 시스템 초기화 */
    SystemInit();

    /* RCC, GPIO, ADC 및 USART 설정 */
    RCC_Configure();
    GPIO_Configure();
    ADC_Configure();
    USART1_Init();

    while (1)
    {
        /* ADC 값 읽기 */
        adc_value = ADC_GetConversionValue(ADC1);

        /* 문자열로 변환하여 USART 전송 */
        sprintf(buffer, "ADC: %d\r\n", adc_value);
        for (int i = 0; buffer[i] != '\0'; i++)
        {
            USART1_SendData(buffer[i]);
        }

        /* 간단한 딜레이 */
        for (volatile int i = 0; i < 100000; i++);
    }
}
