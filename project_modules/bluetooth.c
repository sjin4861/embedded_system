#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_exti.h"
#include "misc.h"

void bt_init(void);
void bt_rcc_configure(void);
void bt_gpio_configure(void);
void bt_usart1_configure(void);
void bt_usart2_configure(void);
void bt_nvic_configure(void);

uint16_t bt_get_user_input(void);
void bt_send_to_user(char *message);


void bt_init(void)
{
    bt_rcc_configure();
    bt_gpio_configure();
    bt_usart1_configure();
    bt_usart2_configure();
    bt_nvic_configure();
}

void bt_rcc_configure(void)
{
    /* USART2 TX/RX port clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

    /* USART2 clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    /* Alternate Function IO clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
}

void bt_gpio_configure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // USART1 ¼³Á¤
    // TX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    // RX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; // hw support pull-up reg
    // GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; // it can be used
    // GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // it can be used
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // USART2 ¼³Á¤
    GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);
    // TX (PD5)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    // RX (PD6)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; // hw support pull-up reg
    // GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; // it can be used
    // GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // it can be used
    GPIO_Init(GPIOD, &GPIO_InitStructure);
}

void bt_usart1_configure(void)
{
    USART_InitTypeDef USART1_InitStructure;

    // Enable the USART1 peripheral
    USART_Cmd(USART1, ENABLE);

    // TODO: Initialize the USART using the structure 'USART_InitTypeDef' and the function 'USART_Init'
    USART1_InitStructure.USART_BaudRate = 9600;
    USART1_InitStructure.USART_StopBits = USART_StopBits_1;
    USART1_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART1_InitStructure.USART_Parity = USART_Parity_No;
    USART1_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART1_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART1_InitStructure);

    // TODO: Enable the USART1 RX interrupts using the function 'USART_ITConfig' and the argument value 'Receive Data register not empty interrupt'
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

void bt_usart2_configure(void)
{
    USART_InitTypeDef USART2_InitStructure;

    // Enable the USART2 peripheral
    USART_Cmd(USART2, ENABLE);

    // TODO: Initialize the USART using the structure 'USART_InitTypeDef' and the function 'USART_Init'
    USART2_InitStructure.USART_BaudRate = 9600;
    USART2_InitStructure.USART_StopBits = USART_StopBits_1;
    USART2_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART2_InitStructure.USART_Parity = USART_Parity_No;
    USART2_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART2_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART2_InitStructure);

    // TODO: Enable the USART2 RX interrupts using the function 'USART_ITConfig' and the argument value 'Receive Data register not empty interrupt'
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
}

void bt_nvic_configure(void)
{

    NVIC_InitTypeDef NVIC_InitStructure;

    // TODO: fill the arg you want
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    // USART1
    // 'NVIC_EnableIRQ' is only required for USART setting
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; // TODO
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        // TODO
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // USART2
    // 'NVIC_EnableIRQ' is only required for USART setting
    NVIC_EnableIRQ(USART2_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; // TODO
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;        // TODO
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


void USART1_IRQHandler()
{
    uint16_t word;
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        // the most recent received data by the USART1 peripheral
        word = USART_ReceiveData(USART1);

        // TODO implement ???
        //while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        //while ((USART1->SR & USART_SR_TXE) == 0);
        USART_SendData(USART2, word);

        // clear 'Read data register not empty' flag
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}
uint16_t is_connected = 0;
uint16_t user_input = 0;
void USART2_IRQHandler()
{
    uint16_t word;
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        // the most recent received data by the USART2 peripheral
        word = USART_ReceiveData(USART2);
        
        is_connected = 1;

        // TODO implement
        //while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
        USART_SendData(USART1, word);
        user_input = word;
        /*
        // Validate the received input
        // 'w' - forward | 'a' - left | 's' - backward | 'd' - right | 'x' - stop | 'c' - change mode
        if (word == 119 || word == 97 || word == 115 || word == 100 || word == 120 || word == 99)
        {
            user_input = word; // Valid input
        }
        else
        {
            // Invalid input, send error message
            char error_message[] = "invalid input\n";
            bt_send_to_user(error_message);
        }
*/

        // clear 'Read data register not empty' flag
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}
/*
void USART2_IRQHandler()
{
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        // Store the most recent received data
        char received = (char)USART_ReceiveData(USART2);

        // Validate the received input
        // 'w' - forward | 'a' - left | 's' - backward | 'd' - right | 'x' - stop | 'c' - change mode
        if (received == 'w' || received == 'a' || received == 's' || received == 'd' || received == 'x' || received == 'c')
        {
            user_input = received; // Valid input
        }
        else
        {
            // Invalid input, send error message
            char error_message[] = "invalid input\n";
            bt_send_to_user(error_message);
        }

        // Clear the 'Read data register not empty' flag
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}
*/

uint16_t bt_get_user_input(void)
{       
    uint16_t result = user_input;
    user_input = 122;
    return result;
}

void bt_send_to_user(char *message)
{
    for (int i = 0; message[i] != '\0'; i++)
    {
        while ((USART2->SR & USART_SR_TXE) == 0) // Wait until TX buffer is empty
            ;

        USART_SendData(USART2, message[i]);
    }
    for (int i = 0; message[i] != '\0'; i++)
    {
        while ((USART1->SR & USART_SR_TXE) == 0) // Wait until TX buffer is empty
            ;

        USART_SendData(USART1, message[i]);
    }
}