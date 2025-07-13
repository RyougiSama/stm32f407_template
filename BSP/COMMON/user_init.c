#include "user_init.h"
#include "usart.h"
#include "uart_user.h"

/**
 * @brief 用户自定义初始化函数
 * 
 */
void User_Init(void)
{
    // Uart 空闲中断接收使能并关闭 DMA 过半中断
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, g_uart_command_buffer, UART_USER_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
}
