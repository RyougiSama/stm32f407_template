#include "uart_user.h"
#include "command.h"
#include "user_init.h"
#include "usart.h"

uint8_t g_uart_command_buffer[UART_USER_BUFFER_SIZE]; // UART command buffer

/**
 * @brief 处理 UART 接收到的数据，该函数应在任务调度器中调用
 * 
 */
void Uart_DataProcess(void)
{
    uint8_t command_length = Command_GetCommand(g_uart_command_buffer);
    // 收到正确格式数据包时的解析
    if (command_length) {
        HAL_UART_Transmit_DMA(&huart1, g_uart_command_buffer, command_length);
    }
}

// 中断空闲接收回调函数
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    // Check if the UART instance is USART1
    if (huart->Instance == USART1)  { 
        Command_Write(g_uart_command_buffer, Size);
        // Re-enable the reception event
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart_command_buffer, UART_USER_BUFFER_SIZE);
        __HAL_DMA_ENABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    }
}