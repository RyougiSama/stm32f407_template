#include "uart_user.h"

#include <stdio.h>

#include "command.h"
#include "laser_shot_common.h"
#include "usart.h"
#include "user_init.h"

uint8_t g_uart_command_buffer[UART_USER_BUFFER_SIZE];  // UART command buffer

/**
 * @brief 处理 UART 接收到的数据，该函数应在任务调度器中调用
 *
 */
void Uart_DataProcess(void)
{
    uint8_t command_length = Command_GetCommand(g_uart_command_buffer);
    // 收到正确格式数据包时的解析
    if (command_length) {
        g_curr_center_point.x = (g_uart_command_buffer[2] << 8) | g_uart_command_buffer[3];
        g_curr_center_point.y = (g_uart_command_buffer[4] << 8) | g_uart_command_buffer[5];
    }
}

// 中断空闲接收回调函数
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    // Check if the UART instance is USART2
    if (huart->Instance == USART2) {
        Command_Write(g_uart_command_buffer, Size);
        // Re-enable the reception event
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart_command_buffer, UART_USER_BUFFER_SIZE);
        __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
    }
}

/**
 * @brief 重定向c库函数printf到DEBUG_USARTx
 *
 * @param ch
 * @param f
 * @return int
 */
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xffff);
    return ch;
}

/**
 * @brief: 重定向c库函数getchar,scanf到DEBUG_USARTx
 * @param f
 * @return int
 *
 */
int fgetc(FILE *f)
{
    uint8_t ch = 0;
    HAL_UART_Receive(&huart2, &ch, 1, 0xffff);
    return ch;
}