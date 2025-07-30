#include "uart_user.h"

#include <stdio.h>

#include "command.h"
#include "laser_shot_common.h"
#include "task_scheduler.h"
#include "usart.h"
#include "user_init.h"

uint8_t g_uart_command_buffer[UART_USER_BUFFER_SIZE];  // UART command buffer

// 中值滤波相关变量

// 中值滤波相关变量
#define FILTER_BUFFER_SIZE 5                           // 滤波缓冲区大小
static PixelPoint_t point_buffer[FILTER_BUFFER_SIZE];  // 坐标缓冲区
static uint8_t buffer_index = 0;                       // 当前缓冲区索引
static uint8_t buffer_filled = 0;                      // 缓冲区是否已填满


/**
 * @brief 获取中值（冒泡排序）
 *
 * @param values 待排序的数组
 * @param size 数组大小
 * @return uint16_t 中值
 */
static uint16_t GetMedian(uint16_t values[], uint8_t size)
{
    uint16_t temp;
    uint8_t i, j;

    // 冒泡排序
    for (i = 0; i < size - 1; i++) {
        for (j = 0; j < size - i - 1; j++) {
            if (values[j] > values[j + 1]) {
                temp = values[j];
                values[j] = values[j + 1];
                values[j + 1] = temp;
            }
        }
    }

    // 返回中值
    return values[size / 2];
}

/**
 * @brief 对中心点坐标进行中值滤波
 *
 * @param point 待处理的坐标点
 * @return PixelPoint_t 处理后的坐标点
 */
static PixelPoint_t FilterCenterPoint(PixelPoint_t point)
{
    // 将新值加入缓冲区
    point_buffer[buffer_index] = point;
    buffer_index = (buffer_index + 1) % FILTER_BUFFER_SIZE;

    // 如果缓冲区未填满，直接返回原值
    if (!buffer_filled && buffer_index != 0) {
        return point;
    }

    // 标记缓冲区已填满
    buffer_filled = 1;

    // 提取X坐标和Y坐标进行独立处理
    uint16_t x_values[FILTER_BUFFER_SIZE];
    uint16_t y_values[FILTER_BUFFER_SIZE];

    for (uint8_t i = 0; i < FILTER_BUFFER_SIZE; i++) {
        x_values[i] = point_buffer[i].x;
        y_values[i] = point_buffer[i].y;
    }

    // 计算中值并返回
    PixelPoint_t filtered_point;
    filtered_point.x = GetMedian(x_values, FILTER_BUFFER_SIZE);
    filtered_point.y = GetMedian(y_values, FILTER_BUFFER_SIZE);

    return filtered_point;
}

/**
 * @brief 处理 UART 接收到的数据，该函数应在任务调度器中调用
 *
 */
void Uart_DataProcess(void)
{
    uint8_t command_length = Command_GetCommand(g_uart_command_buffer);

    // 收到正确格式数据包时的解析
    if (command_length) {
        // 获取原始坐标值
        PixelPoint_t raw_point;
        raw_point.x = (g_uart_command_buffer[2] << 8) | g_uart_command_buffer[3];
        raw_point.y = (g_uart_command_buffer[4] << 8) | g_uart_command_buffer[5];

        // 应用中值滤波
        PixelPoint_t filtered_point = FilterCenterPoint(raw_point);

        // 更新全局坐标变量
        g_curr_center_point = filtered_point;
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
