#include "uart_user.h"
#include "command.h"
#include "user_init.h"
#include "usart.h"
#include <stdio.h>

#include "control.h"

uint8_t g_uart_command_buffer[UART_USER_BUFFER_SIZE]; // UART command buffer

// 角点缓存结构
#define CORNER_CACHE_SIZE 5
static LaserPoint_t corner_cache[4][CORNER_CACHE_SIZE]; // 4个角点，每个缓存5次
static uint8_t cache_index = 0; // 当前缓存索引
static uint8_t cache_count = 0; // 已缓存的次数

/**
 * @brief 计算中位数，用于剔除异常值
 * @param values 数值数组
 * @param size 数组大小
 * @return uint8_t 中位数
 */
static uint8_t CalculateMedian(uint8_t values[], uint8_t size)
{
    // 简单冒泡排序
    for (uint8_t i = 0; i < size - 1; i++) {
        for (uint8_t j = 0; j < size - 1 - i; j++) {
            if (values[j] > values[j + 1]) {
                uint8_t temp = values[j];
                values[j] = values[j + 1];
                values[j + 1] = temp;
            }
        }
    }
    // 返回中位数
    return values[size / 2];
}

/**
 * @brief 处理角点缓存并设置最终角点值
 */
static void ProcessCornerCache(void)
{
    uint8_t x_values[CORNER_CACHE_SIZE];
    uint8_t y_values[CORNER_CACHE_SIZE];
    // 处理左上角点
    for (uint8_t i = 0; i < CORNER_CACHE_SIZE; i++) {
        x_values[i] = corner_cache[0][i].x;
        y_values[i] = corner_cache[0][i].y;
    }
    g_lu_corner.x = CalculateMedian(x_values, CORNER_CACHE_SIZE);
    g_lu_corner.y = CalculateMedian(y_values, CORNER_CACHE_SIZE);
    // 处理右上角点
    for (uint8_t i = 0; i < CORNER_CACHE_SIZE; i++) {
        x_values[i] = corner_cache[1][i].x;
        y_values[i] = corner_cache[1][i].y;
    }
    g_ru_corner.x = CalculateMedian(x_values, CORNER_CACHE_SIZE);
    g_ru_corner.y = CalculateMedian(y_values, CORNER_CACHE_SIZE);
    // 处理右下角点
    for (uint8_t i = 0; i < CORNER_CACHE_SIZE; i++) {
        x_values[i] = corner_cache[2][i].x;
        y_values[i] = corner_cache[2][i].y;
    }
    g_rd_corner.x = CalculateMedian(x_values, CORNER_CACHE_SIZE);
    g_rd_corner.y = CalculateMedian(y_values, CORNER_CACHE_SIZE);
    // 处理左下角点
    for (uint8_t i = 0; i < CORNER_CACHE_SIZE; i++) {
        x_values[i] = corner_cache[3][i].x;
        y_values[i] = corner_cache[3][i].y;
    }
    g_ld_corner.x = CalculateMedian(x_values, CORNER_CACHE_SIZE);
    g_ld_corner.y = CalculateMedian(y_values, CORNER_CACHE_SIZE);
    // 设置完成标志并输出确认信息
    g_is_corner_init = 1;
    printf("OK");
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
        // HAL_UART_Transmit_DMA(&huart1, g_uart_command_buffer, command_length);
        g_laser_point.x = g_uart_command_buffer[2];
        g_laser_point.y = g_uart_command_buffer[3];
        // 处理角点信息缓存
        if (!g_is_corner_init) {
            // 缓存角点数据
            corner_cache[0][cache_index].x = g_uart_command_buffer[4];  // 左上角
            corner_cache[0][cache_index].y = g_uart_command_buffer[5];
            corner_cache[1][cache_index].x = g_uart_command_buffer[6];  // 右上角
            corner_cache[1][cache_index].y = g_uart_command_buffer[7];
            corner_cache[2][cache_index].x = g_uart_command_buffer[8];  // 右下角
            corner_cache[2][cache_index].y = g_uart_command_buffer[9];
            corner_cache[3][cache_index].x = g_uart_command_buffer[10]; // 左下角
            corner_cache[3][cache_index].y = g_uart_command_buffer[11];
            cache_index = (cache_index + 1) % CORNER_CACHE_SIZE;
            cache_count++;
            // 当缓存满5次后，处理角点数据
            if (cache_count >= CORNER_CACHE_SIZE) {
                ProcessCornerCache();
                cache_count = 0; // 重置计数，防止重复处理
            }
        }
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

/**
 * @brief 重定向c库函数printf到DEBUG_USARTx
 * 
 * @param ch 
 * @param f 
 * @return int 
 */
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
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
    HAL_UART_Receive(&huart1, &ch, 1, 0xffff);
    return ch;
}