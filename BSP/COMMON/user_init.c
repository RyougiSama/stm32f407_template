#include "user_init.h"
#include "usart.h"
#include "uart_user.h"
#include "app_tasks.h"
#include "oled_hardware_spi.h"
#include "servo_user.h"
/**
 * @brief 用户自定义初始化函数
 * 
 */
void User_Init(void)
{
    // Uart 空闲中断接收使能并关闭 DMA 过半中断
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, g_uart_command_buffer, UART_USER_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    // 初始化应用任务
    AppTasks_Init();
    // 初始化OLED显示
    OLED_Init();
    // 初始化舵机
    Servo_Init();
    Servo_SetPulseWidth_X(SERVO_HORIZONTAL_CHX_DUTY);
    Servo_SetPulseWidth_Y(SERVO_HORIZONTAL_CHY_DUTY);
}
