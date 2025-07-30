#include "user_init.h"

#include "app_tasks.h"
#include "oled_hardware_spi.h"
#include "servo_user.h"
#include "tim.h"
#include "uart_user.h"
#include "usart.h"
#include "stdbool.h"
#include "draw_task.h"

/**
 * @brief 用户自定义初始化函数
 *
 */
void User_Init(void)
{
    // Uart 空闲中断接收使能并关闭 DMA 过半中断
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, g_uart_command_buffer, UART_USER_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    // 初始化定时器
    HAL_TIM_Base_Start_IT(&htim6);  // 启动定时器6中断
    // 初始化应用任务
    AppTasks_Init();
    // 初始化OLED显示
    OLED_Init();
    // 初始化舵机
    Servo_Init();
    // 初始化GPIO输出
    HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(OUTPUT_TEST_GPIO_Port, OUTPUT_TEST_Pin, GPIO_PIN_RESET);
    // while (true) {
    //     HAL_GPIO_TogglePin(OUTPUT_TEST_GPIO_Port, OUTPUT_TEST_Pin);
    //     HAL_Delay(500);
    // }
}
