#include "user_init.h"

#include "Emm_V5.h"
#include "app_tasks.h"
#include "oled_user.h"
#include "stdbool.h"
#include "stdio.h"
#include "tim.h"
#include "uart_user.h"
#include "usart.h"

/**
 * @brief 用户自定义初始化函数
 *
 */
void User_Init(void)
{
    // Uart 空闲中断接收使能并关闭 DMA 过半中断
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, g_uart_command_buffer, UART_USER_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
    // 初始化定时器
    HAL_TIM_Base_Start_IT(&htim6);  // 启动定时器6中断
    // 初始化应用任务
    AppTasks_Init();
    // 初始化OLED显示
    // OLED_Init();
    // OLED_Clear();
    HAL_Delay(500);
    // 初始化GPIO输出
    HAL_GPIO_WritePin(OUTPUT_TEST_GPIO_Port, OUTPUT_TEST_Pin, GPIO_PIN_RESET);
    HAL_Delay(100);
    Emm_V5_En_Control(0, true, false);
    HAL_Delay(200);
    Emm_V5_Origin_Trigger_Return(STEP_MOTOR_Y, 0, false);
    HAL_Delay(500);
}
