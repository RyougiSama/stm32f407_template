/**
 ******************************************************************************
 * @file           : app_tasks.c
 * @brief          : Application tasks implementation
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include "app_tasks.h"
#include "uart_user.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>
#include "oled_hardware_spi.h"

/* 任务函数实现 */

/**
 * @brief 舵机任务
 * 
 */
static void Task_Servo(void)
{

}

/**
 * @brief OLED显示任务
 * 
 */
static void Task_OLEDDisplay(void)
{
    /* OLED显示相关代码 */
}

/**
 * @brief UART数据处理任务
 */
static void Task_UartProcess(void)
{
    /* 调用原有的UART数据处理函数 */
    Uart_DataProcess();
}

/**
 * @brief 系统监控任务
 */
static void Task_SystemMonitor(void)
{
    static uint32_t counter = 0;
    counter++;
    /* 每10次执行输出一次系统状态 */
    if (counter % 10 == 0) {
        printf("System Monitor - Tick: %lu, Counter: %lu\r\n", 
               TaskScheduler_GetSystemTick(), counter);
        /* 可以在这里添加系统状态监控代码 */
        /* 例如：检查堆栈使用情况、内存使用情况等 */
    }
}

/**
 * @brief 应用任务初始化
 */
void AppTasks_Init(void)
{
    /* 初始化任务调度器 */
    TaskScheduler_Init();
    /* 添加任务到调度器 */
    /* 参数：任务函数, 执行周期(ms), 优先级, 任务名称 */
    TaskScheduler_AddTask(Task_UartProcess, 10, TASK_PRIORITY_HIGH, "UART_Task");
    TaskScheduler_AddTask(Task_Servo, 20, TASK_PRIORITY_NORMAL, "Servo_Task");
    // TaskScheduler_AddTask(Task_OLEDDisplay, 100, TASK_PRIORITY_NORMAL, "OLED_Task");
    // TaskScheduler_AddTask(Task_SystemMonitor, 1000, TASK_PRIORITY_NORMAL, "Monitor_Task");
    /* 输出任务信息 */
    // printf("Task Scheduler Initialized with %d tasks\r\n", TaskScheduler_GetTaskCount());
}
