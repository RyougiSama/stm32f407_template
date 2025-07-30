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

#include <stdio.h>

#include "gpio.h"
#include "key.h"
#include "oled_user.h"
#include "tim.h"
#include "uart_user.h"
#include "laser_shot_common.h"

/* 任务函数实现 */
#if 0
/**
 * @brief 舵机控制任务
 * 
 */
static void Task_ServoCtrl(void)
{
    // 1. 获取目标角度 target_angle
    // 2. 获取当前角度 current_angle
    // 3. PID 控制计算
    PID_SetTarget(&g_servo_position_pid, target_angle);
    float servo_output = PID_Compute(&g_servo_position_pid, current_angle);
    // 4. 将 PID 输出应用到舵机
    Servo_SetAngle_DirY((int16_t)servo_output);
}
#endif

#if 0
static void Task_KeyProcess(void)
{

}
#endif

#if 1
/**
 * @brief OLED显示任务
 *
 */
static void Task_OLEDDisplay(void)
{
    OLED_Display();
}
#endif

#if 0
/**
 * @brief UART数据处理任务
 */
static void Task_UartProcess(void)
{
    /* 调用原有的UART数据处理函数 */
    Uart_DataProcess();
}
#endif

#if 1
/**
 * @brief 中断执行UART数据处理
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        // 每10ms执行一次
        Uart_DataProcess();
    }
}

/**
 * @brief 云台追踪控制任务
 */
static void Task_TrackControl(void)
{
    // 执行云台追踪控制
    Task_BasicQ2_Excute();
}
#endif

#if 0
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
#endif

/**
 * @brief 应用任务初始化
 */
void AppTasks_Init(void)
{
    /* 初始化任务调度器 */
    TaskScheduler_Init();
    /* 添加任务到调度器 */
    /* 参数：任务函数, 执行周期(ms), 优先级, 任务名称 */
    // TaskScheduler_AddTask(Task_UartProcess, 10, TASK_PRIORITY_HIGH, "UART_Task");
    TaskScheduler_AddTask(Key_Proc, 20, TASK_PRIORITY_NORMAL, "Key_Task");
    // TaskScheduler_AddTask(Task_ServoCtrl, 20, TASK_PRIORITY_NORMAL, "Servo_Task");
    // TaskScheduler_AddTask(Task_OLEDDisplay, 50, TASK_PRIORITY_NORMAL, "OLED_Task");
    TaskScheduler_AddTask(Task_TrackControl, 20, TASK_PRIORITY_HIGH, "Track_Task");
    // TaskScheduler_AddTask(Task_SystemMonitor, 1000, TASK_PRIORITY_NORMAL, "Monitor_Task");
    /* 输出任务信息 */
    // printf("Task Scheduler Initialized with %d tasks\r\n", TaskScheduler_GetTaskCount());
}
