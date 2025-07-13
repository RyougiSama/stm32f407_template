/**
 ******************************************************************************
 * @file           : app_tasks.h
 * @brief          : Application tasks header file
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

#ifndef __APP_TASKS_H
#define __APP_TASKS_H

#include "task_scheduler.h"

/* 任务函数声明 */
void Task_LedBlink(void);
void Task_UartProcess(void);
void Task_SystemMonitor(void);
void Task_Idle(void);
void Task_WatchdogFeed(void);
void Task_HeartBeat(void);

/* 任务初始化函数 */
void AppTasks_Init(void);

#endif /* __APP_TASKS_H */
