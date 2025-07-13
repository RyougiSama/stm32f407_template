/**
 ******************************************************************************
 * @file           : task_scheduler.h
 * @brief          : Simple task scheduler header file.
 *                   Remember to call TaskScheduler_Run() in the main loop!!!
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

#ifndef __TASK_SCHEDULER_H
#define __TASK_SCHEDULER_H

#include "main.h"

/* 任务状态定义 */
typedef enum {
    TASK_READY = 0,
    TASK_RUNNING,
    TASK_BLOCKED,
    TASK_SUSPENDED
} TaskState_t;

/* 任务优先级定义 */
typedef enum {
    TASK_PRIORITY_IDLE = 0,
    TASK_PRIORITY_LOW,
    TASK_PRIORITY_NORMAL,
    TASK_PRIORITY_HIGH,
    TASK_PRIORITY_CRITICAL
} TaskPriority_t;

/* 任务函数类型定义 */
typedef void (*TaskFunction_t)(void);

/* 任务控制块 */
typedef struct {
    TaskFunction_t task_function;  /* 任务函数指针 */
    uint32_t period;               /* 任务执行周期(ms) */
    uint32_t last_run_time;          /* 上次执行时间 */
    TaskPriority_t priority;       /* 任务优先级 */
    TaskState_t state;             /* 任务状态 */
    uint8_t enabled;               /* 任务使能标志 */
    const char* taskName;          /* 任务名称 */
} Task_t;

/* 最大任务数量 */
#define MAX_TASKS 10

/* 任务调度器API */
HAL_StatusTypeDef TaskScheduler_Init(void);
HAL_StatusTypeDef TaskScheduler_AddTask(TaskFunction_t function, uint32_t period,
                                       TaskPriority_t priority, const char* name);
void TaskScheduler_Run(void);   // 该任务必须在主循环中调用!!!
void TaskScheduler_SuspendTask(const char* taskName);
void TaskScheduler_ResumeTask(const char* taskName);
void TaskScheduler_DeleteTask(const char* taskName);
uint32_t TaskScheduler_GetSystemTick(void);
uint8_t TaskScheduler_GetTaskCount(void);
void TaskScheduler_PrintTaskInfo(void);

#endif /* __TASK_SCHEDULER_H */
