/**
 ******************************************************************************
 * @file           : task_scheduler.c
 * @brief          : Simple task scheduler implementation
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

#include "task_scheduler.h"
#include <string.h>
#include <stdio.h>

/* 任务表 */
static Task_t task_table[MAX_TASKS];
static uint8_t task_count = 0;

/**
 * @brief 初始化任务调度器
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef TaskScheduler_Init(void)
{
    /* 清空任务表 */
    memset(task_table, 0, sizeof(task_table));
    task_count = 0;
    return HAL_OK;
}

/**
 * @brief 添加任务到调度器
 * @param function: 任务函数指针
 * @param period: 任务执行周期(ms)
 * @param priority: 任务优先级
 * @param name: 任务名称
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef TaskScheduler_AddTask(TaskFunction_t function, uint32_t period,
                                       TaskPriority_t priority, const char* name)
{
    if (task_count >= MAX_TASKS || function == NULL || name == NULL) {
        return HAL_ERROR;
    }
    /* 检查任务名称是否重复 */
    for (uint8_t i = 0; i < task_count; i++) {
        if (strcmp(task_table[i].taskName, name) == 0) {
            return HAL_ERROR; /* 任务名称重复 */
        }
    }

    task_table[task_count].task_function = function;
    task_table[task_count].period = period;
    task_table[task_count].last_run_time = 0;
    task_table[task_count].priority = priority;
    task_table[task_count].state = TASK_READY;
    task_table[task_count].enabled = 1;
    task_table[task_count].taskName = name;

    task_count++;
    return HAL_OK;
}

/**
 * @brief 任务调度器主循环
 */
void TaskScheduler_Run(void)
{
    uint32_t current_time = HAL_GetTick();
    Task_t *ready_task = NULL;
    TaskPriority_t highest_priority = TASK_PRIORITY_IDLE;
    /* 查找最高优先级的就绪任务 */
    for (uint8_t i = 0; i < task_count; i++) {
        if (task_table[i].enabled && task_table[i].state == TASK_READY) {
            /* 检查任务是否到达执行时间 */
            if ((current_time - task_table[i].last_run_time) >= task_table[i].period) {
                /* 选择优先级最高的任务 */
                if (task_table[i].priority > highest_priority) {
                    highest_priority = task_table[i].priority;
                    ready_task = &task_table[i];
                }
            }
        }
    }
    /* 执行选中的任务 */
    if (ready_task != NULL) {
        ready_task->state = TASK_RUNNING;
        ready_task->last_run_time = current_time;
        /* 执行任务函数 */
        if (ready_task->task_function != NULL) {
            ready_task->task_function();
        }
        ready_task->state = TASK_READY;
    }
}

/**
 * @brief 挂起指定任务
 * @param taskName: 任务名称
 */
void TaskScheduler_SuspendTask(const char* taskName)
{
    if (taskName == NULL) return;
    
    for (uint8_t i = 0; i < task_count; i++) {
        if (strcmp(task_table[i].taskName, taskName) == 0) {
            task_table[i].enabled = 0;
            task_table[i].state = TASK_SUSPENDED;
            break;
        }
    }
}

/**
 * @brief 恢复指定任务
 * @param taskName: 任务名称
 */
void TaskScheduler_ResumeTask(const char* taskName)
{
    if (taskName == NULL) return;
    
    for (uint8_t i = 0; i < task_count; i++) {
        if (strcmp(task_table[i].taskName, taskName) == 0) {
            task_table[i].enabled = 1;
            task_table[i].state = TASK_READY;
            task_table[i].last_run_time = HAL_GetTick(); /* 重置执行时间 */
            break;
        }
    }
}

/**
 * @brief 删除指定任务
 * @param taskName: 任务名称
 */
void TaskScheduler_DeleteTask(const char* taskName)
{
    if (taskName == NULL) return;
    
    for (uint8_t i = 0; i < task_count; i++) {
        if (strcmp(task_table[i].taskName, taskName) == 0) {
            /* 将后面的任务前移 */
            for (uint8_t j = i; j < task_count - 1; j++) {
                task_table[j] = task_table[j + 1];
            }
            task_count--;
            
            /* 清空最后一个任务 */
            memset(&task_table[task_count], 0, sizeof(Task_t));
            break;
        }
    }
}

/**
 * @brief 获取系统滴答
 * @retval 系统滴答值
 */
uint32_t TaskScheduler_GetSystemTick(void)
{
    return HAL_GetTick();
}

/**
 * @brief 获取任务数量
 * @retval 当前任务数量
 */
uint8_t TaskScheduler_GetTaskCount(void)
{
    return task_count;
}

/**
 * @brief 打印任务信息（调试用）
 */
void TaskScheduler_PrintTaskInfo(void)
{
    printf("=== Task Scheduler Info ===\r\n");
    printf("Total Tasks: %d/%d\r\n", task_count, MAX_TASKS);
    printf("Current Tick: %lu\r\n", HAL_GetTick());
    printf("------------------------\r\n");
    
    for (uint8_t i = 0; i < task_count; i++) {
        printf("Task[%d]: %s\r\n", i, task_table[i].taskName);
        printf("  Period: %lu ms\r\n", task_table[i].period);
        printf("  Priority: %d\r\n", task_table[i].priority);
        printf("  State: %s\r\n", 
               task_table[i].state == TASK_READY ? "Ready" :
               task_table[i].state == TASK_RUNNING ? "Running" :
               task_table[i].state == TASK_BLOCKED ? "Blocked" : "Suspended");
        printf("  Enabled: %s\r\n", task_table[i].enabled ? "Yes" : "No");
        printf("  Last Run: %lu ms\r\n", task_table[i].last_run_time);
        printf("------------------------\r\n");
    }
}
