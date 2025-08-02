#ifndef __LASER_SHOT_COMMON_H
#define __LASER_SHOT_COMMON_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#define ERROR_THRESHOLD_SMALL 10   // 小误差阈值
#define ERROR_THRESHOLD_MEDIUM 50  // 中等误差阈值

// 激光追踪控制模式
typedef enum {
    TRACK_MODE_STEP = 0,    // 步进控制模式（原始方式）
    TRACK_MODE_PID          // PID控制模式
} TrackMode_t;

extern uint16_t g_sensor_width;
extern uint16_t g_sensor_height;
extern uint16_t g_sensor_aim_x;
extern uint16_t g_sensor_aim_y;
extern uint32_t g_task_basic_q2_with_zdt_start_time;

extern bool g_task_basic_q2_with_zdt_running;
extern bool g_task_basic_q3_running;

typedef struct {
    uint16_t x;  // X坐标
    uint16_t y;  // Y坐标
} PixelPoint_t;

extern PixelPoint_t g_curr_center_point;

// void Task_BasicQ2_Start(void);
// void Task_BasicQ2_Excute(void);

void Task_BasicQ2_WithZDT_Start(void);
void Task_BasicQ2_WithZDT_Execute(void);
void Task_BasicQ2_WithZDT_Stop(void);
bool Task_BasicQ2_WithZDT_IsRunning(void);

void Task_BasicQ3_Start(void);
void Task_BasicQ3_Execute(void);
void Task_BasicQ3_Stop(void);
bool Task_BasicQ3_IsRunning(void);

// 激光追踪功能函数声明
void Laser_TrackAimPoint_Start(void);
void Laser_TrackAimPoint_Stop(void);
bool Laser_TrackAimPoint_IsRunning(void);
void Laser_TrackAimPoint_SetMode(TrackMode_t mode);
void Laser_TrackAimPoint(void);

#endif
