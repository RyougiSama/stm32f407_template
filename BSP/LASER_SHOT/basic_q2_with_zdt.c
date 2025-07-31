#include "Emm_V5.h"
#include "laser_shot_common.h"
#include "task_scheduler.h"

bool g_task_basic_q2_with_zdt_running = false;
uint32_t g_task_basic_q2_with_zdt_start_time = 0;

void Task_BasicQ2_WithZDT_Start(void)
{
    g_task_basic_q2_with_zdt_running = true;
}

void Task_BasicQ2_WithZDT_Stop(void)
{
    g_task_basic_q2_with_zdt_running = false;
}

bool Task_BasicQ2_WithZDT_IsRunning(void)
{
    return g_task_basic_q2_with_zdt_running;
}

static void BasicQ2_Using_Positon(void)
{
    static uint32_t curr_time = 0;
    curr_time = TaskScheduler_GetSystemTick();
    // 定义死区范围，当误差小于这个值时不再调整，防止抖动
    const uint16_t DEADZONE = 3;

    // 计算误差
    int16_t error_x = g_curr_center_point.x - g_sensor_aim_x;
    int16_t error_y = g_curr_center_point.y - g_sensor_aim_y;

    // 如果误差在死区内，不做调整
    if (abs(error_x) < DEADZONE && abs(error_y) < DEADZONE || abs(curr_time - g_task_basic_q2_with_zdt_start_time) > 1900) {
        HAL_GPIO_WritePin(OUTPUT_TEST_GPIO_Port, OUTPUT_TEST_Pin, GPIO_PIN_SET);
        g_task_basic_q2_running = false;  // 停止任务
        return;
    }

    
}

void Task_BasicQ2_WithZDT_Execute(void)
{
    if (!g_task_basic_q2_with_zdt_running) {
        return;
    }
    BasicQ2_Using_Positon();
}
