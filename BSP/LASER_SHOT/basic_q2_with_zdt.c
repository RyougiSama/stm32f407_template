#include "Emm_V5.h"
#include "laser_shot_common.h"
#include "task_scheduler.h"

bool g_task_basic_q2_with_zdt_running = false;
uint32_t g_task_basic_q2_with_zdt_start_time = 0;

PixelPoint_t g_curr_center_point;
uint16_t g_sensor_width = 320;
uint16_t g_sensor_height = 240;

uint16_t g_sensor_aim_x = 160;
uint16_t g_sensor_aim_y = 140;

// 步进控制相关参数
#define CLK_STEP_SMALL  2            // 小步进值，微调用
#define CLK_STEP_MEDIUM 5           // 中等步进值
#define CLK_STEP_LARGE  10           // 大步进值，快速调整用

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
    if (g_curr_center_point.x == 0 && g_curr_center_point.y == 0) {
        // 如果当前坐标为(0, 0)，则不执行任何操作
        return;
    }

    static uint32_t curr_time = 0;
    curr_time = TaskScheduler_GetSystemTick();

    const uint16_t vel = 10;
    const uint8_t acc = 5;
    // 定义死区范围，当误差小于这个值时不再调整，防止抖动
    const uint16_t DEADZONE = 2;
    // 计算误差
    int16_t error_x = g_curr_center_point.x - g_sensor_aim_x;
    int16_t error_y = g_curr_center_point.y - g_sensor_aim_y;

    // 如果误差在死区内，不做调整
    if (abs(error_x) < DEADZONE && abs(error_y) < DEADZONE
         || (abs(curr_time - g_task_basic_q2_with_zdt_start_time) > 1990)) {
        HAL_GPIO_WritePin(OUTPUT_TEST_GPIO_Port, OUTPUT_TEST_Pin, GPIO_PIN_SET);
        g_task_basic_q2_with_zdt_running = false;  // 停止任务
        return;
    }

    // 根据误差大小选择步进值
    uint16_t step_x, step_y;

    // X轴步进值选择
    if (abs(error_x) < ERROR_THRESHOLD_SMALL) {
        step_x = CLK_STEP_SMALL;  // 小误差，小步进
    } else if (abs(error_x) < ERROR_THRESHOLD_MEDIUM) {
        step_x = CLK_STEP_MEDIUM;  // 中等误差，中等步进
    } else {
        step_x = CLK_STEP_LARGE;  // 大误差，大步进
    }

    // Y轴步进值选择
    if (abs(error_y) < ERROR_THRESHOLD_SMALL) {
        step_y = CLK_STEP_SMALL;  // 小误差，小步进
    } else if (abs(error_y) < ERROR_THRESHOLD_MEDIUM) {
        step_y = CLK_STEP_MEDIUM;  // 中等误差，中等步进
    } else {
        step_y = CLK_STEP_LARGE;  // 大误差，大步进
    }

    if (error_x > 0) {
        Emm_V5_Pos_Control(STEP_MOTOR_X, DIR_CCW, vel, acc, step_x, false, false);  // 目标在右侧，舵机右转
    } else if (error_x < 0) {
        Emm_V5_Pos_Control(STEP_MOTOR_X, DIR_CW, vel, acc, step_x, false, false);  // 目标在左侧，舵机左转
    }
    HAL_Delay(20);
    if (error_y > 0) {
        Emm_V5_Pos_Control(STEP_MOTOR_Y, DIR_CCW, vel, acc, step_y, false, false);  // 目标在下方，舵机下转
    } else if (error_y < 0) {
        Emm_V5_Pos_Control(STEP_MOTOR_Y, DIR_CW, vel, acc, step_y, false, false);  // 目标在上方，舵机上转
    }
}

void Task_BasicQ2_WithZDT_Execute(void)
{
    if (!g_task_basic_q2_with_zdt_running) {
        return;
    }
    BasicQ2_Using_Positon();
}
