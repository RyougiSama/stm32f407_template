#include "laser_shot_common.h"
#include "oled_user.h"
#include "servo_user.h"  // 添加舵机控制头文件

PixelPoint_t g_curr_center_point;
bool g_task_basic_q2_running = false;

// 传感器相关全局变量定义
uint16_t g_sensor_width = 320;
uint16_t g_sensor_height = 240;
uint16_t g_sensor_aim_x = 160;
uint16_t g_sensor_aim_y = 100;

// 步进控制相关参数
#define PWM_STEP_SMALL 1           // 小步进值，微调用
#define PWM_STEP_MEDIUM 3          // 中等步进值
#define PWM_STEP_LARGE 5           // 大步进值，快速调整用


void Task_BasicQ2_Start(void)
{
    g_task_basic_q2_running = true;
}

void Task_BasicQ2_Excute(void)
{
    // 如果任务未启动或坐标无效，直接返回
    if (!g_task_basic_q2_running || (g_curr_center_point.x == 0 && g_curr_center_point.y == 0)) {
        return;
    }

    // 定义死区范围，当误差小于这个值时不再调整，防止抖动
    const uint16_t DEADZONE = 3;

    // 计算误差
    int16_t error_x = g_curr_center_point.x - g_sensor_aim_x;
    int16_t error_y = g_curr_center_point.y - g_sensor_aim_y;

    // 如果误差在死区内，不做调整
    if (abs(error_x) < DEADZONE && abs(error_y) < DEADZONE) {
        HAL_GPIO_WritePin(OUTPUT_TEST_GPIO_Port, OUTPUT_TEST_Pin, GPIO_PIN_SET);
        g_task_basic_q2_running = false;  // 停止任务
        return;
    }

    // 根据误差大小选择步进值
    uint16_t step_x, step_y;

    // X轴步进值选择
    if (abs(error_x) < ERROR_THRESHOLD_SMALL) {
        step_x = PWM_STEP_SMALL;  // 小误差，小步进
    } else if (abs(error_x) < ERROR_THRESHOLD_MEDIUM) {
        step_x = PWM_STEP_MEDIUM;  // 中等误差，中等步进
    } else {
        step_x = PWM_STEP_LARGE;  // 大误差，大步进
    }

    // Y轴步进值选择
    if (abs(error_y) < ERROR_THRESHOLD_SMALL) {
        step_y = PWM_STEP_SMALL;  // 小误差，小步进
    } else if (abs(error_y) < ERROR_THRESHOLD_MEDIUM) {
        step_y = PWM_STEP_MEDIUM;  // 中等误差，中等步进
    } else {
        step_y = PWM_STEP_LARGE;  // 大误差，大步进
    }

    // 根据误差方向调整舵机PWM值
    // 注意：根据实际安装方向，需要调整符号
    if (error_x > 0) {
        g_servox_duty -= step_x;  // 目标在右侧，舵机右转
    } else if (error_x < 0) {
        g_servox_duty += step_x;  // 目标在左侧，舵机左转
    }

    if (error_y > 0) {
        g_servoy_duty += step_y;  // 目标在下侧，舵机下转
    } else if (error_y < 0) {
        g_servoy_duty -= step_y;  // 目标在上侧，舵机上转
    }

    // 限制PWM范围
    if (g_servox_duty > SERVO_PWM_MAX)
        g_servox_duty = SERVO_PWM_MAX;
    if (g_servox_duty < SERVO_PWM_MIN)
        g_servox_duty = SERVO_PWM_MIN;
    if (g_servoy_duty > SERVO_PWM_MAX)
        g_servoy_duty = SERVO_PWM_MAX;
    if (g_servoy_duty < SERVO_PWM_MIN)
        g_servoy_duty = SERVO_PWM_MIN;

    // 设置舵机位置
    Servo_SetPulseWidth_DirX(g_servox_duty);
    Servo_SetPulseWidth_DirY(g_servoy_duty);
}
