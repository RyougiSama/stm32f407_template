#include "laser_shot_common.h"
#include "Emm_V5.h"
#include "task_scheduler.h"
#include "gpio.h"

// 激光追踪任务相关变量
static bool g_laser_track_running = false;
static uint32_t g_laser_track_start_time = 0;

// 步进控制相关参数（与Q2保持一致）
#define LASER_CLK_STEP_SMALL  3     // 小步进值，微调用
#define LASER_CLK_STEP_MEDIUM 5     // 中等步进值
#define LASER_CLK_STEP_LARGE  10    // 大步进值，快速调整用

/**
 * @brief 开始激光追踪任务
 */
void Laser_TrackAimPoint_Start(void)
{
    g_laser_track_running = true;
    g_laser_track_start_time = TaskScheduler_GetSystemTick();
    
    // 立即打开激光指示器
    HAL_GPIO_WritePin(OUTPUT_TEST_GPIO_Port, OUTPUT_TEST_Pin, GPIO_PIN_SET);
}

/**
 * @brief 停止激光追踪任务
 */
void Laser_TrackAimPoint_Stop(void)
{
    g_laser_track_running = false;
    
    // 关闭激光指示器
    HAL_GPIO_WritePin(OUTPUT_TEST_GPIO_Port, OUTPUT_TEST_Pin, GPIO_PIN_RESET);
}

/**
 * @brief 检查激光追踪任务是否正在运行
 */
bool Laser_TrackAimPoint_IsRunning(void)
{
    return g_laser_track_running;
}

/**
 * @brief 激光追踪瞄准点功能实现
 * 区别于Q2：先打开激光，然后持续追踪目标点（无超时限制）
 */
void Laser_TrackAimPoint(void)
{
    // 如果任务未启动，直接返回
    if (!g_laser_track_running) {
        return;
    }
    
    // 如果当前坐标为(0, 0)，则不执行任何操作
    if (g_curr_center_point.x == 0 && g_curr_center_point.y == 0) {
        return;
    }

    const uint16_t vel = 10;  // 电机速度
    const uint8_t acc = 5;    // 电机加速度
    
    // 定义死区范围，当误差小于这个值时不再调整，防止抖动
    const uint16_t DEADZONE = 3;
    
    // 计算误差
    int16_t error_x = g_curr_center_point.x - g_sensor_aim_x;
    int16_t error_y = g_curr_center_point.y - g_sensor_aim_y;

    // 如果误差在死区内，不做调整（但保持激光开启，继续追踪）
    if (abs(error_x) < DEADZONE && abs(error_y) < DEADZONE) {
        return;
    }

    // 根据误差大小选择步进值
    uint16_t step_x, step_y;

    // X轴步进值选择
    if (abs(error_x) < ERROR_THRESHOLD_SMALL) {
        step_x = LASER_CLK_STEP_SMALL;  // 小误差，小步进
    } else if (abs(error_x) < ERROR_THRESHOLD_MEDIUM) {
        step_x = LASER_CLK_STEP_MEDIUM;  // 中等误差，中等步进
    } else {
        step_x = LASER_CLK_STEP_LARGE;  // 大误差，大步进
    }

    // Y轴步进值选择
    if (abs(error_y) < ERROR_THRESHOLD_SMALL) {
        step_y = LASER_CLK_STEP_SMALL;  // 小误差，小步进
    } else if (abs(error_y) < ERROR_THRESHOLD_MEDIUM) {
        step_y = LASER_CLK_STEP_MEDIUM;  // 中等误差，中等步进
    } else {
        step_y = LASER_CLK_STEP_LARGE;  // 大误差，大步进
    }

    // 控制X轴电机
    if (error_x > 0) {
        Emm_V5_Pos_Control(STEP_MOTOR_X, DIR_CCW, vel, acc, step_x, false, false);  // 目标在右侧，电机右转
    } else if (error_x < 0) {
        Emm_V5_Pos_Control(STEP_MOTOR_X, DIR_CW, vel, acc, step_x, false, false);   // 目标在左侧，电机左转
    }
    
    // 添加延时，避免指令冲突
    HAL_Delay(20);
    
    // 控制Y轴电机
    if (error_y > 0) {
        Emm_V5_Pos_Control(STEP_MOTOR_Y, DIR_CCW, vel, acc, step_y, false, false);  // 目标在下方，电机下转
    } else if (error_y < 0) {
        Emm_V5_Pos_Control(STEP_MOTOR_Y, DIR_CW, vel, acc, step_y, false, false);   // 目标在上方，电机上转
    }
}
