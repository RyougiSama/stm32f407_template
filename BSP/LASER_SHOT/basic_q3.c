#include "Emm_V5.h"
#include "gpio.h"
#include "laser_shot_common.h"
#include "task_scheduler.h"

// 状态定义
typedef enum {
    Q3_STATE_INIT,       // 初始化状态
    Q3_STATE_HOMING,     // 回零状态
    Q3_STATE_SEARCHING,  // 搜索状态
    Q3_STATE_TRACKING,   // 追踪状态
    Q3_STATE_COMPLETE    // 完成状态
} Q3State_t;

// 全局变量
bool g_task_basic_q3_running = false;
static Q3State_t q3_state = Q3_STATE_INIT;
static uint32_t q3_last_time = 0;
static uint32_t search_timeout_ms = 30000;  // 30秒搜索超时
static bool search_started = false;         // 搜索启动标志

// 搜索参数
#define X_SEARCH_VELOCITY 30  // X轴搜索速度
#define X_SEARCH_ACC 5        // X轴加速度
#define X_SEARCH_DIR DIR_CW   // 固定搜索方向

// 追踪相关参数
#define CLK_STEP_SMALL 3   // 小步进值
#define CLK_STEP_MEDIUM 5  // 中等步进值
#define CLK_STEP_LARGE 10  // 大步进值

// 检查矩形是否被检测到
static bool IsRectangleDetected(void)
{
    static uint32_t last_valid_detection = 0;
    uint32_t current_time = TaskScheduler_GetSystemTick();

    // 矩形检测有效判断条件：g_curr_center_point 不为 (0,0)
    if (g_curr_center_point.x != 0 || g_curr_center_point.y != 0) {
        last_valid_detection = current_time;
        return true;
    }

    // 如果500ms内有过有效检测，仍然认为有效
    if (current_time - last_valid_detection < 500) {
        return true;
    }

    return false;
}

// 使用Q2相同的追踪逻辑
static void TrackRectangle(void)
{
    const uint16_t vel = 10;
    const uint8_t acc = 5;
    const uint16_t DEADZONE = 3;

    // 计算误差
    int16_t error_x = g_curr_center_point.x - g_sensor_aim_x;
    int16_t error_y = g_curr_center_point.y - g_sensor_aim_y;

    // 如果误差在死区内，任务完成
    if (abs(error_x) < DEADZONE && abs(error_y) < DEADZONE) {
        q3_state = Q3_STATE_COMPLETE;
        return;
    }

    // 根据误差大小选择步进值
    uint16_t step_x, step_y;

    // X轴步进值选择
    if (abs(error_x) < ERROR_THRESHOLD_SMALL) {
        step_x = CLK_STEP_SMALL;
    } else if (abs(error_x) < ERROR_THRESHOLD_MEDIUM) {
        step_x = CLK_STEP_MEDIUM;
    } else {
        step_x = CLK_STEP_LARGE;
    }

    // Y轴步进值选择
    if (abs(error_y) < ERROR_THRESHOLD_SMALL) {
        step_y = CLK_STEP_SMALL;
    } else if (abs(error_y) < ERROR_THRESHOLD_MEDIUM) {
        step_y = CLK_STEP_MEDIUM;
    } else {
        step_y = CLK_STEP_LARGE;
    }

    // 只有误差超过死区才调整
    if (abs(error_x) > DEADZONE) {
        if (error_x > 0) {
            Emm_V5_Pos_Control(STEP_MOTOR_X, DIR_CCW, vel, acc, step_x, false, false);
        } else {
            Emm_V5_Pos_Control(STEP_MOTOR_X, DIR_CW, vel, acc, step_x, false, false);
        }
    }

    HAL_Delay(20);  // 延时等待电机运动

    // Y轴调整（虽然通常不需要，但为了稳妥起见保留逻辑）
    if (abs(error_y) > DEADZONE) {
        if (error_y > 0) {
            Emm_V5_Pos_Control(STEP_MOTOR_Y, DIR_CCW, vel, acc, step_y, false, false);
        } else {
            Emm_V5_Pos_Control(STEP_MOTOR_Y, DIR_CW, vel, acc, step_y, false, false);
        }
    }
}

void Task_BasicQ3_Start(void)
{
    g_task_basic_q3_running = true;
    q3_state = Q3_STATE_INIT;
    q3_last_time = TaskScheduler_GetSystemTick();
    search_started = false;

    // 关闭输出指示GPIO
    HAL_GPIO_WritePin(OUTPUT_TEST_GPIO_Port, OUTPUT_TEST_Pin, GPIO_PIN_RESET);
}

void Task_BasicQ3_Stop(void)
{
    g_task_basic_q3_running = false;
    // 停止电机
    Emm_V5_Stop_Now(STEP_MOTOR_X, false);
    Emm_V5_Stop_Now(STEP_MOTOR_Y, false);
    q3_state = Q3_STATE_INIT;
    search_started = false;
}

bool Task_BasicQ3_IsRunning(void)
{
    return g_task_basic_q3_running;
}

void Task_BasicQ3_Execute(void)
{
    if (!g_task_basic_q3_running) {
        return;
    }

    uint32_t current_time = TaskScheduler_GetSystemTick();

    // 检查搜索是否超时
    if ((q3_state == Q3_STATE_SEARCHING) && (current_time - q3_last_time > search_timeout_ms)) {
        // 搜索超时，停止任务
        Task_BasicQ3_Stop();
        return;
    }

    switch (q3_state) {
        case Q3_STATE_INIT:
            // 初始化并启动回零
            // 触发回零 - 先Y轴后X轴
            Emm_V5_Origin_Trigger_Return(STEP_MOTOR_Y, 0, false);
            HAL_Delay(20);
            Emm_V5_Origin_Trigger_Return(STEP_MOTOR_X, 0, false);
            q3_state = Q3_STATE_HOMING;
            HAL_Delay(20);  // 等待回零完成
            break;

        case Q3_STATE_HOMING:
            // 等待回零完成，简化处理直接进入搜索状态
            q3_last_time = current_time;  // 重置搜索计时器
            q3_state = Q3_STATE_SEARCHING;
            break;

        case Q3_STATE_SEARCHING:
            // 检查是否已检测到矩形
            if (IsRectangleDetected()) {
                // 找到矩形，停止电机
                Emm_V5_Stop_Now(STEP_MOTOR_X, false);
                HAL_Delay(50);  // 短暂等待电机停止
                q3_state = Q3_STATE_TRACKING;
                search_started = false;
                break;
            }

            // 未检测到，继续搜索
            // 使用速度模式控制X电机做单方向扫描
            if (!search_started) {
                Emm_V5_Vel_Control(STEP_MOTOR_X, X_SEARCH_DIR, X_SEARCH_VELOCITY, X_SEARCH_ACC,
                                   false);
                search_started = true;
            }
            break;

        case Q3_STATE_TRACKING:
            if (IsRectangleDetected()) {
                // 找到目标，进行追踪
                TrackRectangle();
            } else {
                // 丢失目标，恢复到搜索状态
                q3_state = Q3_STATE_SEARCHING;
                search_started = false;       // 重置搜索启动标志
                q3_last_time = current_time;  // 重置搜索计时器
            }
            break;

        case Q3_STATE_COMPLETE:
            // 任务完成，点亮指示LED
            HAL_GPIO_WritePin(OUTPUT_TEST_GPIO_Port, OUTPUT_TEST_Pin, GPIO_PIN_SET);
            g_task_basic_q3_running = false;  // 停止任务
            break;
    }
}
