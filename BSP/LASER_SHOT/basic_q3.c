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
static uint32_t search_timeout_ms = 2500;  // 2.5秒搜索超时，留1.5秒追踪
static bool search_started = false;        // 搜索启动标志
static uint16_t current_search_position = 0;  // 当前搜索位置

// 搜索参数
#define X_SEARCH_VELOCITY 30  // 提高X轴搜索速度
#define X_SEARCH_ACC 10       // 提高加速度
#define X_SEARCH_DIR DIR_CW   // 固定搜索方向
#define X_SEARCH_STEP 200     // 每次搜索步长(脉冲数)
#define X_SEARCH_MAX_PULSES 1600  // 最大搜索角度(约半圈)

// 追踪相关参数
#define CLK_STEP_SMALL 5    // 提高小步进值
#define CLK_STEP_MEDIUM 10  // 提高中等步进值
#define CLK_STEP_LARGE 15   // 提高大步进值

// 检查矩形是否被检测到
static bool IsRectangleDetected(void)
{
    static uint32_t last_valid_detection = 0;
    static uint8_t consecutive_zeros = 0;       // 连续接收到(0,0)的次数
    static uint8_t consecutive_detections = 0;  // 连续检测到矩形的次数
    uint32_t current_time = TaskScheduler_GetSystemTick();

    // 处理视觉模块返回的坐标
    if (g_curr_center_point.x != 0 || g_curr_center_point.y != 0) {
        // 检测到矩形，重置连续零计数，增加连续检测计数
        consecutive_zeros = 0;
        consecutive_detections++;
        last_valid_detection = current_time;

        // 连续2次检测到才认为确实找到目标，防止偶发误检
        if (consecutive_detections >= 2) {
            return true;
        }
    } else {
        // 未检测到矩形(收到0,0)
        consecutive_zeros++;

        // 连续3次未检测到，认为确实丢失目标
        if (consecutive_zeros >= 3) {
            consecutive_detections = 0;
            // 超过200ms未检测到，认为丢失
            if (current_time - last_valid_detection > 200) {
                return false;
            }
        }
    }

    // 使用更短的有效期(200ms)，加快响应速度
    if (current_time - last_valid_detection < 200) {
        return consecutive_detections > 0;
    }

    return false;
}

// 使用Q2相同的追踪逻辑
static void TrackRectangle(void)
{
    const uint16_t vel = 20;      // 提高速度
    const uint8_t acc = 10;       // 提高加速度
    const uint16_t DEADZONE = 5;  // 适当放宽死区，减少微调次数

    // 计算误差
    int16_t error_x = g_curr_center_point.x - g_sensor_aim_x;
    int16_t error_y = g_curr_center_point.y - g_sensor_aim_y;

    // 如果误差在死区内，任务完成
    if (abs(error_x) < DEADZONE && abs(error_y) < DEADZONE) {
        q3_state = Q3_STATE_COMPLETE;
        return;
    }

    // 同时调整X和Y轴，减少总追踪时间
    // 根据误差大小选择步进值
    uint16_t step_x = CLK_STEP_LARGE;
    uint16_t step_y = CLK_STEP_LARGE;

    // 只针对小误差时减小步进
    if (abs(error_x) < ERROR_THRESHOLD_SMALL) {
        step_x = CLK_STEP_MEDIUM;
    }

    if (abs(error_y) < ERROR_THRESHOLD_SMALL) {
        step_y = CLK_STEP_MEDIUM;
    }

    // 同时控制X和Y轴，确保命令间有延时
    bool y_adjusted = false;

    if (abs(error_x) > DEADZONE) {
        Emm_V5_Pos_Control(STEP_MOTOR_X, (error_x > 0) ? DIR_CCW : DIR_CW, vel, acc, step_x, false,
                           false);
        HAL_Delay(20);  // 添加命令之间的延时
    }

    if (abs(error_y) > DEADZONE) {
        Emm_V5_Pos_Control(STEP_MOTOR_Y, (error_y > 0) ? DIR_CCW : DIR_CW, vel, acc, step_y, false,
                           false);
        y_adjusted = true;
    }

    // Y轴调整后的延时
    if (y_adjusted) {
        HAL_Delay(20);
    }
}

void Task_BasicQ3_Start(void)
{
    g_task_basic_q3_running = true;
    q3_state = Q3_STATE_INIT;
    q3_last_time = TaskScheduler_GetSystemTick();
    search_started = false;
    current_search_position = 0;  // 重置搜索位置计数器

    // 关闭输出指示GPIO
    HAL_GPIO_WritePin(OUTPUT_TEST_GPIO_Port, OUTPUT_TEST_Pin, GPIO_PIN_RESET);
}

void Task_BasicQ3_Stop(void)
{
    g_task_basic_q3_running = false;
    // 停止电机
    Emm_V5_Stop_Now(STEP_MOTOR_X, false);
    HAL_Delay(20);
    Emm_V5_Stop_Now(STEP_MOTOR_Y, false);
    q3_state = Q3_STATE_INIT;
    search_started = false;
    current_search_position = 0;  // 重置搜索位置
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

    static uint32_t q3_total_start_time = 0;
    uint32_t current_time = TaskScheduler_GetSystemTick();

    // 初始化总任务计时器
    if (q3_total_start_time == 0) {
        q3_total_start_time = current_time;
    }

    // 无论如何不超过4秒总时限
    if (current_time - q3_total_start_time > 4000) {
        Task_BasicQ3_Stop();
        q3_total_start_time = 0;
        return;
    }

    // 检查搜索是否超时
    if ((q3_state == Q3_STATE_SEARCHING) && (current_time - q3_last_time > search_timeout_ms)) {
        // 搜索超时，停止任务
        Task_BasicQ3_Stop();
        q3_total_start_time = 0;
        return;
    }

    switch (q3_state) {
        case Q3_STATE_INIT:
            // 先检查是否已经能看到矩形，如果能直接跳到追踪状态
            if (IsRectangleDetected()) {
                q3_state = Q3_STATE_TRACKING;
                break;
            }
            // 否则执行回零操作
            // 只回零Y轴，X轴已在一侧极限位置
            Emm_V5_Origin_Trigger_Return(STEP_MOTOR_Y, 0, false);
            HAL_Delay(100);  // 增加延时，确保命令执行
            // X轴位置设为初始值0
            current_search_position = 0;
            q3_state = Q3_STATE_HOMING;
            break;

        case Q3_STATE_HOMING: {
            // 使用固定延时等待回零完成
            static uint32_t homing_start_time = 0;
            if (homing_start_time == 0) {
                homing_start_time = current_time;
            }

            // 回零等待，然后无论如何进入搜索状态
            if (current_time - homing_start_time > 100) {
                homing_start_time = 0;
                q3_last_time = current_time;  // 重置搜索计时器
                q3_state = Q3_STATE_SEARCHING;
            }
            break;
        }

        case Q3_STATE_SEARCHING:
            // 检查是否已检测到矩形
            if (IsRectangleDetected()) {
                // 找到矩形，停止电机
                Emm_V5_Stop_Now(STEP_MOTOR_X, false);
                HAL_Delay(20);  // 短暂等待电机停止
                q3_state = Q3_STATE_TRACKING;
                search_started = false;
                break;
            }

            // 未检测到，继续搜索
            // 使用位置模式进行步进搜索，防止超过云台结构限位
            if (!search_started) {
                // 检查是否超出最大搜索范围
                if (current_search_position >= X_SEARCH_MAX_PULSES) {
                    // 超出范围，搜索失败
                    Task_BasicQ3_Stop();
                    printf("Q3 search failed: reached max range\r\n");
                    return;
                }
                
                // 增加搜索位置
                current_search_position += X_SEARCH_STEP;
                
                // 使用位置模式控制，向指定方向移动一步
                Emm_V5_Pos_Control(STEP_MOTOR_X, X_SEARCH_DIR, X_SEARCH_VELOCITY,
                                 X_SEARCH_ACC, current_search_position, false, false);
                
                // 标记搜索已启动，等待一段时间后检查结果
                search_started = true;
                q3_last_time = current_time; // 更新计时
            } else {
                // 等待电机到位或一定时间后，重置搜索标志，进行下一步搜索
                if (current_time - q3_last_time > 100) {
                    search_started = false;
                }
            }
            break;

        case Q3_STATE_TRACKING:
            if (IsRectangleDetected()) {
                // 找到目标，进行追踪
                TrackRectangle();
            } else {
                // 丢失目标，恢复到搜索状态
                q3_state = Q3_STATE_SEARCHING;
                search_started = false;  // 重置搜索启动标志
                
                // 继续从当前位置搜索，不重置位置计数器
                // 只给予有限的额外搜索时间，确保4秒内能完成
                uint32_t elapsed = current_time - q3_last_time;
                if (elapsed < search_timeout_ms) {
                    q3_last_time = current_time - (search_timeout_ms - 1000);
                } else {
                    q3_last_time = current_time;
                }
            }
            break;

        case Q3_STATE_COMPLETE:
            // 任务完成
            HAL_GPIO_WritePin(OUTPUT_TEST_GPIO_Port, OUTPUT_TEST_Pin, GPIO_PIN_SET);
            g_task_basic_q3_running = false;  // 停止任务
            break;
    }
}
