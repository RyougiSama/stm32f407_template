#include "Emm_V5.h"
#include "gpio.h"
#include "laser_shot_common.h"
#include "task_scheduler.h"
#include "uart_user.h"

// 状态定义
typedef enum {
    Q3_STATE_INIT,       // 初始化状态
    Q3_STATE_HOMING,     // 回零状态
    Q3_STATE_SEARCHING,  // 搜索状态
    Q3_STATE_TRACKING,   // 追踪状态
    Q3_STATE_COMPLETE    // 完成状态
} Q3State_t;

// 回零阶段定义
typedef enum {
    HOMING_PHASE_Y,      // Y轴回零阶段
    HOMING_PHASE_CHECK,  // 检查矩形阶段
    HOMING_PHASE_X,      // X轴回零阶段
    HOMING_PHASE_DONE    // 回零完成阶段
} HomingPhase_t;

// ============== Q3任务配置参数 ==============
// 任务时间控制
#define Q3_TOTAL_TIMEOUT_MS 400000        // 总任务超时时间(ms)
#define Q3_SEARCH_TIMEOUT_MS 250000       // 搜索阶段超时时间(ms)，留1.5秒追踪
#define Q3_INIT_DETECTION_TIME_MS 200   // 初始化检测时间(ms)
#define Q3_DETECTION_VALID_TIME_MS 200  // 检测有效时间(ms)

// 检测逻辑参数
#define Q3_CONSECUTIVE_DETECTION_MIN 1  // 连续检测到目标的最小次数
#define Q3_CONSECUTIVE_ZERO_MAX 3       // 连续接收(0,0)的最大次数

// 搜索参数
#define X_SEARCH_VELOCITY 20      // X轴搜索速度
#define X_SEARCH_ACC 10           // X轴搜索加速度
#define X_SEARCH_DIR DIR_CCW      // 固定搜索方向
#define X_SEARCH_STEP 100         // 每次搜索步长(脉冲数)
#define X_SEARCH_MAX_PULSES 3000  // 最大搜索角度(约半圈)

// 追踪控制参数
#define Q3_TRACK_VELOCITY 20      // 追踪时的电机速度
#define Q3_TRACK_ACC 10           // 追踪时的电机加速度
#define Q3_TRACK_DEADZONE 5       // 追踪死区范围
#define Q3_MOTOR_CMD_DELAY_MS 20  // 电机命令之间的延时(ms)

// 追踪步进参数
#define CLK_STEP_SMALL 5    // 小步进值
#define CLK_STEP_MEDIUM 10  // 中等步进值
#define CLK_STEP_LARGE 15   // 大步进值
// ============================================

// 全局变量
bool g_task_basic_q3_running = false;
static Q3State_t q3_state = Q3_STATE_INIT;
static HomingPhase_t homing_phase = HOMING_PHASE_Y;  // 当前回零阶段
static uint32_t q3_last_time = 0;
static bool search_started = false;           // 搜索启动标志
static uint16_t current_search_position = 0;  // 当前搜索位置
static uint32_t q3_total_start_time = 0;      // 任务总运行时间计时器

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

        // 连续检测到才认为确实找到目标，防止偶发误检
        if (consecutive_detections >= Q3_CONSECUTIVE_DETECTION_MIN) {
            return true;
        }
    } else {
        // 未检测到矩形(收到0,0)
        consecutive_zeros++;

        // 连续多次未检测到，认为确实丢失目标
        if (consecutive_zeros >= Q3_CONSECUTIVE_ZERO_MAX) {
            consecutive_detections = 0;
            // 超过有效时间未检测到，认为丢失
            if (current_time - last_valid_detection > Q3_DETECTION_VALID_TIME_MS) {
                return false;
            }
        }
    }

    // 使用配置的有效期，加快响应速度
    if (current_time - last_valid_detection < Q3_DETECTION_VALID_TIME_MS) {
        return consecutive_detections > 0;
    }

    return false;
}

// 使用Q2相同的追踪逻辑
static void TrackRectangle(void)
{
    const uint16_t vel = Q3_TRACK_VELOCITY;       // 使用配置的追踪速度
    const uint8_t acc = Q3_TRACK_ACC;             // 使用配置的追踪加速度
    const uint16_t DEADZONE = Q3_TRACK_DEADZONE;  // 使用配置的死区范围

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
        HAL_Delay(Q3_MOTOR_CMD_DELAY_MS);  // 使用配置的命令延时
    }

    if (abs(error_y) > DEADZONE) {
        Emm_V5_Pos_Control(STEP_MOTOR_Y, (error_y > 0) ? DIR_CCW : DIR_CW, vel, acc, step_y, false,
                           false);
        y_adjusted = true;
    }

    // Y轴调整后的延时
    if (y_adjusted) {
        HAL_Delay(Q3_MOTOR_CMD_DELAY_MS);  // 使用配置的命令延时
    }
}

void Task_BasicQ3_Start(void)
{
    g_task_basic_q3_running = true;
    q3_state = Q3_STATE_INIT;
    homing_phase = HOMING_PHASE_Y;  // 从Y轴回零开始
    q3_last_time = TaskScheduler_GetSystemTick();
    q3_total_start_time = 0;  // 重置总计时器，确保全新开始
    search_started = false;
    current_search_position = 0;  // 重置搜索位置计数器

    // Q3任务开始时禁用中值滤波，提高响应速度
    Uart_SetFilterEnabled(false);

    // 关闭输出指示GPIO
    HAL_GPIO_WritePin(OUTPUT_TEST_GPIO_Port, OUTPUT_TEST_Pin, GPIO_PIN_RESET);
}

void Task_BasicQ3_Stop(void)
{
    g_task_basic_q3_running = false;

    // Q3任务结束时重新启用中值滤波
    Uart_SetFilterEnabled(true);

    // 停止电机
    Emm_V5_Stop_Now(STEP_MOTOR_X, false);
    HAL_Delay(20);
    Emm_V5_Stop_Now(STEP_MOTOR_Y, false);
    // 重置所有状态变量
    q3_state = Q3_STATE_INIT;
    homing_phase = HOMING_PHASE_Y;  // 重置回零阶段
    search_started = false;
    current_search_position = 0;  // 重置搜索位置
    q3_total_start_time = 0;      // 重置总时间计时器

    // 关闭输出指示GPIO
    HAL_GPIO_WritePin(OUTPUT_TEST_GPIO_Port, OUTPUT_TEST_Pin, GPIO_PIN_RESET);
}

bool Task_BasicQ3_IsRunning(void)
{
    return g_task_basic_q3_running;
}

void Task_BasicQ3_Execute(void)
{
    if (!g_task_basic_q3_running) {
        // 任务未运行时确保总计时器已重置（防御性编程）
        if (q3_total_start_time != 0) {
            q3_total_start_time = 0;
        }
        return;
    }

    uint32_t current_time = TaskScheduler_GetSystemTick();

    // 初始化总任务计时器
    if (q3_total_start_time == 0) {
        q3_total_start_time = current_time;
    }

    // 无论如何不超过总时限
    if (current_time - q3_total_start_time > Q3_TOTAL_TIMEOUT_MS) {
        Task_BasicQ3_Stop();
        q3_total_start_time = 0;
        return;
    }

    // 检查搜索是否超时
    if ((q3_state == Q3_STATE_SEARCHING) && (current_time - q3_last_time > Q3_SEARCH_TIMEOUT_MS)) {
        // 搜索超时，停止任务
        Task_BasicQ3_Stop();
        q3_total_start_time = 0;
        return;
    }

    switch (q3_state) {
        case Q3_STATE_INIT: {
            // 由于系统上电时Y轴已归零，给予充足时间检查矩形是否已在视野中
            static uint32_t init_detection_start = 0;
            static uint8_t detection_attempts = 0;

            // 初始化检测计时器
            if (init_detection_start == 0) {
                init_detection_start = current_time;
                detection_attempts = 0;
            }

            // X轴位置设为初始值0
            current_search_position = 0;

            // 重置所有计时器，确保任务能正常执行
            q3_last_time = current_time;

            // 给予配置时间进行多次检测，确保不遗漏已在视野中的目标
            if (current_time - init_detection_start < Q3_INIT_DETECTION_TIME_MS) {
                if (IsRectangleDetected()) {
                    // 检测到矩形，重新启用滤波并进入追踪状态
                    Uart_SetFilterEnabled(true);
                    init_detection_start = 0;
                    q3_state = Q3_STATE_TRACKING;
                    return;
                }
                // 每50ms尝试一次检测，总共尝试4次
                if ((current_time - init_detection_start) % 50 == 0) {
                    detection_attempts++;
                }
            } else {
                // 配置时间内未检测到矩形，进入搜索流程
                init_detection_start = 0;  // 重置计时器

                // 矩形不可见，需要X轴回零后搜索
                Emm_V5_Origin_Trigger_Return(STEP_MOTOR_X, 1, false);
                HAL_Delay(100);                 // 确保命令发送
                homing_phase = HOMING_PHASE_X;  // 设置为X轴回零阶段
                q3_state = Q3_STATE_HOMING;
            }
            break;
        }

        case Q3_STATE_HOMING: {
            static uint32_t homing_start_time = 0;

            // 首次进入当前阶段时初始化计时器
            if (homing_start_time == 0) {
                homing_start_time = current_time;
            }

            // 根据回零阶段执行不同操作
            switch (homing_phase) {
                case HOMING_PHASE_Y:
                    // Y轴在系统初始化时已回零，跳过此阶段
                    homing_start_time = 0;
                    homing_phase = HOMING_PHASE_X;  // 直接进入X轴回零
                    break;

                case HOMING_PHASE_CHECK:
                    // 此阶段已在INIT中处理，跳过
                    homing_start_time = 0;
                    homing_phase = HOMING_PHASE_X;
                    break;

                case HOMING_PHASE_X:
                    // 等待X轴回零完成(200ms)
                    if (current_time - homing_start_time > 200) {
                        homing_start_time = 0;
                        homing_phase = HOMING_PHASE_DONE;
                    }
                    break;

                case HOMING_PHASE_DONE:
                    // X轴回零完成，进入搜索状态
                    homing_start_time = 0;
                    q3_last_time = current_time;
                    homing_phase = HOMING_PHASE_Y;  // 重置回零阶段
                    q3_state = Q3_STATE_SEARCHING;
                    break;
            }
            break;
        }

        case Q3_STATE_SEARCHING:
            // 检查是否已检测到矩形
            if (IsRectangleDetected()) {
                // 找到矩形，停止电机
                Emm_V5_Stop_Now(STEP_MOTOR_X, false);
                HAL_Delay(20);  // 短暂等待电机停止

                // 进入追踪状态时重新启用滤波，提高追踪精度
                Uart_SetFilterEnabled(true);

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
                    return;
                }

                // 增加搜索位置
                current_search_position += X_SEARCH_STEP;

                // 使用位置模式控制，向指定方向移动一步
                Emm_V5_Pos_Control(STEP_MOTOR_X, X_SEARCH_DIR, X_SEARCH_VELOCITY, X_SEARCH_ACC,
                                   current_search_position, false, false);

                // 标记搜索已启动，等待一段时间后检查结果
                search_started = true;

                // 确保搜索计时器已设置
                if (q3_last_time == 0 || current_search_position == X_SEARCH_STEP) {
                    q3_last_time = current_time;  // 设置搜索起始时间
                }

                // 确保电机命令执行后有短暂等待，以便视觉系统可以稳定
                HAL_Delay(20);
            } else {
                // 等待电机到位或一定时间后，重置搜索标志，进行下一步搜索
                if (current_time - q3_last_time > 200) {
                    search_started = false;
                }
            }
            break;

        case Q3_STATE_TRACKING:
            if (IsRectangleDetected()) {
                // 找到目标，进行追踪
                TrackRectangle();
            } else {
                // 丢失目标，立即停止电机
                Emm_V5_Stop_Now(STEP_MOTOR_X, false);
                HAL_Delay(20);
                Emm_V5_Stop_Now(STEP_MOTOR_Y, false);

                // 重新禁用滤波以提高搜索响应速度
                Uart_SetFilterEnabled(false);

                // 恢复到搜索状态
                q3_state = Q3_STATE_SEARCHING;
                search_started = false;  // 重置搜索启动标志

                // 继续从当前位置搜索，不重置位置计数器
                // 更新搜索计时器，给予足够的剩余搜索时间
                q3_last_time = current_time;
            }
            break;

        case Q3_STATE_COMPLETE:
            // 任务完成
            HAL_GPIO_WritePin(OUTPUT_TEST_GPIO_Port, OUTPUT_TEST_Pin, GPIO_PIN_SET);
            g_task_basic_q3_running = false;  // 停止任务
            q3_total_start_time = 0;          // 重置总时间计时器
            break;
    }
}
