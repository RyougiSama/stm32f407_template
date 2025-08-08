#include "Emm_V5.h"
#include "gpio.h"
#include "laser_shot_common.h"
#include "pid_controller.h"
#include "task_scheduler.h"
#include "uart_user.h"  // 添加串口用户函数头文件

uint16_t g_sensor_aim_x = 150;
uint16_t g_sensor_aim_y = 130;

// ==================== Q3键盘任务PID参数配置区域 ====================
// 以下参数影响Q3键盘任务的PID追踪响应速度和精度，可根据实际效果调整

// PID控制器参数 - 影响追踪响应速度和稳定性
#define Q3_KEY_PID_KP_VALUE 1.2f   // X轴PID比例系数（针对4秒限制优化）
#define Q3_KEY_PID_KI_VALUE 0.05f  // X轴PID积分系数
#define Q3_KEY_PID_KD_VALUE 0.1f   // X轴PID微分系数

// PID输出限制 - 控制最大步进数，影响追踪速度
#define Q3_KEY_PID_OUTPUT_MAX 15.0f     // 最大输出步进数
#define Q3_KEY_PID_OUTPUT_MIN -15.0f    // 最小输出步进数
#define Q3_KEY_PID_INTEGRAL_MAX 20.0f   // 积分限幅上限
#define Q3_KEY_PID_INTEGRAL_MIN -20.0f  // 积分限幅下限

// 电机控制参数 - 影响实际执行速度（更激进以满足4秒要求）
#define Q3_KEY_MOTOR_VELOCITY 28      // 电机速度（比默认快）
#define Q3_KEY_MOTOR_ACCELERATION 18  // 电机加速度

// 死区和步进限制 - 影响追踪精度和最小动作
#define Q3_KEY_DEADZONE 5   // 死区大小（像素，更小以提高精度）
#define Q3_KEY_MIN_STEP 1   // 最小步进数
#define Q3_KEY_MAX_STEP 20  // 最大步进数（增大以提高大误差时的追踪速度）

// 控制延时 - 影响系统响应速度
#define Q3_KEY_MOTOR_DELAY_MS 20  // 电机命令延时（ms，减小以提高响应速度）

// 任务超时控制
#define Q3_KEY_TIMEOUT_MS 5000  // 总超时时间4秒


// 初始转角配置（45度为基准）
#define Q3_KEY_TURN_VEL 40      // 初始转动电机速度
#define Q3_KEY_TURN_ACC 20      // 初始转动电机加速度
#define Q3_KEY_TURN_45_CLK 400  // 45度转动所需的时钟脉冲数
// =================================================================

// 键盘任务类型枚举
typedef enum {
    Q3_KEY_TASK_NONE = 0,  // 无任务运行
    Q3_KEY_TASK_S5 = 5,    // S5任务：90度（2*45度）
    Q3_KEY_TASK_S6 = 6,    // S6任务：135度（3*45度）
    Q3_KEY_TASK_S7 = 7,    // S7任务：180度（4*45度）
    Q3_KEY_TASK_S8 = 8     // S8任务：225度（5*45度）
} Q3KeyTaskType_t;

// 通用任务状态结构体
typedef struct {
    bool is_running;            // 任务运行标志
    bool pid_initialized;       // PID初始化标志
    uint32_t start_time;        // 任务开始时间
    Q3KeyTaskType_t task_type;  // 任务类型
    PidController_t pid_x;      // X轴PID控制器
} Q3KeyTaskState_t;

// 全局任务状态（只运行一个任务）
static Q3KeyTaskState_t g_q3_key_task_state = {0};

/**
 * @brief 初始化Q3键盘任务PID控制器
 */
static void Q3_Key_PID_Init(void)
{
    // 初始化X轴PID控制器
    PID_Init(&g_q3_key_task_state.pid_x, PID_TYPE_POSITIONAL);
    PID_SetParam(&g_q3_key_task_state.pid_x, Q3_KEY_PID_KP_VALUE, Q3_KEY_PID_KI_VALUE,
                 Q3_KEY_PID_KD_VALUE);
    PID_SetOutputLimit(&g_q3_key_task_state.pid_x, Q3_KEY_PID_OUTPUT_MIN, Q3_KEY_PID_OUTPUT_MAX);
    PID_SetIntegralLimit(&g_q3_key_task_state.pid_x, Q3_KEY_PID_INTEGRAL_MIN,
                         Q3_KEY_PID_INTEGRAL_MAX);

    g_q3_key_task_state.pid_initialized = true;
}

/**
 * @brief 检查任务是否超时
 * @return true 任务超时，false 任务未超时
 */
static bool Q3_Key_Check_Timeout(void)
{
    uint32_t current_time = TaskScheduler_GetSystemTick();
    return (current_time - g_q3_key_task_state.start_time) > Q3_KEY_TIMEOUT_MS;
}

/**
 * @brief X轴PID控制逻辑（只控制X轴）
 * @return true 已对准目标，false 仍在调整中
 */
static bool Q3_Key_PID_Control_X(void)
{
    const uint16_t vel = Q3_KEY_MOTOR_VELOCITY;
    const uint8_t acc = Q3_KEY_MOTOR_ACCELERATION;
    const uint16_t DEADZONE = Q3_KEY_DEADZONE;

    // 计算X轴误差（目标位置 - 当前位置）
    float error_x = (float)(g_sensor_aim_x - g_curr_center_point.x);

    // 如果X轴误差在死区内，认为已对准
    if (abs(error_x) < DEADZONE) {
        return true;  // 已对准
    }

    // 设置PID目标值为0（即消除误差）
    PID_SetTarget(&g_q3_key_task_state.pid_x, 0.0f);

    // 计算PID输出（步进数），将误差作为当前值输入
    float pid_output_x = PID_Compute(&g_q3_key_task_state.pid_x, -error_x);  // 负号使得输出方向正确
    // 限制步进值范围
    uint16_t step_x = (uint16_t)abs(pid_output_x);

    if (step_x > 0 && step_x < Q3_KEY_MIN_STEP)
        step_x = Q3_KEY_MIN_STEP;

    if (step_x > Q3_KEY_MAX_STEP)
        step_x = Q3_KEY_MAX_STEP;

    // 控制X轴电机（根据误差方向直接判断）
    if (abs(error_x) > DEADZONE) {
        if (error_x > 0) {
            // 目标在右侧，需要右转
            Emm_V5_Pos_Control(STEP_MOTOR_X, DIR_CW, vel, acc, step_x, false, false);
        } else {
            // 目标在左侧，需要左转
            Emm_V5_Pos_Control(STEP_MOTOR_X, DIR_CCW, vel, acc, step_x, false, false);
        }
    }

    HAL_Delay(Q3_KEY_MOTOR_DELAY_MS);

    return false;  // 仍在调整中
}

/**
 * @brief 停止Q3键盘任务并清理状态
 */
static void Q3_Key_Task_Stop(void)
{
    g_q3_key_task_state.is_running = false;
    g_q3_key_task_state.pid_initialized = false;
    g_q3_key_task_state.start_time = 0;
    
    // 停止X轴电机
    Emm_V5_Stop_Now(STEP_MOTOR_X, false);

    // 恢复串口中值滤波
    Uart_SetFilterEnabled(true);

    // 打开激光指示器
    HAL_GPIO_WritePin(OUTPUT_TEST_GPIO_Port, OUTPUT_TEST_Pin, GPIO_PIN_SET);
}

/**
 * @brief 通用的Q3键盘任务启动函数
 * @param task_type 任务类型（S5/S6/S7/S8）
 */
static void Q3_Key_Task_Start_Common(Q3KeyTaskType_t task_type)
{
    // 如果已有任务在运行，先停止
    if (g_q3_key_task_state.is_running) {
        Q3_Key_Task_Stop();
        HAL_Delay(100);  // 等待电机停止
    }

    // 关闭串口中值滤波，提高响应速度
    Uart_SetFilterEnabled(false);

    // 初始化任务状态
    g_q3_key_task_state.is_running = true;
    g_q3_key_task_state.task_type = task_type;
    g_q3_key_task_state.start_time = TaskScheduler_GetSystemTick();
    g_q3_key_task_state.pid_initialized = false;

    // 初始化X轴电机位置
    Emm_V5_Origin_Trigger_Return(STEP_MOTOR_X, 0, false);
    HAL_Delay(1000);
    // 计算转角（以45度为基准）
    uint32_t turn_angle_clk = 0;
    switch (task_type) {
        case Q3_KEY_TASK_S5:
            turn_angle_clk = Q3_KEY_TURN_45_CLK * 2;  // 90度
            Emm_V5_Pos_Control(STEP_MOTOR_X, DIR_CW, Q3_KEY_TURN_VEL, Q3_KEY_TURN_ACC, turn_angle_clk,
                       false, false);
            break;
        case Q3_KEY_TASK_S6:
            turn_angle_clk = (uint32_t)(Q3_KEY_TURN_45_CLK / 45.0f * 30.0f);
            Emm_V5_Pos_Control(STEP_MOTOR_X, DIR_CW, Q3_KEY_TURN_VEL, Q3_KEY_TURN_ACC, turn_angle_clk,
                       false, false);
            break;
        case Q3_KEY_TASK_S7:
            turn_angle_clk = Q3_KEY_TURN_45_CLK * 2;  // 90度
            Emm_V5_Pos_Control(STEP_MOTOR_X, DIR_CCW, Q3_KEY_TURN_VEL, Q3_KEY_TURN_ACC, turn_angle_clk,
                       false, false);
            break;
        case Q3_KEY_TASK_S8:
            turn_angle_clk = (uint32_t)(Q3_KEY_TURN_45_CLK / 45.0f * 160.0f);  // 135度
            Emm_V5_Pos_Control(STEP_MOTOR_X, DIR_CW, Q3_KEY_TURN_VEL, Q3_KEY_TURN_ACC, turn_angle_clk,
                       false, false);
            break;
        default:
            break;
    }

    // 初始化PID控制器
    Q3_Key_PID_Init();
}

// ==================== 对外接口函数 ====================

/**
 * @brief S5任务启动函数（90度转角）
 */
void Task_Q3_Key_S5_Start(void)
{
    Q3_Key_Task_Start_Common(Q3_KEY_TASK_S5);
}

/**
 * @brief S6任务启动函数（135度转角）
 */
void Task_Q3_Key_S6_Start(void)
{
    Q3_Key_Task_Start_Common(Q3_KEY_TASK_S6);
}

/**
 * @brief S7任务启动函数（180度转角）
 */
void Task_Q3_Key_S7_Start(void)
{
    Q3_Key_Task_Start_Common(Q3_KEY_TASK_S7);
}

/**
 * @brief S8任务启动函数
 */
void Task_Q3_Key_S8_Start(void)
{
    Q3_Key_Task_Start_Common(Q3_KEY_TASK_S8);
}

/**
 * @brief 通用的Q3键盘任务执行函数
 * - 只控制X轴，Y轴保持不变
 * - 4秒超时自动停止
 * - 达到精度要求时自动停止并打开激光
 */
void Task_Q3_Key_Execute(void)
{
    // 如果任务未运行，直接返回
    if (!g_q3_key_task_state.is_running) {
        return;
    }

    // 检查超时
    if (Q3_Key_Check_Timeout()) {
        Q3_Key_Task_Stop();
        return;
    }

    // 如果当前坐标为(0, 0)，则不执行任何操作
    if (g_curr_center_point.x == 0 && g_curr_center_point.y == 0) {
        return;
    }

    // 确保PID已初始化
    if (!g_q3_key_task_state.pid_initialized) {
        Q3_Key_PID_Init();
    }

    // 执行X轴PID控制
    bool is_aligned = Q3_Key_PID_Control_X();

    // 如果已对准目标，完成任务
    if (is_aligned) {
        // 打开激光指示器
        // HAL_GPIO_WritePin(OUTPUT_TEST_GPIO_Port, OUTPUT_TEST_Pin, GPIO_PIN_SET);

        // 停止任务
        Q3_Key_Task_Stop();
    }
}

/**
 * @brief 检查Q3键盘任务是否正在运行
 * @return true 任务正在运行，false 任务未运行
 */
bool Task_Q3_Key_IsRunning(void)
{
    return g_q3_key_task_state.is_running;
}

/**
 * @brief 获取当前运行的任务类型
 * @return Q3KeyTaskType_t 当前任务类型，如果未运行则返回Q3_KEY_TASK_NONE
 */
Q3KeyTaskType_t Task_Q3_Key_GetCurrentTaskType(void)
{
    if (g_q3_key_task_state.is_running) {
        return g_q3_key_task_state.task_type;
    }
    return Q3_KEY_TASK_NONE;  // 返回明确的枚举值
}

// ==================== 兼容性接口函数 ====================

/**
 * @brief S5任务执行函数（兼容性保持）
 */
void Task_Q3_Key_S5_Execute(void)
{
    Task_Q3_Key_Execute();
}
