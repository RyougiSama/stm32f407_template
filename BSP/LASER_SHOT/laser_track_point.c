#include "Emm_V5.h"
#include "gpio.h"
#include "laser_shot_common.h"
#include "pid_controller.h"
#include "task_scheduler.h"

// ==================== PID追踪参数配置区域 ====================
// 以下参数影响PID追踪的响应速度和精度，可根据实际效果调整

// PID控制器参数 - 影响追踪响应速度和稳定性
#define PID_KP_VALUE        1.5f    // 比例系数：增大可提高响应速度，过大会振荡
#define PID_KI_VALUE        0.02f   // 积分系数：消除稳态误差，过大会超调
#define PID_KD_VALUE        0.15f   // 微分系数：改善动态性能，减少超调

// PID输出限制 - 控制最大步进数，影响追踪速度
#define PID_OUTPUT_MAX      80.0f   // 最大输出步进数（增大可提高追踪速度）
#define PID_OUTPUT_MIN      -80.0f  // 最小输出步进数
#define PID_INTEGRAL_MAX    25.0f   // 积分限幅上限（防止积分饱和）
#define PID_INTEGRAL_MIN    -25.0f  // 积分限幅下限

// 电机控制参数 - 影响实际执行速度
#define PID_MOTOR_VELOCITY  40      // PID模式电机速度（增大可提高追踪速度）
#define PID_MOTOR_ACCELERATION 20   // PID模式电机加速度（影响启动响应）

// 死区和步进限制 - 影响追踪精度和最小动作
#define PID_DEADZONE        2       // 死区大小（像素）：减小提高精度
#define PID_MIN_STEP        1       // 最小步进数：减小提高精度
#define PID_MAX_STEP        30      // 最大步进数：增大提高大误差时的追踪速度

// 控制延时 - 影响系统响应速度
#define PID_MOTOR_DELAY_MS  10      // 电机命令间延时（ms）：减小可提高响应速度

// 调整建议：
// 1. 追踪太慢：增大PID_KP_VALUE、PID_MOTOR_VELOCITY、PID_MAX_STEP
// 2. 追踪振荡：减小PID_KP_VALUE、PID_KD_VALUE，检查PID_MOTOR_DELAY_MS
// 3. 精度不够：减小PID_DEADZONE、PID_MIN_STEP
// 4. 响应迟钝：减小PID_MOTOR_DELAY_MS，增大PID_MOTOR_ACCELERATION
// =============================================================

// 激光追踪任务相关变量
static bool g_laser_track_running = false;
static TrackMode_t g_track_mode = TRACK_MODE_STEP;  // 默认使用步进控制

// 步进控制相关参数（与Q2保持一致）
#define LASER_CLK_STEP_SMALL 3   // 小步进值，微调用
#define LASER_CLK_STEP_MEDIUM 5  // 中等步进值
#define LASER_CLK_STEP_LARGE 10  // 大步进值，快速调整用

// PID控制器实例
static PidController_t g_laser_track_pid_x;  // X轴PID控制器
static PidController_t g_laser_track_pid_y;  // Y轴PID控制器

/**
 * @brief 初始化激光追踪PID控制器
 */
static void Laser_TrackPID_Init(void)
{
    // 初始化X轴PID控制器
    PID_Init(&g_laser_track_pid_x, PID_TYPE_POSITIONAL);
    PID_SetParam(&g_laser_track_pid_x, PID_KP_VALUE, PID_KI_VALUE, PID_KD_VALUE);
    PID_SetOutputLimit(&g_laser_track_pid_x, PID_OUTPUT_MIN, PID_OUTPUT_MAX);
    PID_SetIntegralLimit(&g_laser_track_pid_x, PID_INTEGRAL_MIN, PID_INTEGRAL_MAX);
    
    // 初始化Y轴PID控制器
    PID_Init(&g_laser_track_pid_y, PID_TYPE_POSITIONAL);
    PID_SetParam(&g_laser_track_pid_y, PID_KP_VALUE, PID_KI_VALUE, PID_KD_VALUE);
    PID_SetOutputLimit(&g_laser_track_pid_y, PID_OUTPUT_MIN, PID_OUTPUT_MAX);
    PID_SetIntegralLimit(&g_laser_track_pid_y, PID_INTEGRAL_MIN, PID_INTEGRAL_MAX);
}

/**
 * @brief 设置激光追踪控制模式
 * @param mode 控制模式：TRACK_MODE_STEP(步进) 或 TRACK_MODE_PID(PID)
 */
void Laser_TrackAimPoint_SetMode(TrackMode_t mode)
{
    g_track_mode = mode;
    if (mode == TRACK_MODE_PID) {
        // 切换到PID模式时初始化PID控制器
        Laser_TrackPID_Init();
    }
}

/**
 * @brief 开始激光追踪任务
 */
void Laser_TrackAimPoint_Start(void)
{
    g_laser_track_running = true;

    // 初始化PID控制器（如果使用PID模式）
    if (g_track_mode == TRACK_MODE_PID) {
        Laser_TrackPID_Init();
    }

    // 立即打开激光指示器
    HAL_GPIO_WritePin(OUTPUT_TEST_GPIO_Port, OUTPUT_TEST_Pin, GPIO_PIN_SET);
}

/**
 * @brief 停止激光追踪任务
 */
void Laser_TrackAimPoint_Stop(void)
{
    g_laser_track_running = false;

    // 重置PID控制器（如果使用PID模式）
    if (g_track_mode == TRACK_MODE_PID) {
        PID_Reset(&g_laser_track_pid_x);
        PID_Reset(&g_laser_track_pid_y);
    }

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
 * @brief 步进控制方式的运动追踪（原始方式）
 * @return true 在死区内（已对准），false 仍在调整中
 */
static bool Laser_Track_StepControl(void)
{
    const uint16_t vel = 10;      // 电机速度
    const uint8_t acc = 5;        // 电机加速度
    const uint16_t DEADZONE = 3;  // 死区范围

    // 计算误差
    int16_t error_x = g_curr_center_point.x - g_sensor_aim_x;
    int16_t error_y = g_curr_center_point.y - g_sensor_aim_y;

    // 如果误差在死区内，不做调整
    if (abs(error_x) < DEADZONE && abs(error_y) < DEADZONE) {
        return true;  // 已对准
    }

    // 根据误差大小选择步进值
    uint16_t step_x, step_y;

    // X轴步进值选择
    if (abs(error_x) < ERROR_THRESHOLD_SMALL) {
        step_x = LASER_CLK_STEP_SMALL;
    } else if (abs(error_x) < ERROR_THRESHOLD_MEDIUM) {
        step_x = LASER_CLK_STEP_MEDIUM;
    } else {
        step_x = LASER_CLK_STEP_LARGE;
    }

    // Y轴步进值选择
    if (abs(error_y) < ERROR_THRESHOLD_SMALL) {
        step_y = LASER_CLK_STEP_SMALL;
    } else if (abs(error_y) < ERROR_THRESHOLD_MEDIUM) {
        step_y = LASER_CLK_STEP_MEDIUM;
    } else {
        step_y = LASER_CLK_STEP_LARGE;
    }

    // 控制X轴电机
    if (error_x > 0) {
        Emm_V5_Pos_Control(STEP_MOTOR_X, DIR_CCW, vel, acc, step_x, false, false);
    } else if (error_x < 0) {
        Emm_V5_Pos_Control(STEP_MOTOR_X, DIR_CW, vel, acc, step_x, false, false);
    }

    HAL_Delay(20);  // 延时避免指令冲突

    // 控制Y轴电机
    if (error_y > 0) {
        Emm_V5_Pos_Control(STEP_MOTOR_Y, DIR_CCW, vel, acc, step_y, false, false);
    } else if (error_y < 0) {
        Emm_V5_Pos_Control(STEP_MOTOR_Y, DIR_CW, vel, acc, step_y, false, false);
    }

    return false;  // 仍在调整中
}

/**
 * @brief PID控制方式的运动追踪
 * @return true 在死区内（已对准），false 仍在调整中
 */
static bool Laser_Track_PIDControl(void)
{
    const uint16_t vel = PID_MOTOR_VELOCITY;      // 使用配置的电机速度
    const uint8_t acc = PID_MOTOR_ACCELERATION;   // 使用配置的电机加速度
    const uint16_t DEADZONE = PID_DEADZONE;       // 使用配置的死区

    // 计算误差（目标位置 - 当前位置）
    float error_x = (float)(g_sensor_aim_x - g_curr_center_point.x);
    float error_y = (float)(g_sensor_aim_y - g_curr_center_point.y);

    // 如果误差在死区内，不做调整
    if (abs(error_x) < DEADZONE && abs(error_y) < DEADZONE) {
        return true;  // 已对准
    }

    // 设置PID目标值为0（即消除误差）
    PID_SetTarget(&g_laser_track_pid_x, 0.0f);
    PID_SetTarget(&g_laser_track_pid_y, 0.0f);

    // 计算PID输出（步进数），将误差作为当前值输入
    float pid_output_x = PID_Compute(&g_laser_track_pid_x, -error_x);  // 负号使得输出方向正确
    float pid_output_y = PID_Compute(&g_laser_track_pid_y, -error_y);

    // 限制步进值范围
    uint16_t step_x = (uint16_t)abs(pid_output_x);
    uint16_t step_y = (uint16_t)abs(pid_output_y);

    if (step_x > 0 && step_x < PID_MIN_STEP)
        step_x = PID_MIN_STEP;  // 使用配置的最小步进
    if (step_y > 0 && step_y < PID_MIN_STEP)
        step_y = PID_MIN_STEP;

    if (step_x > PID_MAX_STEP)
        step_x = PID_MAX_STEP;  // 使用配置的最大步进限制
    if (step_y > PID_MAX_STEP)
        step_y = PID_MAX_STEP;

    // 控制X轴电机（根据误差方向直接判断）
    if (abs(error_x) > DEADZONE) {
        if (error_x > 0) {
            // 目标在右侧，需要右转
            Emm_V5_Pos_Control(STEP_MOTOR_X, DIR_CCW, vel, acc, step_x, false, false);
        } else {
            // 目标在左侧，需要左转
            Emm_V5_Pos_Control(STEP_MOTOR_X, DIR_CW, vel, acc, step_x, false, false);
        }
    }

    HAL_Delay(PID_MOTOR_DELAY_MS);  // 使用配置的延时

    // 控制Y轴电机（根据误差方向直接判断）
    if (abs(error_y) > DEADZONE) {
        if (error_y > 0) {
            // 目标在上方，需要上转
            Emm_V5_Pos_Control(STEP_MOTOR_Y, DIR_CCW, vel, acc, step_y, false, false);
        } else {
            // 目标在下方，需要下转
            Emm_V5_Pos_Control(STEP_MOTOR_Y, DIR_CW, vel, acc, step_y, false, false);
        }
    }

    return false;  // 仍在调整中
}

/**
 * @brief 激光追踪瞄准点功能实现
 * 区别于Q2：先打开激光，然后持续追踪目标点（无超时限制）
 * 支持两种控制模式：步进控制和PID控制
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

    bool is_aligned = false;

    // 根据控制模式选择相应的控制函数
    switch (g_track_mode) {
        case TRACK_MODE_STEP:
            is_aligned = Laser_Track_StepControl();
            break;

        case TRACK_MODE_PID:
            is_aligned = Laser_Track_PIDControl();
            break;

        default:
            // 默认使用步进控制
            is_aligned = Laser_Track_StepControl();
            break;
    }

    // 可以在这里添加对准状态的处理逻辑
    // 例如：记录对准时间、状态指示等
    (void)is_aligned;  // 暂时不使用，避免编译警告
}
