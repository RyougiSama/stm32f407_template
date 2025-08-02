#include "Emm_V5.h"
#include "gpio.h"
#include "laser_shot_common.h"
#include "pid_controller.h"
#include "task_scheduler.h"

// 激光追踪任务相关变量
static bool g_laser_track_running = false;
static TrackMode_t g_track_mode = TRACK_MODE_PID;  // 默认使用步进控制

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
    PID_SetParam(&g_laser_track_pid_x, 0.5f, 0.01f, 0.1f);      // Kp=0.5, Ki=0.01, Kd=0.1
    PID_SetOutputLimit(&g_laser_track_pid_x, -50.0f, 50.0f);    // 限制输出步进数
    PID_SetIntegralLimit(&g_laser_track_pid_x, -20.0f, 20.0f);  // 积分限幅

    // 初始化Y轴PID控制器
    PID_Init(&g_laser_track_pid_y, PID_TYPE_POSITIONAL);
    PID_SetParam(&g_laser_track_pid_y, 0.5f, 0.01f, 0.1f);      // Kp=0.5, Ki=0.01, Kd=0.1
    PID_SetOutputLimit(&g_laser_track_pid_y, -50.0f, 50.0f);    // 限制输出步进数
    PID_SetIntegralLimit(&g_laser_track_pid_y, -20.0f, 20.0f);  // 积分限幅
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
    const uint16_t vel = 15;      // PID模式使用稍高速度
    const uint8_t acc = 8;        // PID模式使用稍高加速度
    const uint16_t DEADZONE = 2;  // PID模式使用更小死区，精度更高

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

    // 限制最小步进值，避免过小的调整
    uint16_t step_x = (uint16_t)abs(pid_output_x);
    uint16_t step_y = (uint16_t)abs(pid_output_y);

    if (step_x > 0 && step_x < 2)
        step_x = 2;  // 最小步进
    if (step_y > 0 && step_y < 2)
        step_y = 2;

    if (step_x > 20)
        step_x = 20;  // 最大步进限制
    if (step_y > 20)
        step_y = 20;

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

    HAL_Delay(20);  // 延时避免指令冲突

    // 控制Y轴电机（根据误差方向直接判断）
    if (abs(error_y) > DEADZONE) {
        if (error_y > 0) {
            // 目标在上方，需要上转
            Emm_V5_Pos_Control(STEP_MOTOR_Y, DIR_CW, vel, acc, step_y, false, false);
        } else {
            // 目标在下方，需要下转
            Emm_V5_Pos_Control(STEP_MOTOR_Y, DIR_CCW, vel, acc, step_y, false, false);
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
