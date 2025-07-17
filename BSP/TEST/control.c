#include "control.h"
#include "servo_user.h"
#include "math.h"
#include "pid_controller.h"

static const uint16_t reset_x = SERVO_VERTICAL_X_DUTY; // X轴复位占空比
static const uint16_t reset_y = SERVO_VERTICAL_Y_DUTY; // Y轴复位占空比

static const uint16_t T2_roll_1 = 1580;  // T2任务滚转角度1
static const uint16_t T2_roll_2 = 1390;  // T2任务滚转角度2
static const uint16_t T2_pitch_1 = 1410; // T2任务俯仰角度1
static const uint16_t T2_pitch_2 = 1690; // T2任务俯仰角度2

/**
 * @brief  reset to center
 * @param  None
 *
 * @retval None
 */
void Task1_Reset_To_Ctr(void)
{
    g_servox_duty = reset_x;
    g_servoy_duty = reset_y;
    Servo_SetPulseWidth_DirX(reset_x); // Reset X-axis to center
    Servo_SetPulseWidth_DirY(reset_y); // Reset Y-axis to center
    HAL_Delay(500);                 // Wait for servo to stabilize
}

/**
 * @brief  run task 2
 * @param  None
 *
 * @retval None
 */
void Task2_Run(void)
{
    const uint16_t step_delay = 200, process_delay = 1000;
    uint16_t cnt = 0;
    /*start*/
    Servo_SetPulseWidth_DirY(T2_pitch_1); // set X-axis to first position
    Servo_SetPulseWidth_DirX(T2_roll_1);  // set Y-axis to first position
    HAL_Delay(process_delay);
    /*roll_1 -> roll_2*/
    while (T2_roll_1 - cnt > T2_roll_2) {
        cnt += 20;
        Servo_SetPulseWidth_DirY(T2_pitch_1);      // Reset X-axis to center
        Servo_SetPulseWidth_DirX(T2_roll_1 - cnt); // Reset Y-axis to center
        HAL_Delay(step_delay);
    }
    cnt = 0;
    /*pitch_1 -> pitch_2*/
    HAL_Delay(process_delay);
    while (T2_pitch_1 + cnt < T2_pitch_2) {
        cnt += 20;
        Servo_SetPulseWidth_DirY(T2_pitch_1 + cnt); // Reset X-axis to center
        Servo_SetPulseWidth_DirX(T2_roll_2);        // Reset Y-axis to center
        HAL_Delay(step_delay);
    }
    cnt = 0;
    /*roll_2 -> roll_1*/
    HAL_Delay(100);
    while (T2_roll_2 + cnt < T2_roll_1) {
        cnt += 20;
        Servo_SetPulseWidth_DirY(T2_pitch_2);      // Reset X-axis to center
        Servo_SetPulseWidth_DirX(T2_roll_2 + cnt); // Reset Y-axis to center
        HAL_Delay(step_delay);
    }
    cnt = 0;
    /*pitch_2 -> pitch_1*/
    HAL_Delay(process_delay);
    while (T2_pitch_2 - cnt > T2_pitch_1) {
        cnt += 20;
        Servo_SetPulseWidth_DirY(T2_pitch_2 - cnt); // Reset X-axis to center
        Servo_SetPulseWidth_DirX(T2_roll_1);        // Reset Y-axis to center
        HAL_Delay(step_delay);
    }
    HAL_Delay(process_delay);
    Task1_Reset_To_Ctr();
}


uint8_t g_laser_point_x, g_laser_point_y; // Laser point coordinates
void Task3_Run(void)
{
    const uint8_t laser_servo_step = 1;
    const uint8_t target_x = 27, target_y = 16;
    const uint8_t pos_error = 0; // 定义位置误差允许范围
    const uint16_t step_delay = 30;
    uint8_t x_done = 0, y_done = 0;
    // 继续循环直到 X 轴和 Y 轴都到达目标位置
    while (!x_done || !y_done) {
        // 处理 X 轴一步
        if (!x_done) {
            if (g_laser_point_x == 0) {
                // 如果没有有效数据则跳过本次 X 轴调整
            } else if (fabs(g_laser_point_x - target_x) <= pos_error) {
                x_done = 1; // X 轴到达目标位置
            } else if (g_laser_point_x < target_x) {
                g_servox_duty -= laser_servo_step;
                if (g_servox_duty < SERVO_PWM_MIN) g_servox_duty = SERVO_PWM_MIN;
                Servo_SetPulseWidth_DirX(g_servox_duty);
            } else {
                g_servox_duty += laser_servo_step;
                if (g_servox_duty > SERVO_PWM_MAX) g_servox_duty = SERVO_PWM_MAX;
                Servo_SetPulseWidth_DirX(g_servox_duty);
            }
            HAL_Delay(step_delay);
        }
        // 处理 Y 轴一步
        if (!y_done) {
            if (g_laser_point_y == 0) {
                // 如果没有有效数据则跳过本次 Y 轴调整
            } else if (fabs(g_laser_point_y - target_y) <= pos_error) {
                y_done = 1; // Y 轴到达目标位置
            } else if (g_laser_point_y < target_y) {
                g_servoy_duty += laser_servo_step;
                if (g_servoy_duty > SERVO_PWM_MAX) g_servoy_duty = SERVO_PWM_MAX;
                Servo_SetPulseWidth_DirY(g_servoy_duty);
            } else {
                g_servoy_duty -= laser_servo_step;
                if (g_servoy_duty < SERVO_PWM_MIN) g_servoy_duty = SERVO_PWM_MIN;
                Servo_SetPulseWidth_DirY(g_servoy_duty);
            }
            HAL_Delay(step_delay);
        }
    }
}

/**
 * @brief 使用PID控制器实现激光点追踪
 * @note 与Task3_Run功能相同，但使用PID控制以提高精度和稳定性
 */
void Task4_Run(void)
{
    // PID控制器定义
    PidController_t pid_x, pid_y;
    const uint8_t target_x = 27, target_y = 16;
    const uint8_t pos_error = 2; // 增加误差允许范围，避免无法达到完美精度
    const uint16_t step_delay = 100; // 增加延时，给系统更多响应时间
    
    // 初始化X轴PID控制器 - 调整参数以适应系统特性
    PID_Init(&pid_x, PID_TYPE_POSITIONAL);
    PID_SetParam(&pid_x, 0.8f, 0.01f, 0.4f); // 降低P值，增加D值以减少振荡
    PID_SetOutputLimit(&pid_x, -3.0f, 3.0f); // 缩小输出范围，减少调整幅度
    PID_SetIntegralLimit(&pid_x, -20.0f, 20.0f);
    PID_SetTarget(&pid_x, target_x);
    
    // 初始化Y轴PID控制器 - 可能与X轴特性不同，单独调参
    PID_Init(&pid_y, PID_TYPE_POSITIONAL);
    PID_SetParam(&pid_y, 0.8f, 0.01f, 0.4f); // 降低P值，增加D值以减少振荡
    PID_SetOutputLimit(&pid_y, -3.0f, 3.0f); // 缩小输出范围，减少调整幅度
    PID_SetIntegralLimit(&pid_y, -20.0f, 20.0f);
    PID_SetTarget(&pid_y, target_y);
    // 继续循环直到 X 轴和 Y 轴都到达目标位置
    uint8_t stable_count = 0;
    const uint8_t required_stable_count = 5; // 降低稳定次数要求，避免难以达到稳定条件
    uint8_t x_fine_tuning = 0, y_fine_tuning = 0; // 记录微调模式
    float prev_x_error = 0, prev_y_error = 0; // 记录前一次误差，用于判断趋势
    
    // 添加超时机制
    uint32_t start_time = HAL_GetTick();
    const uint32_t max_runtime = 10000; // 10秒最大运行时间
    
    while (stable_count < required_stable_count) {
        // 检查是否超时
        if (HAL_GetTick() - start_time > max_runtime) {
            break; // 超时退出
        }
        float x_output = 0, y_output = 0;
        bool is_stable = true;
        
        // 处理 X 轴
        if (g_laser_point_x != 0) {
            float x_error = target_x - g_laser_point_x;
            
            // 检查是否接近目标，切换到微调模式
            if (fabs(x_error) <= pos_error + 2) {
                x_fine_tuning = 1;
            }
            
            // 计算PID输出值
            x_output = PID_Compute(&pid_x, g_laser_point_x);
            
            // 检查是否到达目标位置
            if (fabs(x_error) > pos_error) {
                is_stable = false;
                
                // 微调模式下使用更小的步长
                if (x_fine_tuning) {
                    // 检查误差趋势，防止振荡
                    if ((x_error > 0 && prev_x_error < 0) || (x_error < 0 && prev_x_error > 0)) {
                        x_output *= 0.5f; // 减小输出，防止过冲
                    }
                    
                    // 对于小误差使用固定微小步长
                    if (fabs(x_error) <= 3) {
                        x_output = (x_error > 0) ? 0.5f : -0.5f;
                    }
                }
                
                // 反向计算，因为舵机控制与坐标系的方向关系
                g_servox_duty += (int16_t)(-x_output);
                
                // 限制PWM范围
                if (g_servox_duty > SERVO_PWM_MAX) g_servox_duty = SERVO_PWM_MAX;
                if (g_servox_duty < SERVO_PWM_MIN) g_servox_duty = SERVO_PWM_MIN;
                
                // 设置舵机位置
                Servo_SetPulseWidth_DirX(g_servox_duty);
            }
            
            prev_x_error = x_error;
        }
        
        // 处理 Y 轴
        if (g_laser_point_y != 0) {
            float y_error = target_y - g_laser_point_y;
            
            // 检查是否接近目标，切换到微调模式
            if (fabs(y_error) <= pos_error + 2) {
                y_fine_tuning = 1;
            }
            
            // 计算PID输出值
            y_output = PID_Compute(&pid_y, g_laser_point_y);
            
            // 检查是否到达目标位置
            if (fabs(y_error) > pos_error) {
                is_stable = false;
                
                // 微调模式下使用更小的步长
                if (y_fine_tuning) {
                    // 检查误差趋势，防止振荡
                    if ((y_error > 0 && prev_y_error < 0) || (y_error < 0 && prev_y_error > 0)) {
                        y_output *= 0.5f; // 减小输出，防止过冲
                    }
                    
                    // 对于小误差使用固定微小步长
                    if (fabs(y_error) <= 3) {
                        y_output = (y_error > 0) ? 0.5f : -0.5f;
                    }
                }
                
                // Y轴与X轴方向相反
                g_servoy_duty += (int16_t)(y_output);
                
                // 限制PWM范围
                if (g_servoy_duty > SERVO_PWM_MAX) g_servoy_duty = SERVO_PWM_MAX;
                if (g_servoy_duty < SERVO_PWM_MIN) g_servoy_duty = SERVO_PWM_MIN;
                
                // 设置舵机位置
                Servo_SetPulseWidth_DirY(g_servoy_duty);
            }
            
            prev_y_error = y_error;
        }
        
        // 检查稳定性
        if (is_stable && g_laser_point_x != 0 && g_laser_point_y != 0) {
            stable_count++;
        } else {
            stable_count = 0;
        }
        
        HAL_Delay(step_delay);
    }
}
