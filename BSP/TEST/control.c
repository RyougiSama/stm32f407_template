#include "control.h"
#include "servo_user.h"
#include "math.h"

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
