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
    const uint8_t laser_servo_step = 3;
    const uint8_t target_x = 12, target_y = 5;
    const uint16_t step_delay = 100;

    while (fabs(g_laser_point_x - target_x) > 2) {
        if (g_laser_point_x == 0) continue;
        if (g_laser_point_x < target_x) {
            g_servox_duty -= laser_servo_step;
            if (g_servox_duty < SERVO_PWM_MIN) g_servox_duty = SERVO_PWM_MIN;
        } else {
            g_servox_duty += laser_servo_step;
            if (g_servox_duty > SERVO_PWM_MAX) g_servox_duty = SERVO_PWM_MAX;
        }
        Servo_SetPulseWidth_DirX(g_servox_duty);
        HAL_Delay(step_delay);
    }
    while (fabs(g_laser_point_y - target_y) > 2) {
        if (g_laser_point_y == 0) continue;
        if (g_laser_point_y < target_y) {
            g_servoy_duty += laser_servo_step;
            if (g_servoy_duty > SERVO_PWM_MAX) g_servoy_duty = SERVO_PWM_MAX;
        } else {
            g_servoy_duty -= laser_servo_step;
            if (g_servoy_duty < SERVO_PWM_MIN) g_servoy_duty = SERVO_PWM_MIN;
        }
        Servo_SetPulseWidth_DirY(g_servoy_duty);
        HAL_Delay(step_delay);
    }
}
