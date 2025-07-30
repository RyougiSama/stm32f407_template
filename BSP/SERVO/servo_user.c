#include "servo_user.h"

#include "main.h"
#include <stdlib.h>
#include <stdbool.h>

uint16_t g_servox_duty = SERVO_PWM_ANGLE_ZERO;
uint16_t g_servoy_duty = SERVO_PWM_ANGLE_ZERO;

float g_vertical_distance = 100.0f;  // cm

void Servo_Return_To_Zero(void)
{
    const uint16_t target_x = SERVO_VERTICAL_X_DUTY;  // 目标X轴位置
    const uint16_t target_y = SERVO_VERTICAL_Y_DUTY;  // 目标Y轴位置
    const uint16_t step_size = 1;                     // 每步移动的PWM值大小
    const uint16_t step_delay = 20;                   // 每步之间的延时(ms)
    const uint8_t tolerance = 1;                      // 位置容差

    // 平滑移动到目标位置
    while (1) {
        bool x_reached = false, y_reached = false;

        // 处理X轴平滑移动
        if (abs((int16_t)g_servox_duty - (int16_t)target_x) <= tolerance) {
            x_reached = true;
        } else {
            if (g_servox_duty > target_x) {
                // 当前位置大于目标位置，减小PWM值
                if (g_servox_duty - target_x > step_size) {
                    g_servox_duty -= step_size;
                } else {
                    g_servox_duty = target_x;
                }
            } else {
                // 当前位置小于目标位置，增大PWM值
                if (target_x - g_servox_duty > step_size) {
                    g_servox_duty += step_size;
                } else {
                    g_servox_duty = target_x;
                }
            }
            Servo_SetPulseWidth_DirX(g_servox_duty);
        }

        // 处理Y轴平滑移动
        if (abs((int16_t)g_servoy_duty - (int16_t)target_y) <= tolerance) {
            y_reached = true;
        } else {
            if (g_servoy_duty > target_y) {
                // 当前位置大于目标位置，减小PWM值
                if (g_servoy_duty - target_y > step_size) {
                    g_servoy_duty -= step_size;
                } else {
                    g_servoy_duty = target_y;
                }
            } else {
                // 当前位置小于目标位置，增大PWM值
                if (target_y - g_servoy_duty > step_size) {
                    g_servoy_duty += step_size;
                } else {
                    g_servoy_duty = target_y;
                }
            }
            Servo_SetPulseWidth_DirY(g_servoy_duty);
        }

        // 检查是否都到达目标位置
        if (x_reached && y_reached) {
            break;  // 两个轴都到达目标位置，退出循环
        }

        HAL_Delay(step_delay);  // 等待一段时间再进行下一步
    }
}

/**
 * @brief 计算X轴偏转角度
 * @param dist 距离 (cm)
 * @param x_offset X轴偏移量 (cm)
 * @return float 偏转角度 -90°到90°之间
 */
float Calc_DeflectionAngle_DirX(float dist, float x_offset)
{
    // 计算X轴偏转角度
    float angle = atan(x_offset / dist) * 180.0f / PI;  // 弧度转角度
    if (angle < -90.0f) {
        angle = -90.0f;
    } else if (angle > 90.0f) {
        angle = 90.0f;
    }
    return angle;
}

/**
 * @brief 计算Y轴偏转角度
 *
 * @param dist 距离 (cm)
 * @param y_offset Y轴偏移量 (cm)
 * @return float 偏转角度 -90°到90°之间
 */
float Calc_DeflectionAngle_DirY(float dist, float y_offset)
{
    // 计算Y轴偏转角度
    float angle = atan(y_offset / dist) * 180.0f / PI;  // 弧度转角度
    if (angle < -90.0f) {
        angle = -90.0f;
    } else if (angle > 90.0f) {
        angle = 90.0f;
    }
    return angle;
}
