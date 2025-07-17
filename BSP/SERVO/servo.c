#include "servo.h"
#include "tim.h"

/**
 * @brief 舵机初始化
 * 
 */
void Servo_Init(void)
{
    // 启动TIM3-CH1-PWM -PA6
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    // 启动TIM3-CH1-PWM -PA7
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}

/**
 * @brief 设置舵机PWM占空比
 * 
 * @param servo_channel 舵机通道
 * @param compare 舵机PWM占空比
 */
static void Servo_SetCompare(uint16_t servo_channel, uint16_t compare)
{
    __HAL_TIM_SET_COMPARE(&htim3, servo_channel, compare);
}

/**
 * @brief 设置通道1舵机角度
 * 
 * @param angle 舵机角度 -90°到90°之间
 */
void Servo_SetAngle_DirY(float angle)
{
    if (angle < -90 || angle > 90)
        return;

    uint16_t pulse_width = (uint16_t)(SERVO_PWM_MIN + (SERVO_PWM_MAX - SERVO_PWM_MIN) * (angle + 90.0f) / 180.0f);
    Servo_SetCompare(TIM_CHANNEL_1, pulse_width);
}

/**
 * @brief 设置通道1舵机占空比
 * 
 * @param pulse_width 500到2500之间
 */
void Servo_SetPulseWidth_DirY(uint16_t pulse_width)
{
    if (pulse_width < SERVO_PWM_MIN || pulse_width > SERVO_PWM_MAX)
        return;

    Servo_SetCompare(TIM_CHANNEL_1, pulse_width);
}

/**
 * @brief 设置通道2舵机角度
 * 
 * @param angle 舵机角度 -135°到135°之间
 */
void Servo_SetAngle_DirX(float angle)
{
    if (angle < -135 || angle > 135)
        return;

    uint16_t pulse_width = (uint16_t)(SERVO_PWM_MIN + (SERVO_PWM_MAX - SERVO_PWM_MIN) * (angle + 135.0f) / 270.0f);
    Servo_SetCompare(TIM_CHANNEL_2, pulse_width);
}

/**
 * @brief 设置通道2舵机占空比
 * 
 * @param pulse_width 500到2500之间
 */
void Servo_SetPulseWidth_DirX(uint16_t pulse_width)
{
    if (pulse_width < SERVO_PWM_MIN || pulse_width > SERVO_PWM_MAX)
        return;

    Servo_SetCompare(TIM_CHANNEL_2, pulse_width);
}
