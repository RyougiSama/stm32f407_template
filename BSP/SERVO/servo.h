/**
 * @file servo.h
 * @author Shiki
 * @brief 舵机控制头文件
 *        设置TIM3的PWM输出到PA6和PA7，预分频为84-1，计数器周期为20000-1
 * @version 0.1
 * @date 2025-07-14
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef __SERVO_H
#define __SERVO_H

#include <stdint.h>

#define SERVO_PWM_MIN           500  // 舵机的最小占空比
#define SERVO_PWM_ANGLE_ZERO    1500 // 舵机的0度占空比
#define SERVO_PWM_MAX           2500 // 舵机的最大占空比
#define SERVO_PWM_MIN_STEP      3    // 舵机最小步进占空比

typedef enum {
    SERVO_CH_X = 1, // 通道1对应PA6
    SERVO_CH_Y = 2  // 通道2对应PA7
} ServoChannel_t;

void Servo_Init(void);
void Servo_SetAngle_DirY(int16_t angle);
void Servo_SetPulseWidth_DirY(uint16_t pulse_width);
void Servo_SetAngle_DirX(int16_t angle);
void Servo_SetPulseWidth_DirX(uint16_t pulse_width);

#endif