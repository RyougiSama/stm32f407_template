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

void Servo_Init(void);
void Servo_SetAngle_CH1(int16_t angle);
void Servo_SetAngle_CH2(int16_t angle);

#endif