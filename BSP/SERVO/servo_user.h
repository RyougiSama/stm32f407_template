#ifndef __SERVO_USER_H
#define __SERVO_USER_H

#include "servo.h"

#define SERVO_VERTICAL_X_DUTY 1611      // X轴舵机零点角度
#define SERVO_VERTICAL_Y_DUTY 1500      // Y轴舵机零点角度


extern uint16_t g_servox_duty;  // X轴舵机占空比
extern uint16_t g_servoy_duty;  // Y轴舵机占空比

#endif
