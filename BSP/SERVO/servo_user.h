#ifndef __SERVO_USER_H
#define __SERVO_USER_H

#include "servo.h"

#include <math.h>

#define SERVO_VERTICAL_X_DUTY 1500      // X轴舵机零点角度
#define SERVO_VERTICAL_Y_DUTY 1500      // Y轴舵机零点角度

#define PI 3.14159265358979323846

extern uint16_t g_servox_duty;  // X轴舵机占空比
extern uint16_t g_servoy_duty;  // Y轴舵机占空比

extern float g_vertical_distance;  // cm

void Servo_Return_To_Zero(void);
float Calc_DeflectionAngle_DirX(float dist, float x_offset);
float Calc_DeflectionAngle_DirY(float dist, float y_offset);

#endif
