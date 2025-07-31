#ifndef __LASER_SHOT_COMMON_H
#define __LASER_SHOT_COMMON_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

extern uint16_t g_sensor_width;
extern uint16_t g_sensor_height;
extern uint16_t g_sensor_aim_x;
extern uint16_t g_sensor_aim_y;


typedef struct {
    uint16_t x;  // X坐标
    uint16_t y;  // Y坐标
} PixelPoint_t;

extern PixelPoint_t g_curr_center_point;

void Task_BasicQ2_Start(void);
void Task_BasicQ2_Excute(void);

#endif
