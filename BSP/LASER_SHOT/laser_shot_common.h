#ifndef __LASER_SHOT_COMMON_H
#define __LASER_SHOT_COMMON_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#define SENSOR_WIDTH  640
#define SENSOR_HEIGHT 480
#define SENSOR_AIM_X 320
#define SENSOR_AIM_Y 240


typedef struct {
    uint16_t x;  // X坐标
    uint16_t y;  // Y坐标
} PixelPoint_t;

extern PixelPoint_t g_curr_center_point;

void Task_BasicQ2_Start(void);
void Task_BasicQ2_Excute(void);

#endif
