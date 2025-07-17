/**
 * @file control.h
 * @author Xuyi
 * @brief 
 * @version 0.1
 * @date 2025-07-13
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef __CONTROL_H
#define __CONTROL_H

#include "main.h"

extern uint8_t g_laser_point_x, g_laser_point_y;

void Task1_Reset_To_Ctr(void);
void Task2_Run(void);
void Task3_Run(void);

#endif
