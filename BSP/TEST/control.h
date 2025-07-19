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

/**
 * @brief 激光点坐标结构体
 */
typedef struct {
    uint8_t x;  // X坐标
    uint8_t y;  // Y坐标
} LaserPoint_t;

extern LaserPoint_t g_laser_point;
extern LaserPoint_t g_lu_corner, g_ru_corner, g_ld_corner, g_rd_corner; // Laser corners
extern uint8_t g_is_corner_init;

void Task1_Reset_To_Ctr(void);
void Task2_Run(void);
void Task3_Run(void);
void Task4_Run(void);
void TrackPixelPoint(LaserPoint_t target_point);
void MoveToPointStraight(LaserPoint_t target_point);
void MoveAlongEdge(LaserPoint_t target_point, uint8_t change_x, uint8_t change_y, float pixel_ratio);

// 新增的矩形边遍历相关函数
void TraverseRectangleEdge(LaserPoint_t start_point, LaserPoint_t end_point, uint8_t num_points, uint16_t point_delay);
void TrackPixelPointFast(LaserPoint_t target_point);
float CalculateDistance(LaserPoint_t p1, LaserPoint_t p2);
void TraverseRectangleEdgeAdaptive(LaserPoint_t start_point, LaserPoint_t end_point, float points_per_unit, uint16_t point_delay);
void DebugTraverseRectangleEdge(LaserPoint_t start_point, LaserPoint_t end_point, uint8_t num_points);

#endif
