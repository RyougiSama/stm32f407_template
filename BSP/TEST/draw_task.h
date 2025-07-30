#ifndef __DRAW_TASK_H
#define __DRAW_TASK_H

#include "main.h"

void Task_Draw(void);
void DrawLine(float start_x, float start_y, float end_x, float end_y, 
              float distance, uint8_t num_steps, uint16_t step_delay);
void MoveToPoint(float target_x, float target_y, float distance, 
                 uint8_t num_steps, uint16_t step_delay);
void Task_DrawShape(void);

#endif
