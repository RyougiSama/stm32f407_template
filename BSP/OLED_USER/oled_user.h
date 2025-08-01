#ifndef __OLED_USER_H
#define __OLED_USER_H

#include "stdint.h"
#include "oled.h"

typedef enum {
    DISP_CENTER_POINT,
    SET_ZERO_POINT,
    TEST_MODE
} OLED_Disp_t;

extern OLED_Disp_t g_oled_mode;
extern uint8_t current_task;
extern uint8_t current_set_zero_addr;

void OLED_ChangeMode(void);
void OLED_ChangeTask(void);
void OLED_Display(void);

#endif
