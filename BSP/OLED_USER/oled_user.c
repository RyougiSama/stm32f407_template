#include "oled_user.h"

#include "laser_shot_common.h"
#include "Emm_V5.h"

OLED_Disp_t g_oled_mode = DISP_CENTER_POINT;
uint8_t current_task = 1;
uint8_t current_set_zero_addr = STEP_MOTOR_X;

void OLED_ChangeMode(void)
{
    switch (g_oled_mode) {
        case DISP_CENTER_POINT:
            g_oled_mode = SET_ZERO_POINT;
            break;
        case SET_ZERO_POINT:
            g_oled_mode = TEST_MODE;
            break;
        case TEST_MODE:
            g_oled_mode = DISP_CENTER_POINT;
            break;
    }
}

void OLED_ChangeTask(void)
{
    if (current_task < 4) {
        current_task++;
    } else {
        current_task = 1;
    }
}

void OLED_Display(void)
{
    OLED_Clear();
    switch (g_oled_mode) {
        case DISP_CENTER_POINT:
            OLED_ShowString(0, 0, "Center Point", 16);
            break;
        case SET_ZERO_POINT:
            OLED_ShowString(0, 0, "SET_ZERO", 16);
            break;
        case TEST_MODE:
            OLED_ShowString(0, 0, "K0:Choose K1:Run", 8);
    }
    if (g_oled_mode == TEST_MODE) {
        OLED_ShowString(0, 1, "Run Task1", 8);
        OLED_ShowString(0, 2, "Run Task2", 8);
        OLED_ShowString(0, 3, "Run Task3", 8);
        OLED_ShowString(0, 4, "Run Task4", 8);

        OLED_ShowString(6 * 18, current_task, "<--", 8);
        OLED_ShowString(0, 7, "Laser:", 8);
    } else if (g_oled_mode == DISP_CENTER_POINT) {
        // 显示中心点坐标
        OLED_ShowString(0, 2, "Center X:", 16);
        OLED_ShowString(0, 4, "Center Y:", 16);
        OLED_ShowNum(8 * 12, 2, g_curr_center_point.x, 4, 16);
        OLED_ShowNum(8 * 12, 4, g_curr_center_point.y, 4, 16);
        
        // 显示激光追踪状态
        OLED_ShowString(0, 6, "Laser Track:", 8);
        if (Laser_TrackAimPoint_IsRunning()) {
            OLED_ShowString(8 * 12, 6, "ON", 8);
        } else {
            OLED_ShowString(8 * 12, 6, "OFF", 8);
        }
    } else if (g_oled_mode == SET_ZERO_POINT) {
        OLED_ShowString(0, 2, "Mode:", 16);
        if (current_set_zero_addr == STEP_MOTOR_X) {
            OLED_ShowString(6 * 12, 2, "X", 16);
        } else {
            OLED_ShowString(6 * 12, 2, "Y", 16);
        }
    }
}
