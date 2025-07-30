#include "oled_user.h"

#include "control.h"
#include "laser_shot_common.h"
#include "servo_user.h"

OLED_Disp_t g_oled_mode = DISP_CENTER_POINT;
uint8_t current_task = 1;

void OLED_ChangeMode(void)
{
    switch (g_oled_mode) {
        case DISP_CENTER_POINT:
            g_oled_mode = CH_Y_10;
            break;
        case CH_Y_10:
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
        case CH_Y_10:
            OLED_ShowString(0, 0, "CH_Y Mode", 16);
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
    } else {
        OLED_ShowString(0, 2, "CH_X Duty:", 16);
        OLED_ShowString(0, 4, "CH_Y Duty:", 16);
        OLED_ShowNum(8 * 12, 2, g_servox_duty, 4, 16);
        OLED_ShowNum(8 * 12, 4, g_servoy_duty, 4, 16);
        OLED_ShowString(0, 7, "Laser:", 8);
        OLED_ShowNum(6 * 12, 7, g_laser_point.x, 3, 8);
        OLED_ShowNum(9 * 12, 7, g_laser_point.y, 3, 8);
    }
}
