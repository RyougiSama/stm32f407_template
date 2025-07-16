#include "oled_user.h"
#include "oled_hardware_spi.h"

#include "servo_user.h"

OLED_Disp_t g_oled_mode = CH_X_10;

void OLED_ChangeMode(void)
{
    switch (g_oled_mode) {
        case CH_X_10:
            g_oled_mode = CH_X_50;
            break;
        case CH_X_50:
            g_oled_mode = CH_Y_10;
            break;
        case CH_Y_10:
            g_oled_mode = CH_Y_50;
            break;
        case CH_Y_50:
            g_oled_mode = CH_X_10;
            break;
    }
}

void OLED_Display(void)
{
    switch (g_oled_mode) {
        case CH_X_10:
            OLED_ShowString(0, 0, "CH_X 10 Mode", 16);
            break;
        case CH_X_50:
            OLED_ShowString(0, 0, "CH_X 50 Mode", 16);
            break;
        case CH_Y_10:
            OLED_ShowString(0, 0, "CH_Y 10 Mode", 16);
            break;
        case CH_Y_50:
            OLED_ShowString(0, 0, "CH_Y 50 Mode", 16);
            break;
    }
    OLED_ShowNum(8*12, 2, g_servox_duty, 4, 16);
    OLED_ShowNum(8*12, 5, g_servoy_duty, 4, 16);
}
