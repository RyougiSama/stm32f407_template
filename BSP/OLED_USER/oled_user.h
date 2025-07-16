#ifndef __OLED_USER_H
#define __OLED_USER_H

typedef enum {
    CH_X_10,
    CH_X_50,
    CH_Y_10,
    CH_Y_50,
} OLED_Disp_t;

extern OLED_Disp_t g_oled_mode;

void OLED_ChangeMode(void);
void OLED_Display(void);

#endif
