#include "key.h"
#include "task_scheduler.h"
#include "servo_user.h"
#include "oled_user.h"

#include "control.h"

/* 按键消抖实例 */
static KeyDebounce_t key_debounce;

static KeyValue_t Key_GetNum(void)
{
    KeyValue_t key_num = KEY_NONE;
    // Check Key 0 (PE4)
    if (HAL_GPIO_ReadPin(GPIOE, KEY_0_Pin) == GPIO_PIN_RESET) {
        key_num = KEY_0;
    }
    // Check Key 1 (PE3)
    if (HAL_GPIO_ReadPin(GPIOE, KEY_1_Pin) == GPIO_PIN_RESET) {
        key_num = KEY_1;
    }
    if (HAL_GPIO_ReadPin(USER_KEY_GPIO_Port, USER_KEY_Pin) == GPIO_PIN_SET) {
        key_num = USER_KEY;
    }
    return key_num;
}

/**
 * @brief 获取经过消抖处理的按键值
 * 
 * @return KeyValue_t 消抖后的按键值，KEY_NONE表示无按键，其他值表示有按键按下
 */
KeyValue_t Key_GetDebounced(void)
{
    uint32_t current_tick = TaskScheduler_GetSystemTick();
    KeyValue_t raw_key = Key_GetNum();
    switch (key_debounce.state) {
        case KEY_STATE_IDLE:
            if (raw_key != KEY_NONE) {
                key_debounce.current_val = raw_key;
                key_debounce.debounce_timer = current_tick;
                key_debounce.state = KEY_STATE_PRESSED;
            }
            break;
        case KEY_STATE_PRESSED:
            if (raw_key == key_debounce.current_val) {
                /* 按键值稳定，检查是否达到消抖时间 */
                if (current_tick - key_debounce.debounce_timer >= KEY_DEBOUNCE_TIME) {
                    key_debounce.stable_val = key_debounce.current_val;
                    key_debounce.key_pressed = 1;
                    key_debounce.state = KEY_STATE_CONFIRMED;
                    return key_debounce.stable_val;
                }
            } else {
                /* 按键值变化，重新开始消抖 */
                if (raw_key == KEY_NONE) {
                    key_debounce.state = KEY_STATE_IDLE;
                } else {
                    key_debounce.current_val = raw_key;
                    key_debounce.debounce_timer = current_tick;
                }
            }
            break;
        case KEY_STATE_CONFIRMED:
            if (raw_key == KEY_NONE) {
                /* 按键释放 */
                key_debounce.debounce_timer = current_tick;
                key_debounce.state = KEY_STATE_RELEASED;
            }
            break;
        case KEY_STATE_RELEASED:
            if (raw_key == KEY_NONE) {
                /* 确认按键释放 */
                if (current_tick - key_debounce.debounce_timer >= KEY_DEBOUNCE_TIME) {
                    key_debounce.key_pressed = 0;
                    key_debounce.stable_val = KEY_NONE;
                    key_debounce.state = KEY_STATE_IDLE;
                }
            } else {
                /* 按键还在按下状态 */
                key_debounce.current_val = raw_key;
                key_debounce.debounce_timer = current_tick;
                key_debounce.state = KEY_STATE_PRESSED;
            }
            break;
    }
    return KEY_NONE;  /* 无稳定按键 */
}

#if 0
uint8_t Matrix_Key_Scan(void)
{
    uint8_t current_raw_key = 0;

    // --- Scan Row 1 ---
    HAL_GPIO_WritePin(ROW1_PORT, ROW1_PIN, GPIO_PIN_RESET); // Pull Row 1 LOW
    HAL_GPIO_WritePin(ROW2_PORT, ROW2_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ROW3_PORT, ROW3_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ROW4_PORT, ROW4_PIN, GPIO_PIN_SET);

    if (HAL_GPIO_ReadPin(COL1_PORT, COL1_PIN) == GPIO_PIN_RESET) {
        current_raw_key = 1;
    } else if (HAL_GPIO_ReadPin(COL2_PORT, COL2_PIN) == GPIO_PIN_RESET) {
        current_raw_key = 2;
    } else if (HAL_GPIO_ReadPin(COL3_PORT, COL3_PIN) == GPIO_PIN_RESET) {
        current_raw_key = 3;
    } else if (HAL_GPIO_ReadPin(COL4_PORT, COL4_PIN) == GPIO_PIN_RESET) {
        current_raw_key = 4;
    }

    // --- Scan Row 2 ---
    if (current_raw_key == 0) {
        HAL_GPIO_WritePin(ROW1_PORT, ROW1_PIN, GPIO_PIN_SET);   // Set previous row HIGH
        HAL_GPIO_WritePin(ROW2_PORT, ROW2_PIN, GPIO_PIN_RESET); // Pull Row 2 LOW

        if (HAL_GPIO_ReadPin(COL1_PORT, COL1_PIN) == GPIO_PIN_RESET) {
            current_raw_key = 5;
        } else if (HAL_GPIO_ReadPin(COL2_PORT, COL2_PIN) == GPIO_PIN_RESET) {
            current_raw_key = 6;
        } else if (HAL_GPIO_ReadPin(COL3_PORT, COL3_PIN) == GPIO_PIN_RESET) {
            current_raw_key = 7;
        } else if (HAL_GPIO_ReadPin(COL4_PORT, COL4_PIN) == GPIO_PIN_RESET) {
            current_raw_key = 8;
        }
    }

    // --- Scan Row 3 ---
    if (current_raw_key == 0) {
        HAL_GPIO_WritePin(ROW2_PORT, ROW2_PIN, GPIO_PIN_SET);   // Set previous row HIGH
        HAL_GPIO_WritePin(ROW3_PORT, ROW3_PIN, GPIO_PIN_RESET); // Pull Row 3 LOW

        if (HAL_GPIO_ReadPin(COL1_PORT, COL1_PIN) == GPIO_PIN_RESET) {
            current_raw_key = 9;
        } else if (HAL_GPIO_ReadPin(COL2_PORT, COL2_PIN) == GPIO_PIN_RESET) {
            current_raw_key = 10;
        } else if (HAL_GPIO_ReadPin(COL3_PORT, COL3_PIN) == GPIO_PIN_RESET) {
            current_raw_key = 11;
        } else if (HAL_GPIO_ReadPin(COL4_PORT, COL4_PIN) == GPIO_PIN_RESET) {
            current_raw_key = 12;
        }
    }

    // --- Scan Row 4 ---
    if (current_raw_key == 0) {
        HAL_GPIO_WritePin(ROW3_PORT, ROW3_PIN, GPIO_PIN_SET);   // Set previous row HIGH
        HAL_GPIO_WritePin(ROW4_PORT, ROW4_PIN, GPIO_PIN_RESET); // Pull Row 4 LOW

        if (HAL_GPIO_ReadPin(COL1_PORT, COL1_PIN) == GPIO_PIN_RESET) {
            current_raw_key = 13;
        } else if (HAL_GPIO_ReadPin(COL2_PORT, COL2_PIN) == GPIO_PIN_RESET) {
            current_raw_key = 14;
        } else if (HAL_GPIO_ReadPin(COL3_PORT, COL3_PIN) == GPIO_PIN_RESET) {
            current_raw_key = 15;
        } else if (HAL_GPIO_ReadPin(COL4_PORT, COL4_PIN) == GPIO_PIN_RESET) {
            current_raw_key = 16;
        }
    }

    // After scanning, set all rows high to return to idle state
    HAL_GPIO_WritePin(ROW1_PORT, ROW1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ROW2_PORT, ROW2_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ROW3_PORT, ROW3_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ROW4_PORT, ROW4_PIN, GPIO_PIN_SET);

    return current_raw_key;
}
#endif

void Key_Proc(void)
{
    static KeyValue_t key_val_old = KEY_NONE;
    /* 获取消抖后的按键值 */
    KeyValue_t key_val = Key_GetDebounced();
    /* 只有在按键值变化且不为KEY_NONE时才处理 */
    if (key_val != KEY_NONE && key_val != key_val_old) {
        /* 根据当前显示模式处理按键 */
        switch (g_oled_mode) {
            case CH_X_10: {
                switch (key_val) {
                    case USER_KEY:
                        OLED_ChangeMode();
                        break;
                    case KEY_0:
                        /* 微调增加X轴舵机角度 */
                        g_servox_duty += 10;
                        if (g_servox_duty > SERVO_PWM_MAX) g_servox_duty = SERVO_PWM_MAX;
                        break;
                    case KEY_1:
                        /* 微调减少X轴舵机角度 */
                        g_servox_duty -= 10;
                        if (g_servox_duty < SERVO_PWM_MIN) g_servox_duty = SERVO_PWM_MIN;
                        break;
                }
                break;
            }
            case CH_X_50: {
                switch (key_val) {
                    case USER_KEY:
                        OLED_ChangeMode();
                        break;
                    case KEY_0:
                        /* 粗调增加X轴舵机角度 */
                        g_servox_duty += 50;
                        if (g_servox_duty > SERVO_PWM_MAX) g_servox_duty = SERVO_PWM_MAX;
                        break;
                    case KEY_1:
                        /* 粗调减少X轴舵机角度 */
                        g_servox_duty -= 50;
                        if (g_servox_duty < SERVO_PWM_MIN) g_servox_duty = SERVO_PWM_MIN;
                        break;
                }
                break;
            }
            case CH_Y_10: {
                switch (key_val) {
                    case USER_KEY:
                        OLED_ChangeMode();
                        break;
                    case KEY_0:
                        /* 微调增加Y轴舵机角度 */
                        g_servoy_duty += 10;
                        if (g_servoy_duty > SERVO_PWM_MAX) g_servoy_duty = SERVO_PWM_MAX;
                        break;
                    case KEY_1:
                        /* 微调减少Y轴舵机角度 */
                        g_servoy_duty -= 10;
                        if (g_servoy_duty < SERVO_PWM_MIN) g_servoy_duty = SERVO_PWM_MIN;
                        break;
                }
                break;
            }
            case CH_Y_50: {
                switch (key_val) {
                    case USER_KEY:
                        OLED_ChangeMode();
                        break;
                    case KEY_0:
                        /* 粗调增加Y轴舵机角度 */
                        g_servoy_duty += 50;
                        if (g_servoy_duty > SERVO_PWM_MAX) g_servoy_duty = SERVO_PWM_MAX;
                        break;
                    case KEY_1:
                        /* 粗调减少Y轴舵机角度 */
                        g_servoy_duty -= 50;
                        if (g_servoy_duty < SERVO_PWM_MIN) g_servoy_duty = SERVO_PWM_MIN;
                        break;
                }
                break;
            }
            case TEST_MODE:
                switch (key_val) {
                    case USER_KEY:
                        OLED_ChangeMode();
                        break;
                    case KEY_0:
                        Task1_Reset_To_Ctr(); // 执行任务1
                        break;
                    case KEY_1:
                        Task2_Run(); // 执行任务2
                        break;
                }
                break;
        }
        Servo_SetPulseWidth_X(g_servox_duty);
        Servo_SetPulseWidth_Y(g_servoy_duty);
        /* 更新旧按键值 */
        key_val_old = key_val;
    }
    /* 当按键释放时，重置key_val_old */
    if (key_val == KEY_NONE) {
        key_val_old = KEY_NONE;
    }
}
