#include "key.h"
#include "servo.h"
#include "stdio.h"
#include "task_scheduler.h"

/* 按键消抖实例 */
static KeyDebounce_t key_debounce;

KeyValue_t Key_GetNum(void)
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

/*
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
*/

static uint8_t key_mode = 0;

int16_t servox_angle = 0;
int16_t servoy_angle = 0;

void Key_Proc(void)
{
    static KeyValue_t key_val_old = KEY_NONE;
    /* 获取消抖后的按键值 */
    KeyValue_t key_val = Key_GetDebounced();
    /* 只有在按键值变化且不为KEY_NONE时才处理 */
    if (key_val != KEY_NONE && key_val != key_val_old) {
        printf("Key Value: %d\n", key_val);
        if (key_mode == 0) {
            /* 模式0：控制X轴舵机 */
            switch (key_val) {
                case USER_KEY:
                    key_mode = 1;
                    printf("Switch to Y-axis control mode\n");
                    break;
                case KEY_0:
                    servox_angle += 1;
                    if (servox_angle > 90) servox_angle = 90;  /* 角度限制 */
                    printf("Servo X angle: %d\n", servox_angle);
                    break;
                case KEY_1:
                    servox_angle -= 1;
                    if (servox_angle < -90) servox_angle = -90;  /* 角度限制 */
                    printf("Servo X angle: %d\n", servox_angle);
                    break;
            }
        } else if (key_mode == 1) {
            /* 模式1：控制Y轴舵机 */
            switch (key_val) {
                case USER_KEY:
                    key_mode = 0;
                    printf("Switch to X-axis control mode\n");
                    break;
                case KEY_0:
                    servoy_angle += 1;
                    if (servoy_angle > 90) servoy_angle = 90;  /* 角度限制 */
                    printf("Servo Y angle: %d\n", servoy_angle);
                    break;
                case KEY_1:
                    servoy_angle -= 1;
                    if (servoy_angle < -90) servoy_angle = -90;  /* 角度限制 */
                    printf("Servo Y angle: %d\n", servoy_angle);
                    break;
            }
        }
        /* 更新舵机角度 */
        Servo_SetAngle_X(servox_angle);
        Servo_SetAngle_Y(servoy_angle);
        key_val_old = key_val;
    }
    /* 当按键释放时，重置key_val_old */
    if (key_val == KEY_NONE) {
        key_val_old = KEY_NONE;
    }
}
