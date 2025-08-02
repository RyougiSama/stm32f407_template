#include "key.h"

#include "Emm_V5.h"
#include "laser_shot_common.h"
#include "oled_user.h"
#include "task_scheduler.h"

// GPIO端口和引脚宏定义兼容
#define ROW1_PORT ROW1_GPIO_Port
#define ROW1_PIN ROW1_Pin
#define ROW2_PORT ROW2_GPIO_Port
#define ROW2_PIN ROW2_Pin
#define ROW3_PORT ROW3_GPIO_Port
#define ROW3_PIN ROW3_Pin
#define ROW4_PORT ROW4_GPIO_Port
#define ROW4_PIN ROW4_Pin
#define COL1_PORT COL1_GPIO_Port
#define COL1_PIN COL1_Pin
#define COL2_PORT COL2_GPIO_Port
#define COL2_PIN COL2_Pin
#define COL3_PORT COL3_GPIO_Port
#define COL3_PIN COL3_Pin
#define COL4_PORT COL4_GPIO_Port
#define COL4_PIN COL4_Pin

/* 按键消抖实例 */
static KeyDebounce_t key_debounce;

// static KeyValue_t Key_GetNum(void)
// {
//     KeyValue_t key_num = KEY_NONE;
//     if (HAL_GPIO_ReadPin(GPIOE, KEY_0_Pin) == GPIO_PIN_RESET) {
//         key_num = KEY_0;
//     }
//     if (HAL_GPIO_ReadPin(GPIOE, KEY_1_Pin) == GPIO_PIN_RESET) {
//         key_num = KEY_1;
//     }
//     if (HAL_GPIO_ReadPin(USER_KEY_GPIO_Port, USER_KEY_Pin) == GPIO_PIN_SET) {
//         key_num = USER_KEY;
//     }
//     if (HAL_GPIO_ReadPin(COL1_GPIO_Port, COL1_Pin) == GPIO_PIN_RESET) {
//         key_num = KEY_S1;
//     }
//     if (HAL_GPIO_ReadPin(COL2_GPIO_Port, COL2_Pin) == GPIO_PIN_RESET) {
//         key_num = KEY_S2;
//     }
//     if (HAL_GPIO_ReadPin(COL3_GPIO_Port, COL3_Pin) == GPIO_PIN_RESET) {
//         key_num = KEY_S3;
//     }
//     if (HAL_GPIO_ReadPin(COL4_GPIO_Port, COL4_Pin) == GPIO_PIN_RESET) {
//         key_num = KEY_S4;
//     }
//     return key_num;
// }

/**
 * @brief 获取经过消抖处理的按键值
 *
 * @return KeyValue_t 消抖后的按键值，KEY_NONE表示无按键，其他值表示有按键按下
 */
KeyValue_t Key_GetDebounced(void)
{
    uint32_t current_tick = TaskScheduler_GetSystemTick();
    KeyValue_t raw_key = Matrix_Key_Scan();
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
                /* 将缓存中为的key值 */
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
                /* 按键仍在按下状态 */
                key_debounce.current_val = raw_key;
                key_debounce.debounce_timer = current_tick;
                key_debounce.state = KEY_STATE_PRESSED;
            }
            break;
    }
    return KEY_NONE; /* 无稳定按键 */
}

KeyValue_t Matrix_Key_Scan(void)
{
    uint8_t current_raw_key = 0;

    // --- Scan Row 1 ---
    HAL_GPIO_WritePin(ROW1_PORT, ROW1_PIN, GPIO_PIN_RESET);  // Pull Row 1 LOW
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
        HAL_GPIO_WritePin(ROW1_PORT, ROW1_PIN, GPIO_PIN_SET);    // Set previous row HIGH
        HAL_GPIO_WritePin(ROW2_PORT, ROW2_PIN, GPIO_PIN_RESET);  // Pull Row 2 LOW

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
        HAL_GPIO_WritePin(ROW2_PORT, ROW2_PIN, GPIO_PIN_SET);    // Set previous row HIGH
        HAL_GPIO_WritePin(ROW3_PORT, ROW3_PIN, GPIO_PIN_RESET);  // Pull Row 3 LOW

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
        HAL_GPIO_WritePin(ROW3_PORT, ROW3_PIN, GPIO_PIN_SET);    // Set previous row HIGH
        HAL_GPIO_WritePin(ROW4_PORT, ROW4_PIN, GPIO_PIN_RESET);  // Pull Row 4 LOW

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

    // 将数字键值映射到枚举类型
    // 4x4矩阵键盘布局映射：
    // 第1行：1=KEY_S1,  2=KEY_S2,  3=KEY_S3,  4=KEY_S4
    // 第2行：5=KEY_S5,  6=KEY_S6,  7=KEY_S7,  8=KEY_S8
    // 第3行：9=KEY_S9,  10=KEY_S10, 11=KEY_S11, 12=KEY_S12
    // 第4行：13=KEY_S13, 14=KEY_S14, 15=KEY_S15, 16=KEY_S16
    switch (current_raw_key) {
        case 1:
            return KEY_S1;  // 第1行第1列
        case 2:
            return KEY_S2;  // 第1行第2列
        case 3:
            return KEY_S3;  // 第1行第3列
        case 4:
            return KEY_S4;  // 第1行第4列
        case 5:
            return KEY_S5;  // 第2行第1列
        case 6:
            return KEY_S6;  // 第2行第2列
        case 7:
            return KEY_S7;  // 第2行第3列
        case 8:
            return KEY_S8;  // 第2行第4列
        case 9:
            return KEY_S9;  // 第3行第1列
        case 10:
            return KEY_S10;  // 第3行第2列
        case 11:
            return KEY_S11;  // 第3行第3列
        case 12:
            return KEY_S12;  // 第3行第4列
        case 13:
            return KEY_S13;  // 第4行第1列
        case 14:
            return KEY_S14;  // 第4行第2列
        case 15:
            return KEY_S15;  // 第4行第3列
        case 16:
            return KEY_S16;  // 第4行第4列
        default:
            return KEY_NONE;  // 无按键或未定义的按键
    }
}

void Key_Proc(void)
{
    static KeyValue_t key_val_old = KEY_NONE;

    /* 获取经过消抖的按键值 */
    KeyValue_t key_val = Key_GetDebounced();
    /* 只有在按键值变化且不为KEY_NONE时才处理 */
    if (key_val != KEY_NONE && key_val != key_val_old) {
        if (key_val == KEY_S1) {
            g_task_basic_q2_with_zdt_running = !g_task_basic_q2_with_zdt_running;  // 切换任务状态
            g_task_basic_q2_with_zdt_start_time = HAL_GetTick();
        } else if (key_val == KEY_S2) {
            // Emm_V5_Origin_Set_O(STEP_MOTOR_X, true);
            // Emm_V5_Origin_Trigger_Return(STEP_MOTOR_Y, 1, false);
        } else if (key_val == KEY_S3) {
        } else if (key_val == KEY_S4) {
            // if (Laser_TrackAimPoint_IsRunning()) {
            //     Laser_TrackAimPoint_Stop();
            // } else {
            //     Laser_TrackAimPoint_Start();
            // }
        } else if (key_val == KEY_S5) {
            // S5任务：90度左转
            Task_Q3_Key_S5_Start();
        } else if (key_val == KEY_S6) {
            // S6任务：45度左转
            Task_Q3_Key_S6_Start();
        } else if (key_val == KEY_S7) {
            // S7任务：90度右转
            Task_Q3_Key_S7_Start();
        } else if (key_val == KEY_S8) {
            // S8任务：135度左转
            Task_Q3_Key_S8_Start();
        } else if (key_val == KEY_S9) {
            // 处理KEY_S9按键逻辑
        } else if (key_val == KEY_S10) {
            // 处理KEY_S10按键逻辑
        } else if (key_val == KEY_S11) {
            // 处理KEY_S11按键逻辑
        } else if (key_val == KEY_S12) {
            // 处理KEY_S12按键逻辑
        } else if (key_val == KEY_S13) {
            // 处理KEY_S13按键逻辑
            Emm_V5_Origin_Set_O(STEP_MOTOR_X, true);
        } else if (key_val == KEY_S14) {
            // 处理KEY_S14按键逻辑
            Emm_V5_Origin_Trigger_Return(STEP_MOTOR_X, 0, false);
        } else if (key_val == KEY_S15) {
            // 处理KEY_S15按键逻辑
            Emm_V5_Origin_Set_O(STEP_MOTOR_Y, true);
        } else if (key_val == KEY_S16) {
            // 处理KEY_S16按键逻辑
            Emm_V5_Origin_Trigger_Return(STEP_MOTOR_Y, 0, false);
        }
    }
    /* 当按键释放时清除旧key_val_old */
    if (key_val == KEY_NONE) {
        key_val_old = KEY_NONE;
    }
}
