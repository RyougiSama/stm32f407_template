#include "key.h"

#include "Emm_V5.h"
#include "laser_shot_common.h"
#include "oled_user.h"
#include "task_scheduler.h"

/* 按键消抖实例 */
static KeyDebounce_t key_debounce;

static KeyValue_t Key_GetNum(void)
{
    KeyValue_t key_num = KEY_NONE;
    if (HAL_GPIO_ReadPin(GPIOE, KEY_0_Pin) == GPIO_PIN_RESET) {
        key_num = KEY_0;
    }
    if (HAL_GPIO_ReadPin(GPIOE, KEY_1_Pin) == GPIO_PIN_RESET) {
        key_num = KEY_1;
    }
    if (HAL_GPIO_ReadPin(USER_KEY_GPIO_Port, USER_KEY_Pin) == GPIO_PIN_SET) {
        key_num = USER_KEY;
    }
    if (HAL_GPIO_ReadPin(COL1_GPIO_Port, COL1_Pin) == GPIO_PIN_RESET) {
        key_num = KEY_S1;
    }
    if (HAL_GPIO_ReadPin(COL2_GPIO_Port, COL2_Pin) == GPIO_PIN_RESET) {
        key_num = KEY_S2;
    }
    if (HAL_GPIO_ReadPin(COL3_GPIO_Port, COL3_Pin) == GPIO_PIN_RESET) {
        key_num = KEY_S3;
    }
    if (HAL_GPIO_ReadPin(COL4_GPIO_Port, COL4_Pin) == GPIO_PIN_RESET) {
        key_num = KEY_S4;
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
    /* 获取经过消抖的按键值 */
    KeyValue_t key_val = Key_GetDebounced();
    /* 只有在按键值变化且不为KEY_NONE时才处理 */
    if (key_val != KEY_NONE && key_val != key_val_old) {
        if (key_val == KEY_S1) {
            g_task_basic_q2_with_zdt_running = !g_task_basic_q2_with_zdt_running;  // 切换任务状态
            g_task_basic_q2_with_zdt_start_time = HAL_GetTick();
        } else if (key_val == KEY_S2) {
            // Emm_V5_Origin_Set_O(STEP_MOTOR_Y, true);
            // Emm_V5_Origin_Trigger_Return(STEP_MOTOR_X, 1, false);
            HAL_GPIO_WritePin(OUTPUT_TEST_GPIO_Port, OUTPUT_TEST_Pin, GPIO_PIN_SET);
            HAL_Delay(20);
            // OLED_ShowString(0, 0, "OK", 16);
        } else if (key_val == KEY_S3) {
            Task_BasicQ3_Start();
        } else if (key_val == KEY_S4) {
            if (Laser_TrackAimPoint_IsRunning()) {
                Laser_TrackAimPoint_Stop();
            } else {
                Laser_TrackAimPoint_Start();
            }
        }

#if 0
        if (key_val == KEY_S4 || key_val == USER_KEY) {
            OLED_ChangeMode();
        } else {
            if (g_oled_mode == DISP_CENTER_POINT) {

            } else if (g_oled_mode == SET_ZERO_POINT) {
                if (key_val == KEY_S1) {
                    current_set_zero_addr =
                        (current_set_zero_addr == STEP_MOTOR_X) ? STEP_MOTOR_Y : STEP_MOTOR_X;
                } else if (key_val == KEY_S2) {
                    // 执行设置零点操作
                    Emm_V5_Origin_Set_O(current_set_zero_addr, true);
                } else if (key_val == KEY_S3) {
                    Emm_V5_Origin_Trigger_Return(current_set_zero_addr, 0, false);
                    HAL_Delay(20);
                }
            } else if (g_oled_mode == TEST_MODE) {
                if (key_val == KEY_S1) {
                    if (Task_BasicQ2_WithZDT_IsRunning()) {
                        Task_BasicQ2_WithZDT_Stop();
                    } else {
                        Task_BasicQ2_WithZDT_Start();
                        g_task_basic_q2_with_zdt_start_time = TaskScheduler_GetSystemTick();
                    }
                } else if (key_val == KEY_S2) {
                    // Q3任务控制：主动搜索矩形并追踪
                    if (Task_BasicQ3_IsRunning()) {
                        Task_BasicQ3_Stop();
                    } else {
                        Task_BasicQ3_Start();
                    }
                }
            }
        }
#endif
#if 0
        switch (key_val)
        {
        case KEY_0:
            if (Task_BasicQ2_WithZDT_IsRunning()) {
                Task_BasicQ2_WithZDT_Stop();
            } else {
                Task_BasicQ2_WithZDT_Start();
                g_task_basic_q2_with_zdt_start_time = TaskScheduler_GetSystemTick();
            }
            break;
        case KEY_1:
            // Q3任务控制：主动搜索矩形并追踪
            if (Task_BasicQ3_IsRunning()) {
                Task_BasicQ3_Stop();
            } else {
                Task_BasicQ3_Start();
            }
            break;
        case KEY_S1:
            // 激光追踪控制：持续追踪目标点
            if (Laser_TrackAimPoint_IsRunning()) {
                Laser_TrackAimPoint_Stop();
            } else {
                Laser_TrackAimPoint_Start();
            }
            break;
        case KEY_S2:
            break;
        case KEY_S3:
            break;
        case KEY_S4:
            OLED_ChangeMode();
            break;
        default:
            break;
        }
#endif
    }
    /* 当按键释放时清除旧key_val_old */
    if (key_val == KEY_NONE) {
        key_val_old = KEY_NONE;
    }
}
