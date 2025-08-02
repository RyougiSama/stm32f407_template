/**
 * @file key.h
 * @author 
 * @brief Header file for key handling functions
 * @version 0.1
 * @date 2025-07-15
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef __KEY_H
#define __KEY_H

#include <stdint.h>
#include "main.h"

/* 按键消抖时间定义 (ms) */
#define KEY_DEBOUNCE_TIME 20

/* 按键值枚举定义 */
typedef enum {
    KEY_NONE = 0,
    KEY_0 = 1,
    KEY_1 = 2,
    USER_KEY = 3,
    KEY_S1,      // 4
    KEY_S2,      // 5
    KEY_S3,      // 6
    KEY_S4,      // 7
    KEY_S5,      // 8  - 新增
    KEY_S6,      // 9  - 新增
    KEY_S7,      // 10 - 新增
    KEY_S8,      // 11 - 新增
    KEY_S9,      // 12 - 新增
    KEY_S10,     // 13 - 新增
    KEY_S11,     // 14 - 新增
    KEY_S12,     // 15 - 新增
    KEY_S13,     // 16 - 新增
    KEY_S14,     // 17 - 新增
    KEY_S15,     // 18 - 新增
    KEY_S16      // 19 - 新增
} KeyValue_t;

/* 按键状态定义 */
typedef enum {
    KEY_STATE_IDLE = 0,
    KEY_STATE_PRESSED,
    KEY_STATE_CONFIRMED,
    KEY_STATE_RELEASED
} KeyState_t;

/* 按键消抖结构体 */
typedef struct {
    KeyValue_t current_val;   /* 当前按键值 */
    KeyValue_t last_val;      /* 上次按键值 */
    KeyValue_t stable_val;    /* 稳定按键值 */
    uint32_t debounce_timer;  /* 消抖计时器 */
    KeyState_t state;         /* 按键状态 */
    uint8_t key_pressed;      /* 按键按下标志 */
} KeyDebounce_t;

/* 函数声明 */
KeyValue_t Key_GetDebounced(void);
void Key_Proc(void);
KeyValue_t Matrix_Key_Scan(void);  // 修改返回类型为KeyValue_t


#endif
