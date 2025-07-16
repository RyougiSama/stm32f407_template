#include "control.h"
#include "servo.h"

uint16_t reset_x = 1550; // X轴复位占空比
uint16_t reset_y = 1490; // Y轴复位占空比

uint16_t T2_roll_1 = 1580;  // T2任务滚转角度1
uint16_t T2_roll_2 = 1400;  // T2任务滚转角度2
uint16_t T2_pitch_1 = 1420; // T2任务俯仰角度1
uint16_t T2_pitch_2 = 1690; // T2任务俯仰角度2

/**
 * @brief  reset to center
 * @param  None
 *
 * @retval None
 */
void Task1_Reset_To_Ctr(void)
{
    Servo_SetPulseWidth_X(reset_x); // Reset X-axis to center
    Servo_SetPulseWidth_Y(reset_y); // Reset Y-axis to center
    HAL_Delay(500);                 // Wait for servo to stabilize
}

/**
 * @brief  run task 2
 * @param  None
 *
 * @retval None
 */
void Task2_Run(void)
{
    float cnt = 0;
    /*start*/
    Servo_SetPulseWidth_X(T2_pitch_1); // set X-axis to first position
    Servo_SetPulseWidth_Y(T2_roll_1);  // set Y-axis to first position

    HAL_Delay(500);

    /*roll_1 -> roll_2*/
    while (T2_roll_1 - cnt > T2_roll_2)
    {
        cnt += 20;

        Servo_SetPulseWidth_X(T2_pitch_1);      // Reset X-axis to center
        Servo_SetPulseWidth_Y(T2_roll_2 - cnt); // Reset Y-axis to center

        HAL_Delay(15);
    }
    cnt = 0;

    /*pitch_1 -> pitch_2*/
    HAL_Delay(100);
    while (T2_pitch_1 + cnt < T2_pitch_2)
    {
        cnt += 20;

        Servo_SetPulseWidth_X(T2_pitch_1 + cnt); // Reset X-axis to center
        Servo_SetPulseWidth_Y(T2_roll_2);        // Reset Y-axis to center
        HAL_Delay(15);
    }
    cnt = 0;

    /*roll_2 -> roll_1*/
    HAL_Delay(100);
    while (T2_roll_2 + cnt < T2_roll_1)
    {
        cnt += 20;

        Servo_SetPulseWidth_X(T2_pitch_2);      // Reset X-axis to center
        Servo_SetPulseWidth_Y(T2_roll_2 + cnt); // Reset Y-axis to center

        HAL_Delay(15);
    }
    cnt = 0;

    /*pitch_2 -> pitch_1*/
    HAL_Delay(100);
    while (T2_pitch_2 - cnt > T2_pitch_1)
    {
        cnt += 20;

        Servo_SetPulseWidth_X(T2_pitch_2 - cnt); // Reset X-axis to center
        Servo_SetPulseWidth_Y(T2_roll_1);        // Reset Y-axis to center

        HAL_Delay(15);
    }

    HAL_Delay(500);
    Task1_Reset_To_Ctr();
}
