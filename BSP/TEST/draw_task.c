#include "draw_task.h"

#include "main.h"
#include "servo_user.h"

void Task_Draw(void)
{
    const int16_t p1_x = 0, p1_y = 0;
    const int16_t p2_x = -10, p2_y = 10;

    // 插值参数
    const uint8_t num_steps = 100;     // 插值步数
    const uint16_t step_delay = 2;  // 每步延时(ms)
    const float distance = 100.0f;    // 垂直距离(cm)

    // 计算两点间的向量
    int16_t delta_x = p2_x - p1_x;
    int16_t delta_y = p2_y - p1_y;

    // 执行线性插值移动
    for (uint8_t i = 0; i <= num_steps; i++) {
        // 计算插值参数 t (从0到1)
        float t = (float)i / (float)num_steps;

        // 计算当前插值点的坐标
        float current_x = p1_x + delta_x * t;
        float current_y = p1_y + delta_y * t;

        // 将坐标转换为舵机角度
        float angle_x = Calc_DeflectionAngle_DirX(distance, current_x);
        float angle_y = Calc_DeflectionAngle_DirY(distance, current_y);

        // 设置舵机角度
        Servo_SetAngle_DirX(angle_x);
        Servo_SetAngle_DirY(angle_y);

        // 更新全局PWM值（用于追踪当前位置）
        g_servox_duty = (uint16_t)(SERVO_PWM_MIN +
                                   (SERVO_PWM_MAX - SERVO_PWM_MIN) * (angle_x + 135.0f) / 270.0f);
        g_servoy_duty = (uint16_t)(SERVO_PWM_MIN +
                                   (SERVO_PWM_MAX - SERVO_PWM_MIN) * (angle_y + 90.0f) / 180.0f);

        // 等待舵机移动到位
        HAL_Delay(step_delay);
    }

    Servo_Return_To_Zero();
}

/**
 * @brief 在两个坐标点之间绘制平滑直线
 * @param start_x 起始点X坐标 (cm)
 * @param start_y 起始点Y坐标 (cm)
 * @param end_x 结束点X坐标 (cm)
 * @param end_y 结束点Y坐标 (cm)
 * @param distance 垂直距离 (cm)
 * @param num_steps 插值步数
 * @param step_delay 每步延时 (ms)
 */
void DrawLine(float start_x, float start_y, float end_x, float end_y, float distance,
              uint8_t num_steps, uint16_t step_delay)
{
    // 计算两点间的向量
    float delta_x = end_x - start_x;
    float delta_y = end_y - start_y;

    // 执行线性插值移动
    for (uint8_t i = 0; i <= num_steps; i++) {
        // 计算插值参数 t (从0到1)
        float t = (float)i / (float)num_steps;

        // 计算当前插值点的坐标
        float current_x = start_x + delta_x * t;
        float current_y = start_y + delta_y * t;

        // 将坐标转换为舵机角度
        float angle_x = Calc_DeflectionAngle_DirX(distance, current_x);
        float angle_y = Calc_DeflectionAngle_DirY(distance, current_y);

        // 设置舵机角度
        Servo_SetAngle_DirX(angle_x);
        Servo_SetAngle_DirY(angle_y);

        // 更新全局PWM值（用于追踪当前位置）
        g_servox_duty = (uint16_t)(SERVO_PWM_MIN +
                                   (SERVO_PWM_MAX - SERVO_PWM_MIN) * (angle_x + 135.0f) / 270.0f);
        g_servoy_duty = (uint16_t)(SERVO_PWM_MIN +
                                   (SERVO_PWM_MAX - SERVO_PWM_MIN) * (angle_y + 90.0f) / 180.0f);

        // 等待舵机移动到位
        HAL_Delay(step_delay);
    }
}

/**
 * @brief 平滑移动到指定坐标点
 * @param target_x 目标X坐标 (cm)
 * @param target_y 目标Y坐标 (cm)
 * @param distance 垂直距离 (cm)
 * @param num_steps 移动步数
 * @param step_delay 每步延时 (ms)
 */
void MoveToPoint(float target_x, float target_y, float distance, uint8_t num_steps,
                 uint16_t step_delay)
{
    // 获取当前位置（从PWM值反推坐标）
    float current_angle_x =
        (float)(g_servox_duty - SERVO_PWM_MIN) * 270.0f / (SERVO_PWM_MAX - SERVO_PWM_MIN) - 135.0f;
    float current_angle_y =
        (float)(g_servoy_duty - SERVO_PWM_MIN) * 180.0f / (SERVO_PWM_MAX - SERVO_PWM_MIN) - 90.0f;

    // 从角度反推当前坐标
    float current_x = (float)(distance * tan(current_angle_x * PI / 180.0f));
    float current_y = (float)(distance * tan(current_angle_y * PI / 180.0f));

    // 绘制从当前位置到目标位置的直线
    DrawLine(current_x, current_y, target_x, target_y, distance, num_steps, step_delay);
}

/**
 * @brief 绘制简单图形示例
 */
void Task_DrawShape(void)
{
    const float distance = 100.0f;  // 垂直距离
    const uint8_t steps = 100;       // 每段的步数
    const uint16_t delay = 5;      // 每步延时
    const float edge = 8;

    // Servo_SetPulseWidth_DirX(SERVO_VERTICAL_X_DUTY);
    // HAL_Delay(500);
    // Servo_SetPulseWidth_DirY(SERVO_VERTICAL_Y_DUTY);
    // HAL_Delay(500);
    // 绘制一个正方形，从(0,0)开始
    // DrawLine(0.0f, 0.0f, -edge, 0.0f, distance, steps, delay);
    // HAL_Delay(500);

    // DrawLine(-10.0f, 0.0f, -edge, edge, distance, steps, delay);
    // HAL_Delay(500);

    // // 点3: 从左上角到左下角(-5,-5)
    // DrawLine(-5.0f, 5.0f, -5.0f, -5.0f, distance, steps, delay);
    // HAL_Delay(200);

    // // 点4: 从左下角到右下角(5,-5)
    // DrawLine(-5.0f, -5.0f, 5.0f, -5.0f, distance, steps, delay);
    // HAL_Delay(200);

    // // 回到原点闭合
    // DrawLine(5.0f, -5.0f, 0.0f, 0.0f, distance, steps, delay);
    HAL_Delay(500);

    // 最后回到零点
    // Servo_Return_To_Zero();

}
