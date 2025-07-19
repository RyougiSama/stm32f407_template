#include "control.h"
#include "servo_user.h"
#include "math.h"
#include "pid_controller.h"

static const uint16_t reset_x = SERVO_VERTICAL_X_DUTY; // X轴复位占空比
static const uint16_t reset_y = SERVO_VERTICAL_Y_DUTY; // Y轴复位占空比

static const uint16_t T2_roll_1 = 1580;  // T2任务滚转角度1
static const uint16_t T2_roll_2 = 1390;  // T2任务滚转角度2
static const uint16_t T2_pitch_1 = 1410; // T2任务俯仰角度1
static const uint16_t T2_pitch_2 = 1690; // T2任务俯仰角度2

/**
 * @brief  reset to center
 * @param  None
 *
 * @retval None
 */
void Task1_Reset_To_Ctr(void)
{
    g_servox_duty = reset_x;
    g_servoy_duty = reset_y;
    Servo_SetPulseWidth_DirX(reset_x); // Reset X-axis to center
    Servo_SetPulseWidth_DirY(reset_y); // Reset Y-axis to center
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
    const uint16_t step_delay = 200, process_delay = 1000;
    uint16_t cnt = 0;
    /*start*/
    Servo_SetPulseWidth_DirY(T2_pitch_1); // set X-axis to first position
    Servo_SetPulseWidth_DirX(T2_roll_1);  // set Y-axis to first position
    HAL_Delay(process_delay);
    /*roll_1 -> roll_2*/
    while (T2_roll_1 - cnt > T2_roll_2) {
        cnt += 20;
        Servo_SetPulseWidth_DirY(T2_pitch_1);      // Reset X-axis to center
        Servo_SetPulseWidth_DirX(T2_roll_1 - cnt); // Reset Y-axis to center
        HAL_Delay(step_delay);
    }
    cnt = 0;
    /*pitch_1 -> pitch_2*/
    HAL_Delay(process_delay);
    while (T2_pitch_1 + cnt < T2_pitch_2) {
        cnt += 20;
        Servo_SetPulseWidth_DirY(T2_pitch_1 + cnt); // Reset X-axis to center
        Servo_SetPulseWidth_DirX(T2_roll_2);        // Reset Y-axis to center
        HAL_Delay(step_delay);
    }
    cnt = 0;
    /*roll_2 -> roll_1*/
    HAL_Delay(100);
    while (T2_roll_2 + cnt < T2_roll_1) {
        cnt += 20;
        Servo_SetPulseWidth_DirY(T2_pitch_2);      // Reset X-axis to center
        Servo_SetPulseWidth_DirX(T2_roll_2 + cnt); // Reset Y-axis to center
        HAL_Delay(step_delay);
    }
    cnt = 0;
    /*pitch_2 -> pitch_1*/
    HAL_Delay(process_delay);
    while (T2_pitch_2 - cnt > T2_pitch_1) {
        cnt += 20;
        Servo_SetPulseWidth_DirY(T2_pitch_2 - cnt); // Reset X-axis to center
        Servo_SetPulseWidth_DirX(T2_roll_1);        // Reset Y-axis to center
        HAL_Delay(step_delay);
    }
    HAL_Delay(process_delay);
    Task1_Reset_To_Ctr();
}


LaserPoint_t g_laser_point = {0, 0}; // Laser point coordinates
LaserPoint_t g_lu_corner, g_ru_corner, g_ld_corner, g_rd_corner; // Laser corners
uint8_t g_is_corner_init;

/**
 * @brief 任务3 - 遍历矩形的四个角点，沿边沿移动
 * @note 先追踪到左上角点，然后沿边沿移动，每次只改变X或Y坐标
 */
void Task3_Run(void)
{
    const uint16_t corner_delay = 500; // 到达每个角点后的停留时间
    const float pixel_to_pwm_ratio = 1.8f; // 像素到PWM的映射比例
    
    // 第一步：追踪到左上角点
    MoveToPointStraight(g_lu_corner);
    HAL_Delay(corner_delay);
    
    // 沿边沿移动到右上角点 (左上 -> 右上，只改变X坐标)
    MoveAlongEdge(g_ru_corner, 1, 0, pixel_to_pwm_ratio); // 只改变X方向
    HAL_Delay(corner_delay);
    
    // 沿边沿移动到右下角点 (右上 -> 右下，只改变Y坐标)
    MoveAlongEdge(g_rd_corner, 0, 1, pixel_to_pwm_ratio); // 只改变Y方向
    HAL_Delay(corner_delay);
    
    // 沿边沿移动到左下角点 (右下 -> 左下，只改变X坐标)
    MoveAlongEdge(g_ld_corner, 1, 0, pixel_to_pwm_ratio); // 只改变X方向
    HAL_Delay(corner_delay);
    
    // 沿边沿移动回左上角点 (左下 -> 左上，只改变Y坐标)
    MoveAlongEdge(g_lu_corner, 0, 1, pixel_to_pwm_ratio); // 只改变Y方向
    HAL_Delay(corner_delay);
}

/**
 * @brief 任务4 - 遍历任意角度摆放的矩形的四条边
 * @note 通过计算直线方程生成路径点，使用PID控制精确追踪
 */
void Task4_Run(void)
{
    // 检查角点是否已初始化
    if (!g_is_corner_init) {
        return; // 角点未初始化，直接返回
    }
    TrackPixelPoint(g_lu_corner); // 先追踪到左上角点

    const uint8_t points_per_edge = 10; // 每条边生成的点数
    const uint16_t point_delay = 200;   // 每个点的停留时间(ms)
    // 遍历四条边：左上→右上→右下→左下→左上
    TraverseRectangleEdge(g_lu_corner, g_ru_corner, points_per_edge, point_delay); // 边1: 左上→右上
    TraverseRectangleEdge(g_ru_corner, g_rd_corner, points_per_edge, point_delay); // 边2: 右上→右下
    TraverseRectangleEdge(g_rd_corner, g_ld_corner, points_per_edge, point_delay); // 边3: 右下→左下
    TraverseRectangleEdge(g_ld_corner, g_lu_corner, points_per_edge, point_delay); // 边4: 左下→左上
}

/**
 * @brief 沿边沿移动到目标点，只改变指定的坐标轴
 * @param target_point 目标坐标点
 * @param change_x 是否改变X坐标 (1:改变, 0:不变)
 * @param change_y 是否改变Y坐标 (1:改变, 0:不变)
 * @param pixel_ratio 像素到PWM的映射比例
 */
void MoveAlongEdge(LaserPoint_t target_point, uint8_t change_x, uint8_t change_y, float pixel_ratio)
{
    const uint8_t tolerance = 1; // 位置容差
    const uint16_t step_delay = 60; // 每步延时
    const uint32_t max_runtime = 8000; // 最大运行时间8秒
    const float max_step_size = 2.5f; // 最大单步移动像素数
    
    uint32_t start_time = HAL_GetTick();
    
    while (1) {
        // 检查是否超时
        if (HAL_GetTick() - start_time > max_runtime) {
            break;
        }
        
        // 检查当前激光点位置是否有效
        if (g_laser_point.x == 0 || g_laser_point.y == 0) {
            HAL_Delay(step_delay);
            continue;
        }
        
        // 计算误差
        float x_error = 0, y_error = 0;
        if (change_x) {
            x_error = target_point.x - g_laser_point.x;
        }
        if (change_y) {
            y_error = target_point.y - g_laser_point.y;
        }
        
        // 检查是否到达目标
        float total_error = sqrt(x_error * x_error + y_error * y_error);
        if (total_error <= tolerance) {
            break; // 到达目标
        }
        
        // 计算需要移动的步长
        float step_size = total_error;
        if (step_size > max_step_size) {
            step_size = max_step_size;
        }
        
        // 计算PWM调整量
        int16_t pwm_delta_x = 0, pwm_delta_y = 0;
        
        if (change_x && fabs(x_error) > tolerance) {
            // 计算X方向的调整
            float x_direction = (x_error > 0) ? 1.0f : -1.0f;
            float x_step = (fabs(x_error) > step_size) ? step_size : fabs(x_error);
            pwm_delta_x = (int16_t)(-x_direction * x_step * pixel_ratio);
        }
        
        if (change_y && fabs(y_error) > tolerance) {
            // 计算Y方向的调整
            float y_direction = (y_error > 0) ? 1.0f : -1.0f;
            float y_step = (fabs(y_error) > step_size) ? step_size : fabs(y_error);
            pwm_delta_y = (int16_t)(y_direction * y_step * pixel_ratio);
        }
        
        // 更新舵机PWM值
        g_servox_duty += pwm_delta_x;
        g_servoy_duty += pwm_delta_y;
        
        // 限制PWM范围
        if (g_servox_duty > SERVO_PWM_MAX) g_servox_duty = SERVO_PWM_MAX;
        if (g_servox_duty < SERVO_PWM_MIN) g_servox_duty = SERVO_PWM_MIN;
        if (g_servoy_duty > SERVO_PWM_MAX) g_servoy_duty = SERVO_PWM_MAX;
        if (g_servoy_duty < SERVO_PWM_MIN) g_servoy_duty = SERVO_PWM_MIN;
        
        // 设置舵机位置
        Servo_SetPulseWidth_DirX(g_servox_duty);
        Servo_SetPulseWidth_DirY(g_servoy_duty);
        
        HAL_Delay(step_delay);
    }
}

/**
 * @brief 使用直线轨迹算法移动到指定像素位置
 * @param target_point 目标坐标点结构体
 * @note 使用直线插值算法，避免振荡现象
 */
void MoveToPointStraight(LaserPoint_t target_point)
{
    // 如果没有有效的激光点数据，直接返回
    if (g_laser_point.x == 0 || g_laser_point.y == 0) {
        return;
    }
    
    const uint8_t tolerance = 1; // 位置容差
    const uint16_t step_delay = 40; // 每步延时
    const uint32_t max_runtime = 5000; // 最大运行时间5秒
    
    uint32_t start_time = HAL_GetTick();
    
    while (1) {
        // 检查是否超时
        if (HAL_GetTick() - start_time > max_runtime) {
            break;
        }
        
        // 检查是否到达目标
        int16_t x_error = target_point.x - g_laser_point.x;
        int16_t y_error = target_point.y - g_laser_point.y;
        
        if (fabs(x_error) <= tolerance && fabs(y_error) <= tolerance) {
            break; // 到达目标
        }
        
        // 计算需要移动的距离
        float distance = sqrt(x_error * x_error + y_error * y_error);
        
        if (distance <= 0.1f) {
            break; // 距离太小，认为已到达
        }
        
        // 自适应步长控制
        float step_size;
        if (distance > 15) {
            step_size = 4.0f; // 距离远时用大步长
        } else if (distance > 8) {
            step_size = 2.5f; // 中等距离用中步长
        } else if (distance > 4) {
            step_size = 1.5f; // 接近目标用小步长
        } else {
            step_size = 0.8f; // 非常接近时用极小步长
        }
        
        // 限制步长不超过实际距离
        if (step_size > distance) {
            step_size = distance;
        }
        
        // 计算单位方向向量
        float unit_x = x_error / distance;
        float unit_y = y_error / distance;
        
        // 计算PWM值变化
        // 根据实际测试调整像素到PWM的映射比例
        const float pixel_to_pwm_ratio = 1.8f; // 可根据实际情况调整
        
        int16_t pwm_delta_x = (int16_t)(-unit_x * step_size * pixel_to_pwm_ratio);
        int16_t pwm_delta_y = (int16_t)(unit_y * step_size * pixel_to_pwm_ratio);
        
        // 更新舵机PWM值
        g_servox_duty += pwm_delta_x;
        g_servoy_duty += pwm_delta_y;
        
        // 限制PWM范围
        if (g_servox_duty > SERVO_PWM_MAX) g_servox_duty = SERVO_PWM_MAX;
        if (g_servox_duty < SERVO_PWM_MIN) g_servox_duty = SERVO_PWM_MIN;
        if (g_servoy_duty > SERVO_PWM_MAX) g_servoy_duty = SERVO_PWM_MAX;
        if (g_servoy_duty < SERVO_PWM_MIN) g_servoy_duty = SERVO_PWM_MIN;
        
        // 设置舵机位置
        Servo_SetPulseWidth_DirX(g_servox_duty);
        Servo_SetPulseWidth_DirY(g_servoy_duty);
        
        HAL_Delay(step_delay);
    }
}


/**
 * @brief 使用PID控制器实现激光点追踪到指定像素位置
 * @param target_point 目标坐标点结构体
 * @note 使用PID控制以提高精度和稳定性
 */
void TrackPixelPoint(LaserPoint_t target_point)
{
    // PID控制器定义
    PidController_t pid_x, pid_y;
    const uint8_t pos_error = 0; // 增加误差允许范围，避免无法达到完美精度
    const uint16_t step_delay = 60; // 增加延时，减少控制频率
    const uint8_t coarse_threshold = 8; // 粗调阈值
    
    // 初始化X轴PID控制器 - 更保守的参数设置
    PID_Init(&pid_x, PID_TYPE_POSITIONAL);
    PID_SetParam(&pid_x, 0.8f, 0.01f, 0.3f); // 降低P值和I值，减少振荡
    PID_SetOutputLimit(&pid_x, -5.0f, 5.0f); // 减小输出范围
    PID_SetIntegralLimit(&pid_x, -10.0f, 10.0f);
    PID_SetTarget(&pid_x, target_point.x);
    
    // 初始化Y轴PID控制器 - 更保守的参数设置
    PID_Init(&pid_y, PID_TYPE_POSITIONAL);
    PID_SetParam(&pid_y, 0.8f, 0.01f, 0.3f); // 降低P值和I值，减少振荡
    PID_SetOutputLimit(&pid_y, -5.0f, 5.0f); // 减小输出范围
    PID_SetIntegralLimit(&pid_y, -10.0f, 10.0f);
    PID_SetTarget(&pid_y, target_point.y);
    
    // 继续循环直到 X 轴和 Y 轴都到达目标位置
    uint8_t stable_count = 0;
    const uint8_t required_stable_count = 8; // 增加稳定次数要求，确保真正稳定
    uint8_t x_fine_tuning = 0, y_fine_tuning = 0; // 记录微调模式
    float prev_x_error = 0, prev_y_error = 0; // 记录前一次误差，用于判断趋势
    // 添加超时机制
    uint32_t start_time = HAL_GetTick();
    const uint32_t max_runtime = 5000; // 增加最大运行时间
    while (stable_count < required_stable_count) {
        // 检查是否超时
        if (HAL_GetTick() - start_time > max_runtime) {
            break; // 超时退出
        }
        float x_output = 0, y_output = 0;
        bool is_stable = true;
        // 处理 X 轴
        if (g_laser_point.x != 0) {
            float x_error = target_point.x - g_laser_point.x;
            float abs_x_error = fabs(x_error);
            
            // 根据误差大小动态调整PID参数和输出限制 - 更温和的调整
            if (abs_x_error > coarse_threshold) {
                // 粗调模式：距离目标较远时使用中等步长
                PID_SetParam(&pid_x, 1.0f, 0.015f, 0.35f);
                PID_SetOutputLimit(&pid_x, -6.0f, 6.0f);
            } else if (abs_x_error > 3) {
                // 中等调整模式
                PID_SetParam(&pid_x, 0.6f, 0.01f, 0.25f);
                PID_SetOutputLimit(&pid_x, -3.0f, 3.0f);
            } else {
                // 精调模式：接近目标时使用很小步长
                PID_SetParam(&pid_x, 0.3f, 0.005f, 0.15f);
                PID_SetOutputLimit(&pid_x, -1.0f, 1.0f);
                x_fine_tuning = 1;
            }
            
            // 计算PID输出值
            x_output = PID_Compute(&pid_x, g_laser_point.x);
            
            // 检查是否到达目标位置
            if (abs_x_error > pos_error) {
                is_stable = false;
                
                // 微调模式下使用更小的步长和额外的振荡抑制
                if (x_fine_tuning) {
                    // 检查误差趋势，防止振荡
                    if ((x_error > 0 && prev_x_error < 0) || (x_error < 0 && prev_x_error > 0)) {
                        x_output *= 0.3f; // 大幅减小输出，防止过冲
                    }
                    // 对于小误差使用固定微小步长
                    if (abs_x_error <= 2) {
                        x_output = (x_error > 0) ? 0.3f : -0.3f;
                    }
                }
                
                // 反向计算，因为舵机控制与坐标系的方向关系
                g_servox_duty += (int16_t)(-x_output);
                
                // 限制PWM范围
                if (g_servox_duty > SERVO_PWM_MAX) g_servox_duty = SERVO_PWM_MAX;
                if (g_servox_duty < SERVO_PWM_MIN) g_servox_duty = SERVO_PWM_MIN;
                
                // 设置舵机位置
                Servo_SetPulseWidth_DirX(g_servox_duty);
            }
            
            prev_x_error = x_error;
        }
        // 处理 Y 轴
        if (g_laser_point.y != 0) {
            float y_error = target_point.y - g_laser_point.y;
            float abs_y_error = fabs(y_error);
            
            // 根据误差大小动态调整PID参数和输出限制 - 更温和的调整
            if (abs_y_error > coarse_threshold) {
                // 粗调模式：距离目标较远时使用中等步长
                PID_SetParam(&pid_y, 1.0f, 0.015f, 0.35f);
                PID_SetOutputLimit(&pid_y, -6.0f, 6.0f);
            } else if (abs_y_error > 3) {
                // 中等调整模式
                PID_SetParam(&pid_y, 0.6f, 0.01f, 0.25f);
                PID_SetOutputLimit(&pid_y, -3.0f, 3.0f);
            } else {
                // 精调模式：接近目标时使用很小步长
                PID_SetParam(&pid_y, 0.3f, 0.005f, 0.15f);
                PID_SetOutputLimit(&pid_y, -1.0f, 1.0f);
                y_fine_tuning = 1;
            }
            
            // 计算PID输出值
            y_output = PID_Compute(&pid_y, g_laser_point.y);
            
            // 检查是否到达目标位置
            if (abs_y_error > pos_error) {
                is_stable = false;
                
                // 微调模式下使用更小的步长和额外的振荡抑制
                if (y_fine_tuning) {
                    // 检查误差趋势，防止振荡
                    if ((y_error > 0 && prev_y_error < 0) || (y_error < 0 && prev_y_error > 0)) {
                        y_output *= 0.3f; // 大幅减小输出，防止过冲
                    }
                    // 对于小误差使用固定微小步长
                    if (abs_y_error <= 2) {
                        y_output = (y_error > 0) ? 0.3f : -0.3f;
                    }
                }
                
                // Y轴与X轴方向相反
                g_servoy_duty += (int16_t)(y_output);
                
                // 限制PWM范围
                if (g_servoy_duty > SERVO_PWM_MAX) g_servoy_duty = SERVO_PWM_MAX;
                if (g_servoy_duty < SERVO_PWM_MIN) g_servoy_duty = SERVO_PWM_MIN;
                
                // 设置舵机位置
                Servo_SetPulseWidth_DirY(g_servoy_duty);
            }
            
            prev_y_error = y_error;
        }
        // 检查稳定性
        if (is_stable && g_laser_point.x != 0 && g_laser_point.y != 0) {
            stable_count++;
        } else {
            stable_count = 0;
        }
        HAL_Delay(step_delay);
    }
}

/**
 * @brief 遍历矩形的一条边，生成直线上的点并使用PID追踪
 * @param start_point 起始点坐标
 * @param end_point 结束点坐标
 * @param num_points 要生成的中间点数量
 * @param point_delay 每个点的停留时间(ms)
 */
void TraverseRectangleEdge(LaserPoint_t start_point, LaserPoint_t end_point, uint8_t num_points, uint16_t point_delay)
{
    // 计算两点间的向量
    int16_t delta_x = end_point.x - start_point.x;
    int16_t delta_y = end_point.y - start_point.y;
    
    // 生成路径点并逐个追踪
    for (uint8_t i = 0; i <= num_points; i++) {
        // 使用线性插值计算当前点坐标
        float t = (float)i / (float)num_points; // 插值参数，从0到1
        
        LaserPoint_t current_target;
        // 修复类型转换问题：先计算浮点数结果，再转换为整数
        float target_x_float = start_point.x + delta_x * t;
        float target_y_float = start_point.y + delta_y * t;
        
        // 确保结果在有效范围内并转换为uint8_t
        current_target.x = (uint8_t)(target_x_float < 0 ? 0 : (target_x_float > 255 ? 255 : target_x_float));
        current_target.y = (uint8_t)(target_y_float < 0 ? 0 : (target_y_float > 255 ? 255 : target_y_float));
        
        // 使用PID控制器追踪到当前目标点
        TrackPixelPointFast(current_target);
        
        // 在目标点停留一段时间
        HAL_Delay(point_delay);
    }
}

/**
 * @brief 快速版本的PID像素点追踪函数（用于直线遍历）
 * @param target_point 目标坐标点结构体
 * @note 优化了参数以适应连续点追踪，减少了稳定等待时间
 */
void TrackPixelPointFast(LaserPoint_t target_point)
{
    // PID控制器定义
    PidController_t pid_x, pid_y;
    const uint8_t pos_error = 1; // 位置误差容忍度
    const uint16_t step_delay = 40; // 增加延时，减少控制频率
    const uint8_t coarse_threshold = 6; // 降低粗调阈值
    
    // 初始化X轴PID控制器 - 减小参数避免振荡
    PID_Init(&pid_x, PID_TYPE_POSITIONAL);
    PID_SetParam(&pid_x, 0.8f, 0.01f, 0.2f); // 降低P值和D值
    PID_SetOutputLimit(&pid_x, -4.0f, 4.0f); // 大幅减小输出范围
    PID_SetIntegralLimit(&pid_x, -8.0f, 8.0f);
    PID_SetTarget(&pid_x, target_point.x);
    
    // 初始化Y轴PID控制器 - 减小参数避免振荡
    PID_Init(&pid_y, PID_TYPE_POSITIONAL);
    PID_SetParam(&pid_y, 0.8f, 0.01f, 0.2f); // 降低P值和D值
    PID_SetOutputLimit(&pid_y, -4.0f, 4.0f); // 大幅减小输出范围
    PID_SetIntegralLimit(&pid_y, -8.0f, 8.0f);
    PID_SetTarget(&pid_y, target_point.y);
    
    // 快速收敛控制循环
    uint8_t stable_count = 0;
    const uint8_t required_stable_count = 5; // 增加稳定次数要求
    uint32_t start_time = HAL_GetTick();
    const uint32_t max_runtime = 3000; // 增加运行时间容忍度
    
    while (stable_count < required_stable_count) {
        // 检查是否超时
        if (HAL_GetTick() - start_time > max_runtime) {
            break;
        }
        
        bool is_stable = true;
        
        // 处理 X 轴
        if (g_laser_point.x != 0) {
            float x_error = target_point.x - g_laser_point.x;
            float abs_x_error = fabs(x_error);
            
            // 根据误差大小动态调整PID参数 - 更保守的参数
            if (abs_x_error > coarse_threshold) {
                PID_SetParam(&pid_x, 1.0f, 0.015f, 0.25f); // 减小所有参数
                PID_SetOutputLimit(&pid_x, -5.0f, 5.0f);
            } else if (abs_x_error > 2) {
                PID_SetParam(&pid_x, 0.6f, 0.01f, 0.15f);
                PID_SetOutputLimit(&pid_x, -3.0f, 3.0f);
            } else {
                PID_SetParam(&pid_x, 0.4f, 0.005f, 0.1f); // 极小的参数用于精调
                PID_SetOutputLimit(&pid_x, -1.5f, 1.5f);
            }
            
            // 计算PID输出并应用
            if (abs_x_error > pos_error) {
                is_stable = false;
                float x_output = PID_Compute(&pid_x, g_laser_point.x);
                
                // 更新舵机PWM值
                g_servox_duty += (int16_t)(-x_output);
                
                // 限制PWM范围
                if (g_servox_duty > SERVO_PWM_MAX) g_servox_duty = SERVO_PWM_MAX;
                if (g_servox_duty < SERVO_PWM_MIN) g_servox_duty = SERVO_PWM_MIN;
                
                // 设置舵机位置
                Servo_SetPulseWidth_DirX(g_servox_duty);
            }
        }
        
        // 处理 Y 轴
        if (g_laser_point.y != 0) {
            float y_error = target_point.y - g_laser_point.y;
            float abs_y_error = fabs(y_error);
            
            // 根据误差大小动态调整PID参数 - 更保守的参数
            if (abs_y_error > coarse_threshold) {
                PID_SetParam(&pid_y, 1.0f, 0.015f, 0.25f); // 减小所有参数
                PID_SetOutputLimit(&pid_y, -5.0f, 5.0f);
            } else if (abs_y_error > 2) {
                PID_SetParam(&pid_y, 0.6f, 0.01f, 0.15f);
                PID_SetOutputLimit(&pid_y, -3.0f, 3.0f);
            } else {
                PID_SetParam(&pid_y, 0.4f, 0.005f, 0.1f); // 极小的参数用于精调
                PID_SetOutputLimit(&pid_y, -1.5f, 1.5f);
            }
            
            // 计算PID输出并应用
            if (abs_y_error > pos_error) {
                is_stable = false;
                float y_output = PID_Compute(&pid_y, g_laser_point.y);
                
                // 更新舵机PWM值
                g_servoy_duty += (int16_t)(y_output);
                
                // 限制PWM范围
                if (g_servoy_duty > SERVO_PWM_MAX) g_servoy_duty = SERVO_PWM_MAX;
                if (g_servoy_duty < SERVO_PWM_MIN) g_servoy_duty = SERVO_PWM_MIN;
                
                // 设置舵机位置
                Servo_SetPulseWidth_DirY(g_servoy_duty);
            }
        }
        
        // 检查稳定性
        if (is_stable && g_laser_point.x != 0 && g_laser_point.y != 0) {
            stable_count++;
        } else {
            stable_count = 0;
        }
        
        HAL_Delay(step_delay);
    }
}

/**
 * @brief 计算两点间的欧几里得距离
 * @param p1 第一个点
 * @param p2 第二个点
 * @return 两点间的距离
 */
float CalculateDistance(LaserPoint_t p1, LaserPoint_t p2)
{
    float dx = p2.x - p1.x;
    float dy = p2.y - p1.y;
    return sqrt(dx * dx + dy * dy);
}

/**
 * @brief 高精度遍历矩形边（自适应点数版本）
 * @param start_point 起始点坐标
 * @param end_point 结束点坐标
 * @param points_per_unit 每个像素单位的点数密度
 * @param point_delay 每个点的停留时间(ms)
 */
void TraverseRectangleEdgeAdaptive(LaserPoint_t start_point, LaserPoint_t end_point, float points_per_unit, uint16_t point_delay)
{
    // 计算边长并确定点数
    float edge_length = CalculateDistance(start_point, end_point);
    uint8_t num_points = (uint8_t)(edge_length * points_per_unit);
    
    // 限制点数范围
    if (num_points < 5) num_points = 5;     // 最少5个点
    if (num_points > 20) num_points = 20;   // 最多20个点
    
    // 调用标准遍历函数
    TraverseRectangleEdge(start_point, end_point, num_points, point_delay);
}

/**
 * @brief 调试函数：验证直线插值算法的正确性
 * @param start_point 起始点坐标
 * @param end_point 结束点坐标
 * @param num_points 要生成的中间点数量
 * @note 此函数仅用于调试，会通过某种方式输出计算结果（比如通过UART或调试器）
 */
void DebugTraverseRectangleEdge(LaserPoint_t start_point, LaserPoint_t end_point, uint8_t num_points)
{
    // 计算两点间的向量
    int16_t delta_x = end_point.x - start_point.x;
    int16_t delta_y = end_point.y - start_point.y;
    
    // 输出调试信息（您可以根据实际情况修改输出方式）
    // printf("Debug: Start(%d,%d) -> End(%d,%d), Delta(%d,%d)\n", 
    //        start_point.x, start_point.y, end_point.x, end_point.y, delta_x, delta_y);
    
    // 生成并输出所有插值点
    for (uint8_t i = 0; i <= num_points; i++) {
        // 使用线性插值计算当前点坐标
        float t = (float)i / (float)num_points;
        
        // 计算插值点
        float target_x_float = start_point.x + delta_x * t;
        float target_y_float = start_point.y + delta_y * t;
        
        // 转换为最终坐标
        uint8_t target_x = (uint8_t)(target_x_float < 0 ? 0 : (target_x_float > 255 ? 255 : target_x_float));
        uint8_t target_y = (uint8_t)(target_y_float < 0 ? 0 : (target_y_float > 255 ? 255 : target_y_float));
        
        // 输出调试信息（您可以根据实际情况修改输出方式）
        // printf("Point %d: t=%.3f, Float(%.1f,%.1f) -> Final(%d,%d)\n", 
        //        i, t, target_x_float, target_y_float, target_x, target_y);
    }
}

