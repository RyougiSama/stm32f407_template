#include "laser_shot_common.h"

#include "Emm_V5.h"

bool g_task_basic_q2_with_zdt_running = false;

void Task_BasicQ2_WithZDT_Start(void)
{
    g_task_basic_q2_with_zdt_running = true;
}

void Task_BasicQ2_WithZDT_Execute(void)
{
    if (!g_task_basic_q2_with_zdt_running) {
        return;
    }

    // 这里可以添加步进电机控制逻辑
    // 例如，使用 Emm_V5_Pos_Control 函数来控制步进电机位置
    // Emm_V5_Pos_Control(0, 0, 1000, 0, 32000, 0, 0);

    // 停止任务
    g_task_basic_q2_with_zdt_running = false;
}
