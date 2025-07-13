# STM32F407 简单任务调度器使用说明

## 概述
本项目实现了一个轻量级的协作式任务调度器，适用于STM32F407等资源受限的嵌入式系统。

## 文件结构
```
BSP/COMMON/
├── task_scheduler.h    # 任务调度器头文件
├── task_scheduler.c    # 任务调度器实现
├── app_tasks.h         # 应用任务头文件
├── app_tasks.c         # 应用任务实现
└── user_init.c         # 用户初始化（已修改）
```

## 特性
- **协作式调度**：任务主动让出CPU控制权
- **优先级调度**：支持5个优先级等级
- **周期性执行**：每个任务可设置独立的执行周期
- **轻量级设计**：内存占用小，适合资源受限系统
- **易于扩展**：可以轻松添加新任务

## 任务优先级
```c
typedef enum {
    TASK_PRIORITY_IDLE = 0,      // 空闲任务
    TASK_PRIORITY_LOW,           // 低优先级
    TASK_PRIORITY_NORMAL,        // 正常优先级
    TASK_PRIORITY_HIGH,          // 高优先级
    TASK_PRIORITY_CRITICAL       // 关键任务
} TaskPriority_t;
```

## 任务状态
```c
typedef enum {
    TASK_READY = 0,      // 就绪状态
    TASK_RUNNING,        // 运行状态
    TASK_BLOCKED,        // 阻塞状态
    TASK_SUSPENDED       // 挂起状态
} TaskState_t;
```

## 主要API
### 初始化
```c
HAL_StatusTypeDef TaskScheduler_Init(void);
```

### 添加任务
```c
HAL_StatusTypeDef TaskScheduler_AddTask(
    TaskFunction_t function,    // 任务函数指针
    uint32_t period,           // 执行周期(ms)
    TaskPriority_t priority,   // 任务优先级
    const char* name           // 任务名称
);
```

### 运行调度器
```c
void TaskScheduler_Run(void);  // 在主循环中调用
```

### 任务管理
```c
void TaskScheduler_SuspendTask(const char* taskName);  // 挂起任务
void TaskScheduler_ResumeTask(const char* taskName);   // 恢复任务
void TaskScheduler_DeleteTask(const char* taskName);   // 删除任务
```

### 工具函数
```c
uint32_t TaskScheduler_GetSystemTick(void);    // 获取系统时钟
uint8_t TaskScheduler_GetTaskCount(void);      // 获取任务数量
void TaskScheduler_PrintTaskInfo(void);        // 打印任务信息
```

## 示例任务
项目包含以下示例任务：

1. **UART处理任务** (10ms周期，高优先级)
2. **LED闪烁任务** (1000ms周期，低优先级)
3. **系统监控任务** (5000ms周期，正常优先级)
4. **心跳任务** (2000ms周期，正常优先级)
5. **看门狗任务** (100ms周期，关键优先级)
6. **空闲任务** (50ms周期，空闲优先级)

## 使用步骤

### 1. 创建任务函数
```c
void MyTask(void)
{
    // 任务代码
    // 注意：任务应该尽快完成，避免长时间阻塞
}
```

### 2. 在初始化中添加任务
```c
void User_Init(void)
{
    // 其他初始化代码...
    
    // 初始化任务调度器
    TaskScheduler_Init();
    
    // 添加自定义任务
    TaskScheduler_AddTask(MyTask, 100, TASK_PRIORITY_NORMAL, "MyTask");
}
```

### 3. 在主循环中运行调度器
```c
int main(void)
{
    // 系统初始化...
    User_Init();
    
    while (1)
    {
        TaskScheduler_Run();  // 运行任务调度器
    }
}
```

## 注意事项

1. **任务执行时间**：每个任务应该尽快完成，避免长时间阻塞其他任务
2. **周期设置**：根据任务实际需求合理设置执行周期
3. **优先级分配**：关键任务设置较高优先级
4. **中断安全**：在中断服务程序中不要直接操作任务调度器
5. **内存限制**：最大支持10个任务，可根据需要修改MAX_TASKS宏
6. **任务名称**：每个任务必须有唯一的名称

## 扩展建议

1. **任务统计**：可以添加任务执行时间统计功能
2. **任务间通信**：可以添加消息队列或信号量机制
3. **动态优先级**：可以实现动态优先级调整
4. **任务栈监控**：可以添加任务栈使用情况监控
5. **低功耗模式**：可以在空闲任务中实现更精细的低功耗管理

## 调试功能
使用 `TaskScheduler_PrintTaskInfo()` 可以输出所有任务的状态信息，方便调试。

## 性能考虑
- 调度器开销很小，主要在任务查找和优先级比较
- 建议任务执行时间控制在几毫秒以内
- 合理设置任务周期，避免CPU资源浪费
