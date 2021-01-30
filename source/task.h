
#ifndef TASK_H
#define TASK_H

// 调度周期，此值要和TaskPoll函数的调用周期一致，单位ms
#define SCH_CYCLE   (1)

// 任务周期
#define TASK_CYCLE(x)   ( x>SCH_CYCLE?x/SCH_CYCLE:1 )

// 任务最大数量
#define TASKS_COUNT (3)

// 任务的开关
enum e_task_switch
{
    TASK_OFF,   // 任务关闭
    TASK_ON,    // 任务开启
};

// 任务的状态
enum e_task_state
{
    TASK_WAIT,  // 等待状态
    TASK_READY, // 就绪状态
};

//任务控制块
struct task_tcb
{
    enum e_task_switch onoff;   // 任务的开关
    enum e_task_state state;    // 任务的状态־
    uint16_t start_time;        // 任务的开始时间
    uint16_t interval_time;     // 任务的动行间隔
    void (*callback)(void);     // 任务的回调函数
};

void TaskPoll(void);
void TaskSchedule(void);

void task_led(void);
void task_key(void);
void task_uart(void);

#endif


