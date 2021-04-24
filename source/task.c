
#include <stdio.h>
#include <stdint.h>
#include "task.h"
#include "Q-Track2.h"
#include "key.h"

//任务控制块，这里作演示，随便写了几个任务
struct task_tcb task_list[TASKS_COUNT] =
{
//  任务开关    任务初始状态    任务开始时间    任务间隔时间        任务回调函数
    {TASK_ON,   TASK_WAIT,  TASK_CYCLE(10), TASK_CYCLE(2000),   task_led},      // led任务
    {TASK_ON,   TASK_WAIT,  TASK_CYCLE(20), TASK_CYCLE(10),     task_key},     // key任务
    {TASK_ON,   TASK_WAIT,  TASK_CYCLE(30), TASK_CYCLE(3000),   task_uart},     // 串口任务
};

/*********************************************************************//**
 * @brief
 * @param[in]
 * @return
 **********************************************************************/
void TaskPoll(void)
{
    uint8_t i = 0;
    for (i=0; i<TASKS_COUNT; i++) 
    {
        if( task_list[i].onoff == TASK_ON )
        {
            if (task_list[i].start_time > 0) 
             {
                task_list[i].start_time --;
                if (task_list[i].start_time <= 0)
                {
                    task_list[i].start_time = task_list[i].interval_time;
                    task_list[i].state = TASK_READY;
                }
            }
        }
    }
}

/*******************************************************//**
 * @brief
 * @param[in]
 * @return
 *******************************************************/
void TaskSchedule(void)
{
    uint8_t i;
    for (i=0; i<TASKS_COUNT; i++)
    {
        if (task_list[i].state == TASK_READY)
        {
            task_list[i].callback();
            task_list[i].state = TASK_WAIT;
        }
    }
}

/**
 * led任务
 */
void task_led(void)
{
    my_printf("run task_led()\n");
}

/**
 * 按键扫描任务
 */
void task_key(void)
{
	int key_value = Key_Driver();
	switch(key_value)
	{
		case KEY_SW2_VALUE :
			my_printf("sw2 press down\n");
		break;
		case KEY_SW2_VALUE | LONG_PRESS :
			my_printf("sw2 long press\n");
		break;
		case KEY_SW2_VALUE | DOUBLE_CLICK :
			my_printf("sw2 double press\n");
		break;
		default : break;
	}
}

/**
 * 串口
 */
void task_uart(void)
{
    my_printf("run task_uart()\n");
}
