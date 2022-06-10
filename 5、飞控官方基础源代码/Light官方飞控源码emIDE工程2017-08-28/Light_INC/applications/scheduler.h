/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved.
 + 文件名  ：scheduler.h
 + 描述    ：任务调度头文件
代码尽量做到逐行注释 代码有问题，及时加群交流 作者有偿支持对开源代码的完善 */
#ifndef _SCHEDULER_H_
#define _SCHEDULER_H_

#include "stm32f4xx.h"

/* 循环计数结构体 */
typedef struct
{
    /* 循环运行完毕标志 */
    u8 check_flag;
    /* 代码在预定周期内没有运行完错误计数 */
    u8 err_flag;
    /* 不同周期的执行任务独立计时 */
    s16 cnt_2ms;
    s16 cnt_5ms;
    s16 cnt_10ms;
    s16 cnt_20ms;
    s16 cnt_50ms;
    s16 cnt_100ms;
} loop_t;

/*----------------------------------------------------------
 + 实现功能：主循环 由主函数调用
----------------------------------------------------------*/
extern void Main_Loop(void);

/*----------------------------------------------------------
 + 实现功能：由Systick定时器调用 周期：1毫秒
----------------------------------------------------------*/
extern void Call_Loop_timer(void);

#endif
/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved. */
