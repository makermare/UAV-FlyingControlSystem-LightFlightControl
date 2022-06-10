/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved.
 + 文件名  ：device_usart.h
 + 描述    ：串口设备头文件
代码尽量做到逐行注释 代码有问题，及时加群交流 作者有偿支持对开源代码的完善 */
#ifndef _DEVICE_USART_H
#define _DEVICE_USART_H
#include "stm32f4xx.h"

/*----------------------------------------------------------
 + 实现功能：串口1初始化
 + 调用参数功能：
 - u32 bound：波特率
 - u8 Priority：中断主优先级
 - u8 SubPriority：中断从优先级
 - FunctionalState TXenable：发送中断使能
 - FunctionalState RXenable：就收中断使能
----------------------------------------------------------*/
extern void Device_Usart1_ENABLE_Init(u32 bound,u8 Priority,u8 SubPriority,FunctionalState TXenable,FunctionalState RXenable);

/*----------------------------------------------------------
 + 实现功能：串口2初始化
 + 调用参数功能：
 - u32 bound：波特率
 - u8 Priority：中断主优先级
 - u8 SubPriority：中断从优先级
 - FunctionalState TXenable：发送中断使能
 - FunctionalState RXenable：就收中断使能
----------------------------------------------------------*/
extern void Device_Usart2_ENABLE_Init(u32 bound,u8 Priority,u8 SubPriority,FunctionalState TXenable,FunctionalState RXenable);

/*----------------------------------------------------------
 + 实现功能：串口3初始化
 + 调用参数功能：
 - u32 bound：波特率
 - u8 Priority：中断主优先级
 - u8 SubPriority：中断从优先级
 - FunctionalState TXenable：发送中断使能
 - FunctionalState RXenable：就收中断使能
----------------------------------------------------------*/
extern void Device_Usart3_ENABLE_Init(u32 bound,u8 Priority,u8 SubPriority,FunctionalState TXenable,FunctionalState RXenable);

/*----------------------------------------------------------
 + 实现功能：串口4初始化
 + 调用参数功能：
 - u32 bound：波特率
 - u8 Priority：中断主优先级
 - u8 SubPriority：中断从优先级
 - FunctionalState TXenable：发送中断使能
 - FunctionalState RXenable：就收中断使能
----------------------------------------------------------*/
extern void Device_Usart4_ENABLE_Init(u32 bound,u8 Priority,u8 SubPriority,FunctionalState TXenable,FunctionalState RXenable);

/*----------------------------------------------------------
 + 实现功能：串口5初始化
 + 调用参数功能：
 - u32 bound：波特率
 - u8 Priority：中断主优先级
 - u8 SubPriority：中断从优先级
 - FunctionalState TXenable：发送中断使能
 - FunctionalState RXenable：就收中断使能
----------------------------------------------------------*/
extern void Device_Usart5_ENABLE_Init(u32 bound,u8 Priority,u8 SubPriority,FunctionalState TXenable,FunctionalState RXenable);

#endif
/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved. */
