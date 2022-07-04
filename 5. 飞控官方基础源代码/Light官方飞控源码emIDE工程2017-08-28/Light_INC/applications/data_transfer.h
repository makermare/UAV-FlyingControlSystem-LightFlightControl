/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved.
 + 文件名  ：data_transfer.h
 + 描述    ：数据传输头文件
代码尽量做到逐行注释 代码有问题，及时加群交流 作者有偿支持对开源代码的完善 */
#ifndef _DATA_TRANSFER_H
#define	_DATA_TRANSFER_H

#include "stm32f4xx.h"
#include "height_ctrl.h"

/* 等待发送数据的标志 */
extern u8 wait_for_translate;
/* 等待发送数据的标志结构体 */
typedef struct
{
    u8 send_status;
    u8 send_speed;
    u8 send_rcdata;
    u8 send_motopwm;
    u8 send_senser;
    u8 send_senser2;
    u8 send_location;
    u8 send_power;
    u8 send_pid1;
    u8 send_pid2;
    u8 send_pid3;
    u8 send_pid4;
    u8 send_user;
} dt_flag_t;

/*----------------------------------------------------------
 + 实现功能：由任务调度调用周期1ms
----------------------------------------------------------*/
extern void Call_Data_transfer(void);

/*----------------------------------------------------------
 + 实现功能：数传初始化
----------------------------------------------------------*/
extern void Data_transfer_init(void);

/*----------------------------------------------------------
 + 实现功能：数据接收并保存
 + 调用参数：接收到的单字节数据
----------------------------------------------------------*/
extern void DT_Data_Receive_Prepare(u8 data);

#endif
/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved. */
