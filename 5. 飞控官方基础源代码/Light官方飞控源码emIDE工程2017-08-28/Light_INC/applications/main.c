/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved.
 + 文件名  ：main.c
 + 描述    ：主循环
代码尽量做到逐行注释 代码有问题，及时加群交流 作者有偿支持对开源代码的完善 */
#include "init.h"
#include "scheduler.h"

/*----------------------------------------------------------
 + 实现功能：主函数
----------------------------------------------------------*/
void main(void)
{
    /* 飞控初始化 */
    Light_Init();
    /* 主循环 */
    while(1)Main_Loop();
}
/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved. */
