/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved.
 + 文件名  driver_pwm_out.c
 + 描述    ：PWM输出驱动
代码尽量做到逐行注释 代码有问题，及时加群交流 作者有偿支持对开源代码的完善 */
#include "stm32f4xx.h"
#include "driver_pwm_out.h"
#include "mymath.h"
#include "database.h"

/*----------------------------------------------------------
 + PWM信号范围：
 - 飞控发给电调的最小油门行程信号 400HZ 1000us电平 40% 占空比
 - 飞控发给电调的最大油门行程信号 400HZ 2000us电平 80% 占空比
----------------------------------------------------------*/

/* 宏定义起始占空比40%=1000us */
#define INIT_DUTY 4000

/* 占空比对1000数据的比例 (8000 - 4000)/1000.0 */
#define PWM_RADIO 4

/* PWM的8个通道输出模式 */
int PWM_Mode = 0;

/* PWM的8个通道配对数组，修改可更换输出位置 */
u8 CH_out_Mapping[8] = {0,1,2,3,4,5,6,7};

/*----------------------------------------------------------
 + 实现功能：PWM的8个通道输出数据的调用
 + 调用参数功能：双字节整数数组：设置的数据(0-1000)
----------------------------------------------------------*/
void SetPwm(int16_t set_8pwm[8])
{
    /* 用于赋值并限幅的数组 */
    s16 pwm_tem[8];

    /* 正常输出PWM模式 */
    if(PWM_Mode==0)//
    {
        /* 为了防止程序出错,赋值并限幅 */
        for(u8 i=0; i<8; i++)
        {
            pwm_tem[i] = set_8pwm[i] ;
            pwm_tem[i] = LIMIT(pwm_tem[i],0,1000);
        }

        /* 配置数据按比例和起始值赋值寄存器 */
        TIM1->CCR4 = ( pwm_tem[CH_out_Mapping[0]] ) * PWM_RADIO + INIT_DUTY; //pwmout1
        TIM1->CCR3 = ( pwm_tem[CH_out_Mapping[1]] ) * PWM_RADIO + INIT_DUTY; //pwmout2
        TIM1->CCR2 = ( pwm_tem[CH_out_Mapping[2]] ) * PWM_RADIO + INIT_DUTY; //pwmout3
        TIM1->CCR1 = ( pwm_tem[CH_out_Mapping[3]] ) * PWM_RADIO + INIT_DUTY; //pwmout4
        TIM5->CCR4 = ( pwm_tem[CH_out_Mapping[4]] ) * PWM_RADIO + INIT_DUTY; //pwmout5
        TIM5->CCR3 = ( pwm_tem[CH_out_Mapping[5]] ) * PWM_RADIO + INIT_DUTY; //pwmout6
        TIM8->CCR4 = ( pwm_tem[CH_out_Mapping[6]] ) * PWM_RADIO + INIT_DUTY; //pwmout7
        TIM8->CCR3 = ( pwm_tem[CH_out_Mapping[7]] ) * PWM_RADIO + INIT_DUTY; //pwmout8
    }

    /* 输出PWM最小信号模式 */
    else if(PWM_Mode==1)
    {
        /* 配置数据按比例和起始值赋值寄存器 */
        TIM1->CCR4 = 0 * PWM_RADIO + INIT_DUTY; //pwmout1
        TIM1->CCR3 = 0 * PWM_RADIO + INIT_DUTY; //pwmout2
        TIM1->CCR2 = 0 * PWM_RADIO + INIT_DUTY; //pwmout3
        TIM1->CCR1 = 0 * PWM_RADIO + INIT_DUTY; //pwmout4
        TIM5->CCR4 = 0 * PWM_RADIO + INIT_DUTY; //pwmout5
        TIM5->CCR3 = 0 * PWM_RADIO + INIT_DUTY; //pwmout6
        TIM8->CCR4 = 0 * PWM_RADIO + INIT_DUTY; //pwmout7
        TIM8->CCR3 = 0 * PWM_RADIO + INIT_DUTY; //pwmout8
    }

    /* 输出PWM最大信号模式 */
    else if(PWM_Mode==2)
    {
        /* 配置数据按比例和起始值赋值寄存器 */
        TIM1->CCR4 = 1000 * PWM_RADIO + INIT_DUTY; //pwmout1
        TIM1->CCR3 = 1000 * PWM_RADIO + INIT_DUTY; //pwmout2
        TIM1->CCR2 = 1000 * PWM_RADIO + INIT_DUTY; //pwmout3
        TIM1->CCR1 = 1000 * PWM_RADIO + INIT_DUTY; //pwmout4
        TIM5->CCR4 = 1000 * PWM_RADIO + INIT_DUTY; //pwmout5
        TIM5->CCR3 = 1000 * PWM_RADIO + INIT_DUTY; //pwmout6
        TIM8->CCR4 = 1000 * PWM_RADIO + INIT_DUTY; //pwmout7
        TIM8->CCR3 = 1000 * PWM_RADIO + INIT_DUTY; //pwmout8
    }
}

/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved. */
