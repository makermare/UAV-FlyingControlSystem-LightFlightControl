/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved.
 + 文件名  driver_pwm_in.c
 + 描述    ：PWM输入捕获驱动
代码尽量做到逐行注释 代码有问题，及时加群交流 作者有偿支持对开源代码的完善 */
#include "stm32f4xx.h"
#include "rc.h"

/* 8通道输入PWM数据的数组1000-2000微秒的高电平信号 */
u16 Rc_Pwm_In[8];

/*----------------------------------------------------------
 + 实现功能：PWM输入信号捕获
 + 保留函数名，由中断调用
----------------------------------------------------------*/
void TIM3_IRQHandler(void)
{
    /* PWM输入保存数据 */
    static u16 temp_cnt1,temp_cnt1_2,temp_cnt2,temp_cnt2_2,temp_cnt3,temp_cnt3_2,temp_cnt4,temp_cnt4_2;

    /* 记录RadioControl类型为PWM输入方式 */
    Call_RadioControl_Sign(1);

    if(TIM3->SR & TIM_IT_CC1)
    {
        TIM3->SR = ~TIM_IT_CC1;
        TIM3->SR = ~TIM_FLAG_CC1OF;
        if(GPIOC->IDR & GPIO_Pin_6)
        {
            temp_cnt1 = TIM_GetCapture1(TIM3);
        }
        else
        {
            temp_cnt1_2 = TIM_GetCapture1(TIM3);
            if(temp_cnt1_2>=temp_cnt1)
                Rc_Pwm_In[0] = temp_cnt1_2-temp_cnt1;
            else
                Rc_Pwm_In[0] = 0xffff-temp_cnt1+temp_cnt1_2+1;
        }
    }
    if(TIM3->SR & TIM_IT_CC2)
    {
        TIM3->SR = ~TIM_IT_CC2;
        TIM3->SR = ~TIM_FLAG_CC2OF;
        if(GPIOC->IDR & GPIO_Pin_7)
        {
            temp_cnt2 = TIM_GetCapture2(TIM3);
        }
        else
        {
            temp_cnt2_2 = TIM_GetCapture2(TIM3);
            if(temp_cnt2_2>=temp_cnt2)
                Rc_Pwm_In[1] = temp_cnt2_2-temp_cnt2;
            else
                Rc_Pwm_In[1] = 0xffff-temp_cnt2+temp_cnt2_2+1;
        }
    }
    if(TIM3->SR & TIM_IT_CC3)
    {
        TIM3->SR = ~TIM_IT_CC3;
        TIM3->SR = ~TIM_FLAG_CC3OF;
        if(GPIOB->IDR & GPIO_Pin_0)
        {
            temp_cnt3 = TIM_GetCapture3(TIM3);
        }
        else
        {
            temp_cnt3_2 = TIM_GetCapture3(TIM3);
            if(temp_cnt3_2>=temp_cnt3)
                Rc_Pwm_In[2] = temp_cnt3_2-temp_cnt3;
            else
                Rc_Pwm_In[2] = 0xffff-temp_cnt3+temp_cnt3_2+1;
        }
    }
    if(TIM3->SR & TIM_IT_CC4)
    {
        TIM3->SR = ~TIM_IT_CC4;
        TIM3->SR = ~TIM_FLAG_CC4OF;
        if(GPIOB->IDR & GPIO_Pin_1)
        {
            temp_cnt4 = TIM_GetCapture4(TIM3);
        }
        else
        {
            temp_cnt4_2 = TIM_GetCapture4(TIM3);
            if(temp_cnt4_2>=temp_cnt4)
                Rc_Pwm_In[3] = temp_cnt4_2-temp_cnt4;
            else
                Rc_Pwm_In[3] = 0xffff-temp_cnt4+temp_cnt4_2+1;
        }
    }
}

/*----------------------------------------------------------
 + 实现功能：PWM输入信号捕获
 + 保留函数名，由中断调用
----------------------------------------------------------*/
void TIM4_IRQHandler(void)
{
    /* PWM输入保存数据 */
    static u16 temp_cnt1,temp_cnt1_2,temp_cnt2,temp_cnt2_2,temp_cnt3,temp_cnt3_2,temp_cnt4,temp_cnt4_2;

    /* 记录RadioControl类型为PWM输入方式 */
    Call_RadioControl_Sign(1);

    if(TIM4->SR & TIM_IT_CC1)
    {
        TIM4->SR = ~TIM_IT_CC1;
        TIM4->SR = ~TIM_FLAG_CC1OF;
        if(GPIOD->IDR & GPIO_Pin_12)
        {
            temp_cnt1 = TIM_GetCapture1(TIM4);
        }
        else
        {
            temp_cnt1_2 = TIM_GetCapture1(TIM4);
            if(temp_cnt1_2>=temp_cnt1)
                Rc_Pwm_In[4] = temp_cnt1_2-temp_cnt1;
            else
                Rc_Pwm_In[4] = 0xffff-temp_cnt1+temp_cnt1_2+1;
        }
    }
    if(TIM4->SR & TIM_IT_CC2)
    {
        TIM4->SR = ~TIM_IT_CC2;
        TIM4->SR = ~TIM_FLAG_CC2OF;
        if(GPIOD->IDR & GPIO_Pin_13)
        {
            temp_cnt2 = TIM_GetCapture2(TIM4);
        }
        else
        {
            temp_cnt2_2 = TIM_GetCapture2(TIM4);
            if(temp_cnt2_2>=temp_cnt2)
                Rc_Pwm_In[5] = temp_cnt2_2-temp_cnt2;
            else
                Rc_Pwm_In[5] = 0xffff-temp_cnt2+temp_cnt2_2+1;
        }
    }
    if(TIM4->SR & TIM_IT_CC3)
    {
        TIM4->SR = ~TIM_IT_CC3;
        TIM4->SR = ~TIM_FLAG_CC3OF;
        if(GPIOD->IDR & GPIO_Pin_14)
        {
            temp_cnt3 = TIM_GetCapture3(TIM4);
        }
        else
        {
            temp_cnt3_2 = TIM_GetCapture3(TIM4);
            if(temp_cnt3_2>=temp_cnt3)
                Rc_Pwm_In[6] = temp_cnt3_2-temp_cnt3;
            else
                Rc_Pwm_In[6] = 0xffff-temp_cnt3+temp_cnt3_2+1;
        }
    }
    if(TIM4->SR & TIM_IT_CC4)
    {
        TIM4->SR = ~TIM_IT_CC4;
        TIM4->SR = ~TIM_FLAG_CC4OF;
        if(GPIOD->IDR & GPIO_Pin_15)
        {
            temp_cnt4 = TIM_GetCapture4(TIM4);
        }
        else
        {
            temp_cnt4_2 = TIM_GetCapture4(TIM4);
            if(temp_cnt4_2>=temp_cnt4)
                Rc_Pwm_In[7] = temp_cnt4_2-temp_cnt4;
            else
                Rc_Pwm_In[7] = 0xffff-temp_cnt4+temp_cnt4_2+1;
        }
    }
}

/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved. */
