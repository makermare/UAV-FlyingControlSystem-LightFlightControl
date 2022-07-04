#include "led.h"
#include "include.h"
//#include "ll_gps.h"
#include "mymath.h"

void LED_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(LIGHT_RCC_LED,ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_Pin   = LIGHT_Pin_LED1| LIGHT_Pin_LED2| LIGHT_Pin_LED3| LIGHT_Pin_LED4;
	GPIO_Init(LIGHT_GPIO_LED, &GPIO_InitStructure);
	
	GPIO_SetBits(LIGHT_GPIO_LED, LIGHT_Pin_LED1);		
	GPIO_SetBits(LIGHT_GPIO_LED, LIGHT_Pin_LED2);		
	GPIO_SetBits(LIGHT_GPIO_LED, LIGHT_Pin_LED3);		
	GPIO_SetBits(LIGHT_GPIO_LED, LIGHT_Pin_LED4);		
	
// 	GPIO_ResetBits(LIGHT_GPIO_LED, LIGHT_Pin_LED1);		
// 	GPIO_ResetBits(LIGHT_GPIO_LED, LIGHT_Pin_LED2);		
// 	GPIO_ResetBits(LIGHT_GPIO_LED, LIGHT_Pin_LED3);		
// 	GPIO_ResetBits(LIGHT_GPIO_LED, LIGHT_Pin_LED4);	


}
void double_flash()//delay__
{
			LED1_ON;
			LED2_ON;
			LED3_OFF;
			LED4_ON;
			Delay_ms(50);
			LED1_OFF;
			LED2_OFF;
			LED3_OFF;
			LED4_OFF;
			Delay_ms(100);	
			LED1_ON;
			LED2_OFF;
			LED3_ON;
			LED4_ON;
			Delay_ms(50);		
			LED1_OFF;
			LED2_OFF;
			LED3_OFF;
			LED4_OFF;
			Delay_ms(600);

}

void aircraft_mode_led(u8 maxmotors)
{
	switch(maxmotors)
	{
		case 4:

		double_flash();
		double_flash();
		
		break;
		
		case 6:
		double_flash();			
		double_flash();
		double_flash();		

		case 8:
		double_flash();			
		double_flash();
		double_flash();			
		double_flash();			
		
		break;
		
		default:
			
		break;
	}

}

u16 led_accuracy = 20;//该时间应与LED_Duty()调用周期相同
float LED_Brightness[4] = {0,20,0,0}; //TO 20 //XBRG

void LED_1ms_DRV( ) //0~20
{
	static u16 led_cnt[4];

	u8 i;
	
	for(i=0;i<4;i++)
	{
			
		if( led_cnt[i] < LED_Brightness[i] )
		{
			switch(i)
			{
				case 0:	
					LED1_ON;
				break;
				case 1:	
					LED2_ON;
				break;
				case 2:	
					LED3_ON;
				break;
				case 3:	
					LED4_ON;
				break;
			}
		}
		else
		{
			switch(i)
			{
				case 0:	
					LED1_OFF;
				break;
				case 1:	
					LED2_OFF;
				break;
				case 2:	
					LED3_OFF;
				break;
				case 3:	
					LED4_OFF;
				break;
			}
		}
		
		if(++led_cnt[i]>=led_accuracy)
		{
			led_cnt[i] = 0;
		}
	}
	

}

//void LED_RGB(u8 red,u8 green,u8 blue)//20,20,20
//{
//	LED_Brightness[1] = blue;
//	LED_Brightness[2] = red;
//	LED_Brightness[3] = green;

//}



u8 led_breath(float dT,u8 i,u16 T)// T,led编号,一次半程的时间，单位ms
{
	u8 f = 0;
	static u8 dir[LED_NUM];
	switch(dir[i])
	{
		case 0:
			LED_Brightness[i] += safe_div(led_accuracy,(T/(dT*1000)),0);
			if(LED_Brightness[i]>20)
			{
				dir[i] = 1;
			}
		
		break;
		case 1:
			LED_Brightness[i] -= safe_div(led_accuracy,(T/(dT*1000)),0);
			if(LED_Brightness[i]<0)
			{
				dir[i] = 0;
				f = 1;//流程已完成1次
			}
			
		break;
			
		default:
			dir[i] = 0;
			
		break;
		
	}
	return (f);
}	

static u16 ms_cnt[LED_NUM];
static u16 group_n_cnt[LED_NUM];
//亮-灭 为一组
//调用周期（s），LED编号, 亮度（20级），组数，亮时间(ms)，灭时间(ms)，组间隔 ,ms>led_accuracy;
u8 led_flash(float dT,u8 i,u8 lb,u16 group_n,u16 on_ms,u16 off_ms,u16 group_dT_ms)
{
	u8 f = 0;

	
	if(group_n_cnt[i] < group_n)   //组数没到
	{
		if(ms_cnt[i]<on_ms)
		{
			LED_Brightness[i] = lb;
		}
		else if(ms_cnt[i]<(on_ms+off_ms))
		{
			LED_Brightness[i] = 0;
		}
		if(ms_cnt[i]>=(on_ms+off_ms))
		{
			group_n_cnt[i] ++;
			ms_cnt[i] = 0;
		}
	}
	else						//进入组间隔
	{
		if(ms_cnt[i]<group_dT_ms)
		{
			LED_Brightness[i] = 0;
		}
		else
		{
			group_n_cnt[i] = 0;
			ms_cnt[i] = 0;
			f = 1; //流程完成1次
		}
	}
	
	ms_cnt[i] += (dT*1000);        //计时
	return (f); //0，未完成，1完成
}

#include "rc.h"
#include "ak8975.h"
#include "fly_mode.h"

void led_cnt_restar() //灯光驱动计数器复位
{

			for(u8 i =0;i<LED_NUM;i++)
			{
				LED_Brightness[i] = 0;
				ms_cnt[i] = 0;
				group_n_cnt[i] = 0;
			}

}

LED_state light;

void led_cnt_res_check()
{
		if(light.RGB_Info_old != light.RGB_Info)
		{
			led_cnt_restar();
			light.RGB_Info_old = light.RGB_Info;		
		}

}


extern u8 height_ctrl_mode;


//u8 LED_status[2];  //  0:old;  1:now


void LED_Duty() //50ms一次
{
	
	led_cnt_res_check();
	
//	if(Mag_CALIBRATED)
//	{
//		LED_status[1] = 1;
//	}
//	else if(mode_value[BACK_HOME]==1)
//	{
//		LED_status[1] = 4; //返航
//	}
//	else if(height_ctrl_mode==1)
//	{
//		LED_status[1] = 2;
//	}
//	else if(height_ctrl_mode==2)
//	{
//		LED_status[1] = 3;
//	}
//	else if(height_ctrl_mode==0)
//	{
//		LED_status[1] = 0;
//	}
	if(Mag_CALIBRATED) //传感器校准指示优先
	{
		light.RGB_Info = 19;
	}
	else if(mode_state == 0) //手动油门
	{
		if(!fly_ready)//没解锁
		{
			light.RGB_Info = 9; 
		}
		else
		{
			light.RGB_Info = 23; 
		}
	}
	else if(mode_state ==1) //气压定高
	{
		if(!fly_ready)//没解锁
		{
			light.RGB_Info = 24;
		} 
		else     //解锁
		{
			light.RGB_Info = 25;
		}
	}

	else if(mode_state ==2)//超声波
	{
		if(!fly_ready)
		{
			light.RGB_Info = 26;
		}
		else
		{
			light.RGB_Info = 27;
		}
	}
	
	
	
	static u8 step;
	switch(light.RGB_Info)
	{
		case 0:
			
		break;
		case 1:
			
		break;
		case 9:
			LED_Brightness[X] = 0;
			led_breath(0.05f,R,1000);
			led_breath(0.05f,G,1000);
			led_breath(0.05f,B,1000);
		break;
		case 10:
			LED_Brightness[X] = 0;
			LED_Brightness[R] = 0;		
			led_breath(0.05f,G,1000);
			LED_Brightness[B] = 0;
		break;
		case 11:
			LED_Brightness[X] = 0;
			if(step)
			{
				step += led_breath(0.05f,G,1000);
				LED_Brightness[R] = 0;
			}
			else
			{
				step += led_breath(0.05f,R,1000);
				LED_Brightness[G] = 0;
			}
			LED_Brightness[B] = 0;
			
			if(step>1)
			{
				step = 0;
			}				
		break;
		case 12:
			LED_Brightness[X] = 0;
			if(step)
			{
				step += led_breath(0.05f,G,1000);
				LED_Brightness[R] = 0;
			}
			else
			{
								led_breath(0.05f,R,1000);
				step += led_breath(0.05f,G,1000);
			}
			LED_Brightness[B] = 0;
			
			if(step>1)
			{
				step = 0;
			}				
		break;
		case 13: //双闪白白
			LED_Brightness[X] = 0;
			led_flash(0.05f,R,16,2,100,100,500);
			led_flash(0.05f,G,16,2,100,100,500);
			led_flash(0.05f,B,16,2,100,100,500);
		break;
		case 14: //双闪绿绿
			LED_Brightness[X] = 0;
			LED_Brightness[R] = 0;//led_flash(0.05f,R,2,100,100,100);
			led_flash(0.05f,G,16,2,100,100,500);
			LED_Brightness[B] = 0;//led_flash(0.05f,B,2,100,100,100);			
		break;
		case 15: //双闪绿红
			if(!step)
			{
				LED_Brightness[X] = 0;
				LED_Brightness[R] = 0;//led_flash(0.05f,R,2,100,100,100);
				step +=led_flash(0.05f,G,16,1,100,100,0);
				LED_Brightness[B] = 0;//led_flash(0.05f,B,2,100,100,100);		
			}
			else
			{
				LED_Brightness[X] = 0;
				LED_Brightness[G] = 0;//led_flash(0.05f,R,2,100,100,100);
				step +=led_flash(0.05f,R,16,1,100,100,500);
				LED_Brightness[B] = 0;//led_flash(0.05f,B,2,100,100,100);				
			}
			
			if(step>1)
			{
				step = 0;
			}
		break;	
		case 16: //双闪绿黄
			if(!step)
			{
				LED_Brightness[X] = 0;
				LED_Brightness[R] = 0;//led_flash(0.05f,R,2,100,100,100); 
				step +=led_flash(0.05f,G,16,1,100,100,0);
				LED_Brightness[B] = 0;//led_flash(0.05f,B,2,100,100,100);		
			}
			else
			{
				LED_Brightness[X] = 0;
							 led_flash(0.05f,R,16,1,100,100,500);
				step +=led_flash(0.05f,G,16,1,100,100,500);
				LED_Brightness[B] = 0;//led_flash(0.05f,B,2,100,100,100);	
			}
			
			if(step>1)
			{
				step = 0;
			}			
		break;	
		case 17://双闪蓝蓝
			LED_Brightness[X] = 0;
			LED_Brightness[R] = 0;//	led_flash(0.05f,R,2,100,100,100);
			LED_Brightness[G] = 0;//	led_flash(0.05f,G,2,100,100,100);
			led_flash(0.05f,B,16,2,100,100,500);	
		break;	
		case 18://三闪红红红
			LED_Brightness[X] = 0;
			led_flash(0.05f,R,16,3,100,100,500);
			LED_Brightness[G] = 0;//	led_flash(0.05f,G,2,100,100,100);
			LED_Brightness[B] = 0;//led_flash(0.05f,B,2,100,100,100);		
		break;
		case 19:
			LED_Brightness[X] = 0;
			LED_Brightness[R] = 0;//led_flash(0.05f,R,1,100,100,0);
			led_flash(0.05f,G,16,1,100,100,0);
			led_flash(0.05f,B,16,1,100,100,0);					
		break;
		
		case 23:
			LED_Brightness[X] = 0;
			LED_Brightness[R] = 0;
			LED_Brightness[G] = 20;
			LED_Brightness[B] = 0;
		
		break;
		
		case 24:
			LED_Brightness[X] = 0;
			led_breath(0.05f,R,1000);
			led_breath(0.05f,G,1000);
			LED_Brightness[B] = 0;//led_breath(0.05f,B,1000);
		
		break;		
		case 25:
			LED_Brightness[X] = 0;
			LED_Brightness[R] = 20;
			LED_Brightness[G] = 20;
			LED_Brightness[B] = 0;
		
		break;
		case 26:
			LED_Brightness[X] = 0;
			led_breath(0.05f,R,1000);
			LED_Brightness[G] = 0;//led_breath(0.05f,G,1000);
			led_breath(0.05f,B,1000);
		
		break;
		case 27:
			LED_Brightness[X] = 0;
			LED_Brightness[R] = 20;
			LED_Brightness[G] = 0;
			LED_Brightness[B] = 20;
		
		break;		
		default:break;
	}

			
}

void LED_MPU_Err(void)
{
	LED1_OFF;
	LED2_OFF;
	LED3_OFF;
	LED4_OFF;
	while(1)
	{
		LED1_ON;
		Delay_ms(150);
		LED1_OFF;
		Delay_ms(200);
		
	}
}

void LED_Mag_Err(void)
{
	LED1_OFF;
	LED2_OFF;
	LED3_OFF;
	LED4_OFF;
	while(1)
	{
		LED1_ON;
		Delay_ms(150);
		LED1_OFF;
		Delay_ms(150);
		LED1_ON;
		Delay_ms(150);
		LED1_OFF;
		Delay_ms(600);
	}
}

void LED_MS5611_Err(void)
{
	LED1_OFF;
	LED2_OFF;
	LED3_OFF;
	LED4_OFF;
	while(1)
	{
		LED1_ON;
		Delay_ms(150);
		LED1_OFF;
		Delay_ms(150);
		
		LED1_ON;
		Delay_ms(150);
		LED1_OFF;
		Delay_ms(150);
		
		LED1_ON;
		Delay_ms(150);
		LED1_OFF;
		Delay_ms(600);
	}

}



