#include "include.h"
#include "ultrasonic.h"
#include "usart.h"


void Ultrasonic_Init()
{
  Uart5_Init(9600);			//串口5初始化，函数参数为波特率
	
}

s8 ultra_start_f;

void Ultra_Duty()
{
	u8 temp[3];

	ultra.h_dt = 0.05f; //50ms一次

	temp[0] = 0x55;
	Uart5_Send(temp ,1);

	ultra_start_f = 1;

	if(ultra.measure_ot_cnt<200) //200ms
	{
		ultra.measure_ot_cnt += ultra.h_dt *1000;
	}
	else
	{
		ultra.measure_ok = 0;//超时，复位
	}
}

u16 ultra_distance_old;

_height_st ultra;

void Ultra_Get(u8 com_data)
{
	static u8 ultra_tmp;
	
	if( ultra_start_f == 1 )
	{
		ultra_tmp = com_data;
		ultra_start_f = 2;
	}
	else if( ultra_start_f == 2 )
	{
		ultra.height =  ((ultra_tmp<<8) + com_data)/10;
		
		if(ultra.height < 500) // 5米范围内认为有效，跳变值约10米.
		{
			ultra.relative_height = ultra.height;
			ultra.measure_ok = 1;
		}
		else
		{
			ultra.measure_ok = 2; //数据超范围
		}

		ultra_start_f = 0;
	}
	ultra.measure_ot_cnt = 0; //清除超时计数（喂狗）
	
	ultra.h_delta = ultra.relative_height - ultra_distance_old;
	
	ultra_distance_old = ultra.relative_height;
	
}

