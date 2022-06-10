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
 
	#if defined(USE_KS103)
		temp[0] = 0xe8;
		temp[1] = 0x02;
		temp[2] = 0xbc;
		Uart5_Send(temp ,3);
	#elif defined(USE_US100)
		temp[0] = 0x55;
		Uart5_Send(temp ,1);
		//Usart1_Send(temp ,1);
	#endif
///////////////////////////////////////////////
		ultra_start_f = 1;//开始测量高度

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
	//启动测高，开始读取数据
	if(ultra_start_f ==1)
	{
		ultra_tmp=com_data;
		ultra_start_f = 2;//读取数据完成标志
	}
	else if( ultra_start_f == 2 )//开始计算高度
	{
		ultra.height =  ((ultra_tmp<<8)+com_data)/10;
		if(ultra.height<500)//5米范围内认为有效，跳变值约10米.
		{
			ultra.relative_height = ultra.height;
			ultra.measure_ok = 1;//数据有效的标志
		}
		else
		{
			ultra.measure_ok = 2; //数据超范围，数据无效的标志
		}
		ultra_start_f = 0;      //完成一次测高数据的读取和赋值，测高标志清零
	}
	ultra.measure_ot_cnt = 0; //清除超时计数（喂狗）
	
	ultra.h_delta = ultra.relative_height - ultra_distance_old;//计算前后两次的高度数据之差
	
	ultra_distance_old = ultra.relative_height;
	
}

