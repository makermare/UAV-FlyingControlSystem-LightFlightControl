#include "fly_mode.h"
#include "rc.h"
#include "PID.h"
u8 mode_value[10];
u8 mode_state,mode_state_old;
extern float  PixFlow_Roll;
extern float  PixFlow_Pitch;
extern u8      PixFlow_TrackLine_flag;        //通过遥控器6通道进行改变模式
extern float X_Offset,Y_Offset;
extern float X_Speed,Y_Speed;
extern float  no_compen_x_speed,no_compen_y_speed;//没有补偿的速度
extern float  no_compen_x_offset,no_compen_y_offset;//没有补偿的位移
extern _PID_val_st x_speed_val;
extern _PID_val_st x_offset_val;

extern _PID_val_st y_speed_val;
extern _PID_val_st y_offset_val;

//遥控是输入滤波之后的值 ch_in -+500  
void mode_check(float *ch_in,u8 *mode_value)
{
	//-----------------------------------------通道5进行自稳，定高，降落控制---------------------------------------------------//
	if(*(ch_in+AUX1) <-200)
	{
		mode_state = 0;//0;
		
		
		PixFlow_Roll=0;
		PixFlow_Pitch=0;
		
		no_compen_x_offset=0;
		no_compen_y_offset=0;
		
		X_Offset=0;
		Y_Offset=0;
    X_Speed=0;
		Y_Speed=0;
		
		//积分清零
		x_speed_val.err_old=0;
		x_speed_val.feedback_old=0;
		x_speed_val.err_i=0;
		
		y_speed_val.err_old=0;
		y_speed_val.feedback_old=0;
		y_speed_val.err_i=0;
		
		x_offset_val.err_old=0;
		x_offset_val.feedback_old=0;
		x_offset_val.err_i=0;
		
		y_offset_val.err_old=0;
		y_offset_val.feedback_old=0;
		y_offset_val.err_i=0;
	}
	else if(*(ch_in+AUX1) >200)
	{
		mode_state = 2;
	}
	else
	{
		mode_state = 1;
	}
	//----------------------------------------通道6（AUX2）进行光流定点和寻线控制----------------------------------------------//
	if(*(ch_in+AUX2) <-200)
	{
		//
		PixFlow_TrackLine_flag=0;
	}
	else if(*(ch_in+AUX2)>200)
	{
		PixFlow_TrackLine_flag=2;//旋钮转到最右边寻线
	}
	else
	{
		PixFlow_TrackLine_flag = 1;//旋钮转到中间光流定点
	}
	//--------------------------------------通道6(AUX2)进行光流定点和寻线控制----------------------------------------------//
	
	//=========== GPS、气压定高 ===========
	if(mode_state ==0)
	{
		*(mode_value+GPS) = *(mode_value+BARO) = 0;
	}
	else
	{
		*(mode_value+GPS) = *(mode_value+BARO) = 1;
	}

	mode_state_old = mode_state; //历史模式
}
