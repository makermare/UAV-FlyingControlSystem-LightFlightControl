#include "fly_mode.h"
#include "rc.h"

u8 mode_value[10];
u8 mode_state,mode_state_old;

//遥控是输入滤波之后的值 ch_in -+500  
void mode_check(float *ch_in,u8 *mode_value)
{
	if(*(ch_in+AUX1) <-200)
	{
		mode_state = 0;//0;
	}
	else if(*(ch_in+AUX1) >200)
	{
		mode_state = 2;//气压计和超声波融合-----超声波定高
	}
	else
	{
		mode_state = 1;//气压计定高
	}
	
	//=========== GPS、气压定高 ===========
	if(mode_state == 0 )
	{
		*(mode_value+GPS) = *(mode_value+BARO) = 0;
	}
	else
	{
		*(mode_value+GPS) = *(mode_value+BARO) = 1;
	}

	mode_state_old = mode_state; //历史模式
}
