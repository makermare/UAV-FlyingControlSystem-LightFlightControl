#include "ak8975.h"
#include "parameter.h"
#include "mymath.h"
#include "include.h"
#include "i2c_soft.h"

// 	xyz_f_t Mag_Offset = { -1 , -1 , -1 };
// 	xyz_f_t Mag_Gain   = { 1 , 0.8538 , 0.9389 };

ak8975_t ak8975 = { {0,0,0},{-1,-1,-1},{1,0.8538,0.9389},{0,0,0} };

bool LIGHT_AK8975_Run(void)
{
	return IIC_Write_1Byte(AK8975_ADDRESS,AK8975_CNTL,0x01);	
}

xyz_f_t XYZ_STRUCT_COPY(float x,float y, float z)
{
	xyz_f_t m ;
	m.x = x;
	m.y = y;
	m.z = z;
	return m;
}
u8 ak8975_ok;
void LIGHT_AK8975_Read_Mag_Data(void)
{
	int16_t mag_temp[3];
	u8 ak8975_buffer[6]; //接收数据缓存
	
	I2C_FastMode = 0;
	
	IIC_Read_1Byte(AK8975_ADDRESS,AK8975_HXL,&ak8975_buffer[0]); 
	IIC_Read_1Byte(AK8975_ADDRESS,AK8975_HXH,&ak8975_buffer[1]);
	mag_temp[1] = ((((int16_t)ak8975_buffer[1]) << 8) | ak8975_buffer[0]) ;  //磁力计X轴

	IIC_Read_1Byte(AK8975_ADDRESS,AK8975_HYL,&ak8975_buffer[2]);
	IIC_Read_1Byte(AK8975_ADDRESS,AK8975_HYH,&ak8975_buffer[3]);
	mag_temp[0] = ((((int16_t)ak8975_buffer[3]) << 8) | ak8975_buffer[2]) ;  //磁力计Y轴

	IIC_Read_1Byte(AK8975_ADDRESS,AK8975_HZL,&ak8975_buffer[4]);
	IIC_Read_1Byte(AK8975_ADDRESS,AK8975_HZH,&ak8975_buffer[5]);
	mag_temp[2] = -((((int16_t)ak8975_buffer[5]) << 8) | ak8975_buffer[4]) ;  //磁力计Z轴	
	
	ak8975.Mag_Adc.x = mag_temp[0];
	ak8975.Mag_Adc.y = mag_temp[1];
	ak8975.Mag_Adc.z = mag_temp[2];
	
	ak8975.Mag_Val.x = (ak8975.Mag_Adc.x - ak8975.Mag_Offset.x) ;
	ak8975.Mag_Val.y = (ak8975.Mag_Adc.y - ak8975.Mag_Offset.y) ;
	ak8975.Mag_Val.z = (ak8975.Mag_Adc.z - ak8975.Mag_Offset.z) ;
	//磁力计中点矫正	
	LIGHT_AK8975_CalOffset_Mag();
	
	//AK8975采样触发
	LIGHT_AK8975_Run();
}

xyz_f_t LIGHT_AK8975_Get_Mag(void)
{
	return ak8975.Mag_Val;
}

u8 Mag_CALIBRATED = 0;
//磁力计中点矫正

void LIGHT_AK8975_CalOffset_Mag(void)
{
	static xyz_f_t	MagMAX = { -100 , -100 , -100 }, MagMIN = { 100 , 100 , 100 }, MagSum;
	static uint16_t cnt_m=0;
	
	if(Mag_CALIBRATED)
	{	
		
		if(ABS(ak8975.Mag_Adc.x)<400&&ABS(ak8975.Mag_Adc.y)<400&&ABS(ak8975.Mag_Adc.z)<400)
		{
			MagMAX.x = _MAX(ak8975.Mag_Adc.x, MagMAX.x);
			MagMAX.y = _MAX(ak8975.Mag_Adc.y, MagMAX.y);
			MagMAX.z = _MAX(ak8975.Mag_Adc.z, MagMAX.z);
			
			MagMIN.x = _MIN(ak8975.Mag_Adc.x, MagMIN.x);
			MagMIN.y = _MIN(ak8975.Mag_Adc.y, MagMIN.y);
			MagMIN.z = _MIN(ak8975.Mag_Adc.z, MagMIN.z);		
			
			if(cnt_m == CALIBRATING_MAG_CYCLES)
			{
				ak8975.Mag_Offset.x = (int16_t)((MagMAX.x + MagMIN.x) * 0.5f);
				ak8975.Mag_Offset.y = (int16_t)((MagMAX.y + MagMIN.y) * 0.5f);
				ak8975.Mag_Offset.z = (int16_t)((MagMAX.z + MagMIN.z) * 0.5f);
	
				MagSum.x = MagMAX.x - MagMIN.x;
				MagSum.y = MagMAX.y - MagMIN.y;
				MagSum.z = MagMAX.z - MagMIN.z;
				
				ak8975.Mag_Gain.y = MagSum.x / MagSum.y;
				ak8975.Mag_Gain.z = MagSum.x / MagSum.z;
				
				Param_SaveMagOffset(&ak8975.Mag_Offset);//param_Save();//保存数据
				cnt_m = 0;
				Mag_CALIBRATED = 0;
				f.msg_id = 3;
				f.msg_data = 1;
			}
		}
		cnt_m++;
		
	}
	else
	{

	}
}

void LIGHT_AK8975_Read(void)
{
		//读取磁力计
		LIGHT_AK8975_Read_Mag_Data();
}




