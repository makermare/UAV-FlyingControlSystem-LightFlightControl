#include "data_transfer.h"
#include "usart.h"
#include "imu.h"
#include "mpu6050.h"
#include "ak8975.h"
#include "ms5611.h"
#include "rc.h"
#include "ctrl.h"
#include "time.h"
#include "usbd_user_hid.h"
#include "ultrasonic.h"
#include "baro_ctrl.h"
#include "PID.h"
/////////////////////////////////////////////////////////////////////////////////////
//数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、float等，需要把数据拆分成单独字节进行发送
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

dt_flag_t f;					//需要发送数据的标志
u8 data_to_send[50];	//发送数据缓存
u8 checkdata_to_send,checksum_to_send;

/////////////////////////////////////////////////////////////////////////////////////
//Send_Data函数是协议中所有发送数据功能使用到的发送函数
//移植时，用户应根据自身应用的情况，根据使用的通信方式，实现此函数
void LIGHT_DT_Send_Data(u8 *dataToSend , u8 length)
{
#ifdef LIGHT_DT_USE_USB_HID
	Usb_Hid_Adddata(data_to_send,length);
#endif
#ifdef LIGHT_DT_USE_USART2
	Usart2_Send(data_to_send, length);
#endif
}
static void LIGHT_DT_Send_Check(u8 head, u8 check_sum)
{
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xEF;
	data_to_send[3]=2;
	data_to_send[4]=head;
	data_to_send[5]=check_sum;
	
	u8 sum=0;
	for(u8 i=0;i<6;i++)
		sum += data_to_send[i];
	data_to_send[6]=sum;

	LIGHT_DT_Send_Data(data_to_send, 7);
}
static void LIGHT_DT_Send_Msg(u8 id, u8 data)
{
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xEE;
	data_to_send[3]=2;
	data_to_send[4]=id;
	data_to_send[5]=data;

	u8 sum = 0;
	for(u8 i=0;i<6;i++)
		sum += data_to_send[i];
	data_to_send[6]=sum;

	LIGHT_DT_Send_Data(data_to_send, 7);
}
/////////////////////////////////////////////////////////////////////////////////////
//Data_Exchange函数处理各种数据发送请求，比如想实现每5ms发送一次传感器数据至上位机，即在此函数内实现
//此函数应由用户每1ms调用一次
extern float ultra_dis_lpf;

extern float PixFlow_Roll;
extern float PixFlow_Pitch;
extern float  no_compen_x_speed,no_compen_y_speed;
extern _PID_arg_st x_speed_arg;
extern _PID_arg_st y_speed_arg;
extern _PID_arg_st x_offset_arg;
extern _PID_arg_st y_offset_arg;

extern uint8_t  RXOVER;              //接收数据完成标志
extern int16_t  my_data[10];  
extern float X_Offset,Y_Offset;
extern float X_Speed,Y_Speed;
extern float  exp_speed_x,exp_speed_y;

extern u8     PixFlow_TrackLine_flag;
float angle_max=1.0f;
float speed_max=2.5f;
extern u8    turn_flag;
void LIGHT_DT_Data_Exchange(void)
{
	static u8 cnt = 0;
	static u8 senser_cnt 	= 50;
	static u8 senser2_cnt = 200;
	static u8 user_cnt 	  = 100;
	static u8 status_cnt 	= 150;
	static u8 rcdata_cnt 	= 200;
	static u8 motopwm_cnt	= 200;
	static u8 power_cnt		=	250;
	static u8 speed_cnt   = 250;
	static u8 location_cnt   = 250;
	
	if((cnt % senser_cnt) == (senser_cnt-1))
		f.send_senser = 1;

	if((cnt % senser2_cnt) == (senser2_cnt-1))
		f.send_senser2 = 1;	

	if((cnt % user_cnt) == (user_cnt-2))
		f.send_user = 1;
	
	if((cnt % status_cnt) == (status_cnt-1))
		f.send_status = 1;	
	
	if((cnt % rcdata_cnt) == (rcdata_cnt-1))
		f.send_rcdata = 1;	
	
	if((cnt % motopwm_cnt) == (motopwm_cnt-2))
		f.send_motopwm = 1;	
	
	if((cnt % power_cnt) == (power_cnt-2))
		f.send_power = 1;		
	
	if((cnt % speed_cnt) == (speed_cnt-3))
		f.send_speed = 1;		
	
	if((cnt % location_cnt) == (location_cnt-3))
	{
		f.send_location += 1;		
	}
	
	if(++cnt>200) cnt = 0;
/////////////////////////////////////////////////////////////////////////////////////
	if(f.msg_id)
	{
		LIGHT_DT_Send_Msg(f.msg_id,f.msg_data);
		f.msg_id = 0;
	}
/////////////////////////////////////////////////////////////////////////////////////
	if(f.send_check)
	{
		f.send_check = 0;
		LIGHT_DT_Send_Check(checkdata_to_send,checksum_to_send);
	}
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_version)
	{
		f.send_version = 0;
		LIGHT_DT_Send_Version(4,300,100,400,0);
	}
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_status)
	{
		f.send_status = 0;
		LIGHT_DT_Send_Status(Roll,Pitch,Yaw,(0.1f *baro_fusion.fusion_displacement.out),0,fly_ready);	
	}	
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_speed)
	{
		f.send_speed = 0;
		LIGHT_DT_Send_Speed(0,0,wz_speed);
	}
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_user)
	{
		f.send_user = 0;
		LIGHT_DT_Send_User();
	}
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_senser)
	{
		 f.send_senser = 0;// 
//		 LIGHT_DT_Send_Senser((int16_t)X_Speed,(int16_t)Y_Speed,(int16_t)turn_flag,
//												(int16_t)no_compen_x_speed, (int16_t)no_compen_y_speed,(int16_t)exp_speed_x,
//												(int16_t)exp_speed_y,(int16_t)PixFlow_Roll,(int16_t)PixFlow_Pitch);
			LIGHT_DT_Send_Senser(mpu6050.Acc.x,mpu6050.Acc.y,mpu6050.Acc.z,
															mpu6050.Gyro.x,mpu6050.Gyro.y,mpu6050.Gyro.z,
															ak8975.Mag_Val.x,ak8975.Mag_Val.y,ak8975.Mag_Val.z);
	}	
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_senser2)
	{
		f.send_senser2 = 0;
		LIGHT_DT_Send_Senser2(baro.height,ultra.height);//原始数据
	}	
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_rcdata)
	{
		f.send_rcdata = 0;
		LIGHT_DT_Send_RCData(CH[2]+1500,CH[3]+1500,CH[0]+1500,CH[1]+1500,CH[4]+1500,CH[5]+1500,CH[6]+1500,CH[7]+1500,0 +1500,0 +1500);
	}	
/////////////////////////////////////////////////////////////////////////////////////	
	else if(f.send_motopwm)
	{
		f.send_motopwm = 0;
#if MAXMOTORS == 8
		LIGHT_DT_Send_MotoPWM(motor_out[0],motor_out[1],motor_out[2],motor_out[3],motor_out[4],motor_out[5],motor_out[6],motor_out[7]);
#elif MAXMOTORS == 6
		LIGHT_DT_Send_MotoPWM(motor_out[0],motor_out[1],motor_out[2],motor_out[3],motor_out[4],motor_out[5],0,0);
#elif MAXMOTORS == 4
		LIGHT_DT_Send_MotoPWM(motor_out[0],motor_out[1],motor_out[2],motor_out[3],0,0,0,0);
#else
		
#endif
	}	
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_power)
	{
		f.send_power = 0;
		LIGHT_DT_Send_Power(123,456);
	}
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_pid1)
	{
		f.send_pid1 = 0;
		LIGHT_DT_Send_PID(1,ctrl_1.PID[PIDROLL].kp,ctrl_1.PID[PIDROLL].ki,ctrl_1.PID[PIDROLL].kd,
											ctrl_1.PID[PIDPITCH].kp,ctrl_1.PID[PIDPITCH].ki,ctrl_1.PID[PIDPITCH].kd,
											ctrl_1.PID[PIDYAW].kp,ctrl_1.PID[PIDYAW].ki,ctrl_1.PID[PIDYAW].kd);
	}	
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_pid2)
	{
		f.send_pid2 = 0;
		LIGHT_DT_Send_PID(2,ctrl_2.PID[PIDROLL].kp,ctrl_2.PID[PIDROLL].ki,ctrl_2.PID[PIDROLL].kd,
											ctrl_2.PID[PIDPITCH].kp,ctrl_2.PID[PIDPITCH].ki,ctrl_2.PID[PIDPITCH].kd,
											ctrl_2.PID[PIDYAW].kp,ctrl_2.PID[PIDYAW].ki,ctrl_2.PID[PIDYAW].kd);
	}
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_pid3)
	{
		f.send_pid3 = 0;
		LIGHT_DT_Send_PID(3,pid_setup.groups.hc_sp.kp,pid_setup.groups.hc_sp.ki,pid_setup.groups.hc_sp.kd,
											pid_setup.groups.hc_height.kp,pid_setup.groups.hc_height.ki,pid_setup.groups.hc_height.kd,
											pid_setup.groups.ctrl3.kp,pid_setup.groups.ctrl3.ki,pid_setup.groups.ctrl3.kd);
	}
	else if(f.send_pid4)
	{
		f.send_pid4 = 0;
		LIGHT_DT_Send_PID(4,pid_setup.groups.ctrl4.kp,pid_setup.groups.ctrl4.ki,pid_setup.groups.ctrl4.kd,
									    x_speed_arg.kp    ,x_speed_arg.ki	,x_speed_arg.kd	,
											y_speed_arg.kp 	,y_speed_arg.ki	,y_speed_arg.kd);
	}
	else if(f.send_pid5)
	{
		f.send_pid5 = 0;
		//LIGHT_DT_Send_PID(5,angle_max,0,0,0,0,0,0,0,0);
		LIGHT_DT_Send_PID(5,x_offset_arg.kp,x_offset_arg.ki,x_offset_arg.kd,y_offset_arg.kp,y_offset_arg.ki,y_offset_arg.kd,angle_max,speed_max,0);
	}
	else if(f.send_pid6)
	{
		f.send_pid6 = 0;
		LIGHT_DT_Send_PID(6,0,0,0,0,0,0,0,0,0);
	}
	else if(f.send_location == 2)
	{
		f.send_location = 0;
		LIGHT_DT_Send_Location(0,0,0 *10000000,0  *10000000,0);
	}
/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
	Usb_Hid_Send();					
/////////////////////////////////////////////////////////////////////////////////////
}

/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Prepare函数是协议预解析，根据协议的格式，将收到的数据进行一次格式性解析，格式正确的话再进行数据解析
//移植时，此函数应由用户根据自身使用的通信方式自行调用，比如串口每收到一字节数据，则调用此函数一次
//此函数解析出符合格式的数据帧后，会自行调用数据解析函数
void LIGHT_DT_Data_Receive_Prepare(u8 data)
{
	static u8 RxBuffer[50];
	static u8 _data_len = 0,_data_cnt = 0;
	static u8 state = 0;
	
	if(state==0&&data==0xAA)
	{
		state=1;
		RxBuffer[0]=data;
	}
	else if(state==1&&data==0xAF)
	{
		state=2;
		RxBuffer[1]=data;
	}
	else if(state==2&&data<0XF1)
	{
		state=3;
		RxBuffer[2]=data;
	}
	else if(state==3&&data<50)
	{
		state = 4;
		RxBuffer[3]=data;
		_data_len = data;
		_data_cnt = 0;
	}
	else if(state==4&&_data_len>0)
	{
		_data_len--;
		RxBuffer[4+_data_cnt++]=data;
		if(_data_len==0)
			state = 5;
	}
	else if(state==5)
	{
		state = 0;
		RxBuffer[4+_data_cnt]=data;
		LIGHT_DT_Data_Receive_Anl(RxBuffer,_data_cnt+5);
	}
	else
		state = 0;
}
/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Anl函数是协议数据解析函数，函数参数是符合协议格式的一个数据帧，该函数会首先对协议数据进行校验
//校验通过后对数据进行解析，实现相应功能
//此函数可以不用用户自行调用，由函数Data_Receive_Prepare自动调用
u16 flash_save_en_cnt = 0;
u16 RX_CH[CH_NUM];

void LIGHT_DT_Data_Receive_Anl(u8 *data_buf,u8 num)
{
	u8 sum = 0;
	for(u8 i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
	
	if(*(data_buf+2)==0X01)
	{
		if(*(data_buf+4)==0X01)
		{
			mpu6050.Acc_CALIBRATE = 1;
			//mpu6050.Cali_3d = 1;
		}
		else if(*(data_buf+4)==0X02)
			mpu6050.Gyro_CALIBRATE = 1;
		else if(*(data_buf+4)==0X03)
		{
			mpu6050.Acc_CALIBRATE = 1;		
			mpu6050.Gyro_CALIBRATE = 1;			
		}
		else if(*(data_buf+4)==0X04)
		{
			Mag_CALIBRATED = 1;
		}
		else if((*(data_buf+4)>=0X021)&&(*(data_buf+4)<=0X26))
		{
			//acc_3d_calibrate_f = 1;
		}
		else if(*(data_buf+4)==0X20)
		{
			//acc_3d_step = 0; //退出，6面校准步清0
		}
	}
	
	if(*(data_buf+2)==0X02)
	{
		if(*(data_buf+4)==0X01)
		{
			f.send_pid1 = 1;
			f.send_pid2 = 1;
			f.send_pid3 = 1;
			f.send_pid4 = 1;
			f.send_pid5 = 1;
			f.send_pid6 = 1;
		}
		if(*(data_buf+4)==0X02)
		{
			
		}
		if(*(data_buf+4)==0XA0)		//读取版本信息
		{
			f.send_version = 1;
		}
		if(*(data_buf+4)==0XA1)		//恢复默认参数
		{
			Para_ResetToFactorySetup();
		}
	}

	if(*(data_buf+2)==0X03)
	{
		if( NS != 1 )
		{
			Feed_Rc_Dog(2);
		}

		RX_CH[THR] = (vs16)(*(data_buf+4)<<8)|*(data_buf+5) ;
		RX_CH[YAW] = (vs16)(*(data_buf+6)<<8)|*(data_buf+7) ;
		RX_CH[ROL] = (vs16)(*(data_buf+8)<<8)|*(data_buf+9) ;
		RX_CH[PIT] = (vs16)(*(data_buf+10)<<8)|*(data_buf+11) ;
		RX_CH[AUX1] = (vs16)(*(data_buf+12)<<8)|*(data_buf+13) ;
		RX_CH[AUX2] = (vs16)(*(data_buf+14)<<8)|*(data_buf+15) ;
		RX_CH[AUX3] = (vs16)(*(data_buf+16)<<8)|*(data_buf+17) ;
		RX_CH[AUX4] = (vs16)(*(data_buf+18)<<8)|*(data_buf+19) ;
	}

	if(*(data_buf+2)==0X10)								//PID1
    {
        ctrl_1.PID[PIDROLL].kp  = 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        ctrl_1.PID[PIDROLL].ki  = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        ctrl_1.PID[PIDROLL].kd  = 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
        ctrl_1.PID[PIDPITCH].kp = 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
        ctrl_1.PID[PIDPITCH].ki = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
        ctrl_1.PID[PIDPITCH].kd = 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
        ctrl_1.PID[PIDYAW].kp 	= 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
        ctrl_1.PID[PIDYAW].ki 	= 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
        ctrl_1.PID[PIDYAW].kd 	= 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
				if(f.send_check == 0)
				{
					f.send_check = 1;
					checkdata_to_send = *(data_buf+2);
					checksum_to_send = sum;
				}
			  PID_Para_Init();
				flash_save_en_cnt = 1;
    }
    if(*(data_buf+2)==0X11)								//PID2
    {
        ctrl_2.PID[PIDROLL].kp  = 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        ctrl_2.PID[PIDROLL].ki  = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        ctrl_2.PID[PIDROLL].kd  = 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
        ctrl_2.PID[PIDPITCH].kp = 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
        ctrl_2.PID[PIDPITCH].ki = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
        ctrl_2.PID[PIDPITCH].kd = 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
        ctrl_2.PID[PIDYAW].kp 	= 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
        ctrl_2.PID[PIDYAW].ki 	= 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
        ctrl_2.PID[PIDYAW].kd 	= 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
        if(f.send_check == 0)
				{
					f.send_check = 1;
					checkdata_to_send = *(data_buf+2);
					checksum_to_send = sum;
				}
				PID_Para_Init();
				flash_save_en_cnt = 1;
    }
    if(*(data_buf+2)==0X12)								//PID3
    {	
        pid_setup.groups.hc_sp.kp  = 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        pid_setup.groups.hc_sp.ki  = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        pid_setup.groups.hc_sp.kd  = 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
			
        pid_setup.groups.hc_height.kp = 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
        pid_setup.groups.hc_height.ki = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
        pid_setup.groups.hc_height.kd = 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
			
        pid_setup.groups.ctrl3.kp 	= 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
        pid_setup.groups.ctrl3.ki 	= 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
        pid_setup.groups.ctrl3.kd 	= 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
        if(f.send_check == 0)
				{
					f.send_check = 1;
					checkdata_to_send = *(data_buf+2);
					checksum_to_send = sum;
				}
				PID_Para_Init();
				flash_save_en_cnt =1;
    }
	if(*(data_buf+2)==0X13)								//PID4
	{
		    pid_setup.groups.ctrl4.kp  = 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        pid_setup.groups.ctrl4.ki  = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        pid_setup.groups.ctrl4.kd  = 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
			  
        x_speed_arg.kp = 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
        x_speed_arg.ki = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
        x_speed_arg.kd = 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
		    
		    y_speed_arg.kp = 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
        y_speed_arg.ki = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
        y_speed_arg.kd = 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );			
//      pid_setup.groups.ctrl3.kp= 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
//      pid_setup.groups.ctrl3.ki= 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
//      pid_setup.groups.ctrl3.kd= 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
		   if(f.send_check == 0)
		   {
			   f.send_check = 1;
			   checkdata_to_send=*(data_buf+2);
			   checksum_to_send =sum;
		  }
//		PID_Para_Init();
//		flash_save_en_cnt = 1;
	}
	if(*(data_buf+2)==0X14)//PID5
	{
		//angle_max= 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
		 x_offset_arg.kp  = 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
     x_offset_arg.ki  = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
     x_offset_arg.kd  = 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
			  
     y_offset_arg.kp = 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
     y_offset_arg.ki = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
     y_offset_arg.kd = 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
		 
		 angle_max= 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
		 speed_max = 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19));
		if(f.send_check == 0)
		{
			f.send_check=1;
			checkdata_to_send=*(data_buf+2);
			checksum_to_send = sum;
		}
	}
	if(*(data_buf+2)==0X15)								//PID6
	{
		if(f.send_check == 0)
		{
			f.send_check = 1;
			checkdata_to_send = *(data_buf+2);
			checksum_to_send = sum;
		}
	}
}

void LIGHT_DT_Send_Version(u8 hardware_type, u16 hardware_ver,u16 software_ver,u16 protocol_ver,u16 bootloader_ver)
{
	u8 _cnt=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x00;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=hardware_type;
	data_to_send[_cnt++]=BYTE1(hardware_ver);
	data_to_send[_cnt++]=BYTE0(hardware_ver);
	data_to_send[_cnt++]=BYTE1(software_ver);
	data_to_send[_cnt++]=BYTE0(software_ver);
	data_to_send[_cnt++]=BYTE1(protocol_ver);
	data_to_send[_cnt++]=BYTE0(protocol_ver);
	data_to_send[_cnt++]=BYTE1(bootloader_ver);
	data_to_send[_cnt++]=BYTE0(bootloader_ver);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	LIGHT_DT_Send_Data(data_to_send, _cnt);
}

void LIGHT_DT_Send_Speed(float x_s,float y_s,float z_s)
{
	u8 _cnt=0;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x0B;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(0.1f *x_s);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(0.1f *y_s);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(0.1f *z_s);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	LIGHT_DT_Send_Data(data_to_send, _cnt);

}

void LIGHT_DT_Send_Location(u8 state,u8 sat_num,s32 lon,s32 lat,float back_home_angle)
{
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp2;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x04;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=state;
	data_to_send[_cnt++]=sat_num;
	
	_temp2 = lon;//经度
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);	
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	_temp2 = lat;//纬度
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);	
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	
	_temp = (s16)(100 *back_home_angle);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	LIGHT_DT_Send_Data(data_to_send, _cnt);

}


void LIGHT_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed)
{
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp2 = alt;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(angle_rol*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_pit*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	data_to_send[_cnt++] = fly_model;
	
	data_to_send[_cnt++] = armed;
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	LIGHT_DT_Send_Data(data_to_send, _cnt);
}
void LIGHT_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z)
{
	u8 _cnt=0;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	
	_temp = a_x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = g_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = m_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
/////////////////////////////////////////
	_temp = 0;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);	
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	LIGHT_DT_Send_Data(data_to_send, _cnt);
}
void LIGHT_DT_Send_Senser2(s32 bar_alt,u16 csb_alt)
{
	u8 _cnt=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x07;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTE3(bar_alt);
	data_to_send[_cnt++]=BYTE2(bar_alt);
	data_to_send[_cnt++]=BYTE1(bar_alt);
	data_to_send[_cnt++]=BYTE0(bar_alt);

	data_to_send[_cnt++]=BYTE1(csb_alt);
	data_to_send[_cnt++]=BYTE0(csb_alt);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	LIGHT_DT_Send_Data(data_to_send, _cnt);
}
void LIGHT_DT_Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6)
{
	u8 _cnt=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x03;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=BYTE1(thr);
	data_to_send[_cnt++]=BYTE0(thr);
	data_to_send[_cnt++]=BYTE1(yaw);
	data_to_send[_cnt++]=BYTE0(yaw);
	data_to_send[_cnt++]=BYTE1(rol);
	data_to_send[_cnt++]=BYTE0(rol);
	data_to_send[_cnt++]=BYTE1(pit);
	data_to_send[_cnt++]=BYTE0(pit);
	data_to_send[_cnt++]=BYTE1(aux1);
	data_to_send[_cnt++]=BYTE0(aux1);
	data_to_send[_cnt++]=BYTE1(aux2);
	data_to_send[_cnt++]=BYTE0(aux2);
	data_to_send[_cnt++]=BYTE1(aux3);
	data_to_send[_cnt++]=BYTE0(aux3);
	data_to_send[_cnt++]=BYTE1(aux4);
	data_to_send[_cnt++]=BYTE0(aux4);
	data_to_send[_cnt++]=BYTE1(aux5);
	data_to_send[_cnt++]=BYTE0(aux5);
	data_to_send[_cnt++]=BYTE1(aux6);
	data_to_send[_cnt++]=BYTE0(aux6);

	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	LIGHT_DT_Send_Data(data_to_send, _cnt);
}
void LIGHT_DT_Send_Power(u16 votage, u16 current)
{
	u8 _cnt=0;
	u16 temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x05;
	data_to_send[_cnt++]=0;
	
	temp = votage;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	temp = current;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	LIGHT_DT_Send_Data(data_to_send, _cnt);
}
void LIGHT_DT_Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8)
{
	u8 _cnt=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x06;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTE1(m_1);
	data_to_send[_cnt++]=BYTE0(m_1);
	data_to_send[_cnt++]=BYTE1(m_2);
	data_to_send[_cnt++]=BYTE0(m_2);
	data_to_send[_cnt++]=BYTE1(m_3);
	data_to_send[_cnt++]=BYTE0(m_3);
	data_to_send[_cnt++]=BYTE1(m_4);
	data_to_send[_cnt++]=BYTE0(m_4);
	data_to_send[_cnt++]=BYTE1(m_5);
	data_to_send[_cnt++]=BYTE0(m_5);
	data_to_send[_cnt++]=BYTE1(m_6);
	data_to_send[_cnt++]=BYTE0(m_6);
	data_to_send[_cnt++]=BYTE1(m_7);
	data_to_send[_cnt++]=BYTE0(m_7);
	data_to_send[_cnt++]=BYTE1(m_8);
	data_to_send[_cnt++]=BYTE0(m_8);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	LIGHT_DT_Send_Data(data_to_send, _cnt);
}
void LIGHT_DT_Send_PID(u8 group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d)
{
	u8 _cnt=0;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x10+group-1;
	data_to_send[_cnt++]=0;
	
	
	_temp = p1_p * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_d  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_p  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_d * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_p  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_d * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	LIGHT_DT_Send_Data(data_to_send, _cnt);
}


extern float yaw_mag,airframe_x_sp,airframe_y_sp,wx_sp,wy_sp;
extern float werr_x_gps,werr_y_gps,aerr_x_gps,aerr_y_gps;

void LIGHT_DT_Send_User()
{
	u8 _cnt=0;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA; 
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xf1; //用户数据
	data_to_send[_cnt++]=0;
	
	
	_temp = (s16)baro_p.displacement;            //1
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	_temp = (s16)wz_speed;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (s16)baro_p.speed;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);	
	
	_temp = (s16)baro_fusion.fusion_acceleration.out;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
  _temp = (s16)baro_fusion.fusion_displacement.out;              //5
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (s16)(ultra.height *10);              //6
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	_temp = (s16)(10000 * reference_v.z);              //7
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	LIGHT_DT_Send_Data(data_to_send, _cnt);
}
