#include "usart.h"
#include "data_transfer.h"
#include "ultrasonic.h"
#include "ms5611.h"
#include "math.h"
#include "mpu6050.h"

extern _height_st ultra;
extern MPU6050_STRUCT mpu6050;

void Usart1_Init(u32 br_num)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //Ê¹ÄÜGPIOAÊ±ÖÓ
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//Ê¹ÄÜUSART1Ê±ÖÓ
  
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9¸´ÓÃÎªUSART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10¸´ÓÃÎªUSART1
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //GPIOA9ÓëGPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//¸´ÓÃ¹¦ÄÜ
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//ËÙ¶È50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //ÍÆÍì¸´ÓÃÊä³ö
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //ÉÏÀ­
	GPIO_Init(GPIOA,&GPIO_InitStructure); //³õÊ¼»¯PA9£¬PA10
   
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //GPIOA9ÓëGPIOA10
	GPIO_InitStructure.GPIO_Mode= GPIO_Mode_AF;//¸´ÓÃ¹¦ÄÜ
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;	//ËÙ¶È50MHz
	GPIO_InitStructure.GPIO_OType=GPIO_OType_OD; //ÍÆÍì¸´ÓÃÊä³ö
	GPIO_InitStructure.GPIO_PuPd =GPIO_PuPd_NOPULL; //ÉÏÀ­
	GPIO_Init(GPIOA,&GPIO_InitStructure); //³õÊ¼»¯PA9£¬PA10
  
	//USART1 
	USART_InitStructure.USART_BaudRate = br_num;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//×Ö³¤Îª8Î»Êý¾Ý¸ñÊ½
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//Ò»¸öÍ£Ö¹Î»
	USART_InitStructure.USART_Parity = USART_Parity_No;//ÎÞÆæÅ¼Ð£ÑéÎ»
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//ÎÞÓ²¼þÊý¾ÝÁ÷¿ØÖÆ
	USART_InitStructure.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;	//ÊÕ·¢Ä£Ê½
  
	USART_Init(USART1, &USART_InitStructure); //³õÊ¼»¯´®¿Ú1
	
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		
	NVIC_Init(&NVIC_InitStructure);	
	
	//开启串口接收中断
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//
  USART_Cmd(USART1, ENABLE);  
}
 
///////////////////////////////////////串口1接收中断////////////////////////////////////////
//图片大小 B64*32
#define  Roll_Compensation     64.0f/(2.0f*atan(32.0f/50.0f)*57.3f)   //每一度对应多少个像素点
#define  Pitch_Compensation    64.0f/(2.0f*atan(32.0f/50.0f)*57.3f)

uint8_t USART_RXBUF[25];//缓冲数组
int16_t my_data[10];     //分别存储 偏转角度(度)X轴位移CM  Y轴位移CM
uint8_t RXOVER = 0;     //接收数据完成标志
uint8_t RXCUNT = 0;     //接收字节数量
extern float X_Offset,Y_Offset; //补偿之后的位移
extern float X_Speed,Y_Speed;   //补偿之后的速度
float  no_compen_x_speed,no_compen_y_speed;  //没有补偿的速度
float  no_compen_x_offset,no_compen_y_offset;//没有补偿的位移
extern u8    PixFlow_TrackLine_flag;//定点寻线标位
extern float Roll,Pitch,Yaw;
extern u8    turn_flag;
extern u8    start_flag;
extern u8    position_control;
void USART1_IRQHandler(void)
{
  uint8_t temp;
	int i;
	if(USART_GetITStatus(USART1, USART_IT_RXNE)!= RESET)//接收到数据
	{
	  USART_ClearITPendingBit(USART1,USART_IT_RXNE);		
		temp=USART_ReceiveData(USART1);
		if(temp=='#')//接收数据完成   摄像头uart.write('s'+out_str1+'#')
		{  
		    if(USART_RXBUF[0]=='s')//校验帧头正确
				{  
						 if(USART_RXBUF[1]=='-')
						 {
								my_data[0]=-((USART_RXBUF[2]-'0')*10+(USART_RXBUF[3]-'0'));//控制yaw控制偏航
						 }
						 else if(USART_RXBUF[1]=='+')
						 {
								 my_data[0]=(USART_RXBUF[2]-'0')*10+(USART_RXBUF[3]-'0');
						 }
						 if(USART_RXBUF[4]=='-')
						 {
								 my_data[1]=-((USART_RXBUF[5]-'0')*10+ (USART_RXBUF[6]-'0'));
						 }
						 else if(USART_RXBUF[4]=='+')
						 {
								 my_data[1]=(USART_RXBUF[5]-'0')*10+ (USART_RXBUF[6]-'0');
						 }
						 if(USART_RXBUF[7]=='-')
						 {
									my_data[2]=-((USART_RXBUF[8]-'0')*10+ (USART_RXBUF[9]-'0'));
						 }
						 else if(USART_RXBUF[7]=='+')
						 {
									my_data[2]=(USART_RXBUF[8]-'0')*10+(USART_RXBUF[9]-'0');
						 }
						 my_data[3]=(USART_RXBUF[10]-'0')*10+(USART_RXBUF[11]-'0');//center_pos的位置 , 控制roll
						 my_data[4]=(USART_RXBUF[12]-'0')*10+(USART_RXBUF[13]-'0');//center_x 圆心坐标x
						 my_data[5]=(USART_RXBUF[14]-'0')*10+(USART_RXBUF[15]-'0');//center_y 圆心坐标y
						 my_data[6]=(USART_RXBUF[16]-'0')*10+(USART_RXBUF[17]-'0');//开始转弯的标志  ,=1左转90度  ,=2右转90度
						 my_data[7]=(USART_RXBUF[18]-'0')*10+(USART_RXBUF[19]-'0');//直角点x坐标
						 my_data[8]=(USART_RXBUF[20]-'0')*10+(USART_RXBUF[21]-'0');//直角点y坐标
				}
				if(PixFlow_TrackLine_flag==1)//定点模式计算速度和位移
				{
							if(start_flag==2&&my_data[6]==1)//如果start_flag==2表示正在前进在，此时需要查询有没有检测到直角转弯的标志
							{
								 turn_flag=1;
							}
							//位移暂时用圆心坐标来表示
							if(position_control==1&&turn_flag==0)//没有遇到直角，圆心定点
							{
								 X_Offset=(float)(32-my_data[7])*(float)ultra.height/50.0f*10;
								 Y_Offset=-(float)(32-my_data[8])*(float)ultra.height/50.0f*10;		
//								 X_Offset= (float)(32-my_data[4])*(float)ultra.height/50.0f*10;//单位mm
//								 Y_Offset=-(float)(32-my_data[5])*(float)ultra.height/50.0f*10;
							}
							else if(position_control==1&&turn_flag==1)//遇到直角之后, 直角定点
							{
								 X_Offset=(float)(32-my_data[7])*(float)ultra.height/50.0f*10;
								 Y_Offset=-(float)(32-my_data[8])*(float)ultra.height/50.0f*10;					
							}
							if(position_control==2)
							{
								 X_Offset=(float)(32-my_data[3])*(float)ultra.height/50.0f*10;
								 Y_Offset=0;
							}
							if(X_Offset>-20&&X_Offset<20)
							{
								 X_Offset=0;
							}
							if(Y_Offset>-20&&Y_Offset<20)
							{
								 Y_Offset=0;
							}
							//像素点的位移之和转换成实际的距离,实际测试，B64x32的图片   高度50cm 60cm对应60个像素点
							no_compen_x_speed=(float)my_data[1]*(float)ultra.height/50.0f/0.05f;
							no_compen_y_speed=(float)my_data[2]*(float)ultra.height/50.0f/0.05f;
							//计算对应的速度. my_data[1]=移动像素点*10的倍数  T=0.02s    陀螺仪角速度单位 degree/s 
							X_Speed=((float)my_data[1]/10.0f-Roll_Compensation*(-mpu6050.Gyro_deg.x)*0.05f)*(float)ultra.height/50.0f*10.0f/0.05f;//mm/s 
							Y_Speed=((float)my_data[2]/10.0f-Pitch_Compensation*(-mpu6050.Gyro_deg.y)*0.05f)*(float)ultra.height/50.0f*10.0f/0.05f;
				}
			  RXCUNT = 0;
			  RXOVER = 1;//接收完成标志位置
			 	for(i=0;i<25;i++)
			  {
				  USART_RXBUF[i]=' ';//清空接收区
			  }
        //USART_ITConfig(USART1,USART_IT_RXNE,DISABLE);//关闭接收中断
		}
		else//还没有接收完成，继续接收
		{
			USART_RXBUF[RXCUNT]=temp;
			++RXCUNT;			
		}								 
   } 
}


void Usart2_Init(u32 br_num)
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);	
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);
	
	//配置PD5作为USART2　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
	//配置PD6作为USART2　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
	
	//配置USART2
	//中断被屏蔽了
	USART_InitStructure.USART_BaudRate = br_num;       //波特率可以通过地面站配置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8位数据
	USART_InitStructure.USART_StopBits = USART_StopBits_1;   //在帧结尾传输1个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;    //禁用奇偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //发送、接收使能
	//配置USART2时钟
	USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;  //时钟低电平活动
	USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;  //SLCK引脚上时钟输出的极性->低电平
	USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;  //时钟第二个边沿进行数据捕获
	USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable; //最后一位数据的时钟脉冲不从SCLK输出
	
	USART_Init(USART2, &USART_InitStructure);
	USART_ClockInit(USART2, &USART_ClockInitStruct);

	//使能USART2接收中断
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	//使能USART2
	USART_Cmd(USART2, ENABLE); 
//	//使能发送（进入移位）中断
//	if(!(USART2->CR1 & USART_CR1_TXEIE))
//	{
//		USART_ITConfig(USART2, USART_IT_TXE, ENABLE); 
//	}
}

u8 TxBuffer[256];
u8 TxCounter=0;
u8 count=0; 

u8 Rx_Buf[256];	//串口接收缓存

void Usart2_IRQ(void)
{
	u8 com_data;
	
	if(USART2->SR & USART_SR_ORE)//ORE中断
	{
		com_data = USART2->DR;
	}

  //接收中断
	if( USART_GetITStatus(USART2,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);//清除中断标志

		com_data = USART2->DR;
		LIGHT_DT_Data_Receive_Prepare(com_data);
	}
	//发送（进入移位）中断
	if( USART_GetITStatus(USART2,USART_IT_TXE ) )
	{		
		USART2->DR = TxBuffer[TxCounter++]; //写DR清除中断标志          
		if(TxCounter == count)
		{
			USART2->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
		}
		//USART_ClearITPendingBit(USART2,USART_IT_TXE);
	}
}

void Usart2_Send(unsigned char *DataToSend ,u8 data_num)
{
  u8 i;
	for(i=0;i<data_num;i++)
	{
		TxBuffer[count++] = *(DataToSend+i);
	}

	if(!(USART2->CR1 & USART_CR1_TXEIE))
	{
		USART_ITConfig(USART2, USART_IT_TXE, ENABLE); //打开发送中断
	}
}

void Uart5_Init(u32 br_num)
{
	USART_InitTypeDef USART_InitStructure;
	//USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);
	
	//配置PC12作为UART5　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 
	//配置PD2作为UART5　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
	
	//配置UART5
	//中断被屏蔽了
	USART_InitStructure.USART_BaudRate = br_num;       //波特率可以通过地面站配置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8位数据
	USART_InitStructure.USART_StopBits = USART_StopBits_1;   //在帧结尾传输1个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;    //禁用奇偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //发送、接收使能
	USART_Init(UART5, &USART_InitStructure);
	
	//使能UART5接收中断
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
	//使能USART5
	USART_Cmd(UART5, ENABLE); 
//	//使能发送（进入移位）中断
//	if(!(USART2->CR1 & USART_CR1_TXEIE))
//	{
//		USART_ITConfig(USART2, USART_IT_TXE, ENABLE); 
//	}

}
u8 Tx5Buffer[256];
u8 Tx5Counter=0;
u8 count5=0; 

void Uart5_IRQ(void)
{
	u8 com_data;

  //接收中断
	if( USART_GetITStatus(UART5,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(UART5,USART_IT_RXNE);//清除中断标志

		com_data = UART5->DR;
		
		Ultra_Get(com_data);
	}

	//发送（进入移位）中断
	if( USART_GetITStatus(UART5,USART_IT_TXE ) )
	{
				
		UART5->DR = Tx5Buffer[Tx5Counter++]; //写DR清除中断标志
          
		if(Tx5Counter == count5)
		{
			UART5->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
		}
		//USART_ClearITPendingBit(USART2,USART_IT_TXE);
	}
}

void Uart5_Send(unsigned char *DataToSend ,u8 data_num)
{
	u8 i;
	for(i=0;i<data_num;i++)
	{
		Tx5Buffer[count5++] = *(DataToSend+i);
	}
	if(!(UART5->CR1 & USART_CR1_TXEIE))
	{
		USART_ITConfig(UART5, USART_IT_TXE, ENABLE); //打开发送中断
	}
}



