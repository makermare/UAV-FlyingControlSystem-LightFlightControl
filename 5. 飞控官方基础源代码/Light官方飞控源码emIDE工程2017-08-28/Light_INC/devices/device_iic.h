/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved.
 + 文件名  device_iic.h
 + 描述    ：IIC设备头文件
代码尽量做到逐行注释 代码有问题，及时加群交流 作者有偿支持对开源代码的完善 */
#ifndef _DEVICE_IIC_H
#define	_DEVICE_IIC_H

#include "stm32f4xx.h"

/* IIC延时长短 */
extern volatile u8 I2C_FastMode;

/*----------------------------------------------------------
 + 实现功能：IIC设备初始化
----------------------------------------------------------*/
void I2c_Device_Init();

/*----------------------------------------------------------
 + 实现功能：IIC写入单字节数据
 + 调用参数功能：设备，寄存器，数据
 + 返回参数功能：0正常 1异常
----------------------------------------------------------*/
extern u8 IIC_Write_1Byte(u8 SlaveAddress,u8 REG_Address,u8 REG_data);

/*----------------------------------------------------------
 + 实现功能：IIC读取单字节数据
 + 调用参数功能：设备，寄存器，数据
 + 返回参数功能：0正常 1异常
----------------------------------------------------------*/
extern u8 IIC_Read_1Byte(u8 SlaveAddress,u8 REG_Address,u8 *REG_data);

/*----------------------------------------------------------
 + 实现功能：IIC写入多字节数据
 + 调用参数功能：设备，寄存器，数据长度，数据
 + 返回参数功能：0正常 1异常
----------------------------------------------------------*/
extern u8 IIC_Write_nByte(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf);

/*----------------------------------------------------------
 + 实现功能：IIC读取多字节数据
 + 调用参数功能：设备，寄存器，数据长度，数据
 + 返回参数功能：0正常 1异常
----------------------------------------------------------*/
extern u8 IIC_Read_nByte(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf);

#endif
/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved. */
