#ifndef _USART_H
#define _USART_H

#include "stm32f4xx.h"

extern u8 Rx_Buf[];

void Usart1_Init(u32 br_num);
void Usart3_Init(u32 br_num);
void USART_SendString(int8_t *str);

void Usart2_Init(u32 br_num);
void Usart2_IRQ(void);
void Usart2_Send(unsigned char *DataToSend ,u8 data_num);

void Uart5_Init(u32 br_num);
void Uart5_IRQ(void);
void Uart5_Send(unsigned char *DataToSend ,u8 data_num);

#endif
