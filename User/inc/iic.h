#ifndef __IIC_H_
#define __IIC_H_

#include "stm32f10x.h"
  
/* 模拟IIC相关宏定义: SCL-PB6  SDA-PB7 */
#define SCL_H() (GPIOB->ODR |= 1 << 6)         // 时钟线高电平
#define SCL_L() (GPIOB->ODR &= ~(1 << 6))      // 时钟线低电平
#define SDA_H() (GPIOB->ODR |= 1 << 7)         // 数据线高电平
#define SDA_L() (GPIOB->ODR &= ~(1 << 7))      // 数据线低电平

#define READ_SDA() (GPIOB->IDR & 1 << 7)       // 读取数据线电平状态

// SDA设置为浮空输入
#define SDA_IN()	  {GPIOB->MODER &=~(3 << (7*2));GPIOB->MODER |= 0 << (7*2);}   
// SDA设置为输出	
#define SDA_OUT() 	{GPIOB->MODER &=~(3 << (7*2));GPIOB->MODER |= 1 << (7*2);}   

//应答信号
#define ACK_N 1
#define ACK_Y 0

void IIC_GPIO_Init(void);
void IIC_Start(void);
void IIC_Stop(void);
void IIC_ACK_Send(u8 ack);
u8 IIC_Wait_Ack(void);
void IIC_Send_Byte(u8 data);
u8 IIC_Read_OneByte(void);
#endif
