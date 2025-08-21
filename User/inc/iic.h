#ifndef __IIC_H_
#define __IIC_H_

#include "stm32f10x.h"
  
/* ģ��IIC��غ궨��: SCL-PB6  SDA-PB7 */
#define SCL_H() (GPIOB->ODR |= 1 << 6)         // ʱ���߸ߵ�ƽ
#define SCL_L() (GPIOB->ODR &= ~(1 << 6))      // ʱ���ߵ͵�ƽ
#define SDA_H() (GPIOB->ODR |= 1 << 7)         // �����߸ߵ�ƽ
#define SDA_L() (GPIOB->ODR &= ~(1 << 7))      // �����ߵ͵�ƽ

#define READ_SDA() (GPIOB->IDR & 1 << 7)       // ��ȡ�����ߵ�ƽ״̬

// SDA����Ϊ��������
#define SDA_IN()	  {GPIOB->MODER &=~(3 << (7*2));GPIOB->MODER |= 0 << (7*2);}   
// SDA����Ϊ���	
#define SDA_OUT() 	{GPIOB->MODER &=~(3 << (7*2));GPIOB->MODER |= 1 << (7*2);}   

//Ӧ���ź�
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
