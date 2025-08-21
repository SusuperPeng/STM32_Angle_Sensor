#include "main.h"

/****************************************************************************************
�������ƣ� USART1_Config
�������ܣ� ����1��ʼ��
�����βΣ� ��
��������ֵ�� ��
����������
	�����ʣ�115200
	Ӳ�����ӣ�
	USART1_TX -- PA9
****************************************************************************************/
void USART1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	USART_Cmd(USART1, ENABLE);	    
}

/*fgetc��scanf��ײ�ĵ��ú���*/
int fgetc(FILE*file)
{
	while(!(USART1->SR & 1<<5));  /*�ȴ����մ��ڵ�����*/
	return USART1->DR;
}
/*fputc��printf��ײ�ĵ��ú���*/
int fputc(int data, FILE* file)
{
	USART1->DR = data;             //��������
	while(!(USART1->SR & (1<<7))); /*�ȴ����ʹ��ڵ�����*/
	return data;
}
