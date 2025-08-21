#ifndef __USART_H_
#define	__USART_H_

#include "stm32f10x.h"
#include <stdio.h>

void USART1_Init(void);
void USART2_Init(void);
static void USART2_SendByte( USART_TypeDef * pUSARTx, uint8_t ch );
void USART2_SendStr_length( USART_TypeDef * pUSARTx, uint8_t *str,uint32_t strlen );
void USART2_SendString( USART_TypeDef * pUSARTx, uint8_t *str);
void USART2_SendString( USART_TypeDef * pUSARTx, uint8_t *str);
char *get_rebuff(uint16_t *len) ;
void clean_rebuff(void);
void USART2_Process(void);
#endif
