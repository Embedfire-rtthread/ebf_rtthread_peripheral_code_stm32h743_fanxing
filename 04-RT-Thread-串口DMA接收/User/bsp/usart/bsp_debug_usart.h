#ifndef __DEBUG_USART_H
#define	__DEBUG_USART_H

#include "stm32h7xx.h"
#include <stdio.h>


//串口波特率
#define DEBUG_USART_BAUDRATE                    115200
//引脚定义
/*******************************************************/
#define DEBUG_USART                             USART1
#define DEBUG_USART_CLK_ENABLE()                __USART1_CLK_ENABLE();

#define DEBUG_USART_RX_GPIO_PORT                GPIOB
#define DEBUG_USART_RX_GPIO_CLK_ENABLE()        __GPIOB_CLK_ENABLE()
#define DEBUG_USART_RX_PIN                      GPIO_PIN_7
#define DEBUG_USART_RX_AF                       GPIO_AF7_USART1

#define DEBUG_USART_TX_GPIO_PORT                GPIOB
#define DEBUG_USART_TX_GPIO_CLK_ENABLE()        __GPIOB_CLK_ENABLE()
#define DEBUG_USART_TX_PIN                      GPIO_PIN_6
#define DEBUG_USART_TX_AF                       GPIO_AF7_USART1

#define DEBUG_USART_IRQHandler                  USART1_IRQHandler
#define DEBUG_USART_IRQ                 		    USART1_IRQn
/************************************************************/
//DMA
#define DEBUG_USART_DMA_CLK_ENABLE()      		__HAL_RCC_DMA2_CLK_ENABLE()	
#define DEBUG_USART_DMA_CHANNEL           		DMA_CHANNEL_4
#define DEBUG_USART_DMA_STREAM            		DMA2_Stream5

#define  USART_RBUFF_SIZE            1000 

extern uint8_t Usart_Rx_Buf[USART_RBUFF_SIZE];//缓冲区

extern UART_HandleTypeDef UartHandle;      //UART句柄
extern DMA_HandleTypeDef  DMA_Handle;      //DMA句柄
void callback( struct __DMA_HandleTypeDef * hdma);
void Uart_DMA_Rx_Data(void);
void Debug_USART_Config(void);
void USART_DMA_Config(void);
void USART_Init(void);
#endif /* __USART1_H */
