/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "stm32h7xx.h"
#include "stm32h7xx_it.h"
#include "board.h"
#include "rtthread.h"
#include "./usart/bsp_usart_dma.h"
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/******************************************************************************/
/*            Cortex Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
//void HardFault_Handler(void)
//{
//  /* USER CODE BEGIN HardFault_IRQn 0 */

//  /* USER CODE END HardFault_IRQn 0 */
//  while (1)
//  {
//  }
//  /* USER CODE BEGIN HardFault_IRQn 1 */

//  /* USER CODE END HardFault_IRQn 1 */
//}

/**
* @brief This function handles Memory management fault.
*/
//void MemManage_Handler(void)
//{
//  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

//  /* USER CODE END MemoryManagement_IRQn 0 */
//  while (1)
//  {
//  }
//  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

//  /* USER CODE END MemoryManagement_IRQn 1 */
//}

/**
* @brief This function handles Pre-fetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
//void PendSV_Handler(void)
//{
//  /* USER CODE BEGIN PendSV_IRQn 0 */

//  /* USER CODE END PendSV_IRQn 0 */
//  /* USER CODE BEGIN PendSV_IRQn 1 */

//  /* USER CODE END PendSV_IRQn 1 */
//}
/* 外部定义消息队列控制块 */
extern rt_mq_t test_mq;

uint32_t send_data1 = 1;
uint32_t send_data2 = 2;
/*********************************************************************************
  * @ 函数名  ： KEY1_IRQHandler
  * @ 功能说明： 中断服务函数
  * @ 参数    ： 无  
  * @ 返回值  ： 无
  ********************************************************************************/
void KEY1_IRQHandler(void)
{
  /* 进入中断 */
  rt_interrupt_enter();
	//确保是否产生了EXTI Line中断
	if(__HAL_GPIO_EXTI_GET_IT(KEY1_INT_GPIO_PIN) != RESET) 
	{
		//清除中断标志位
		__HAL_GPIO_EXTI_CLEAR_IT(KEY1_INT_GPIO_PIN);     
	}
  
  /* 离开中断 */
  rt_interrupt_leave();
}

/*********************************************************************************
  * @ 函数名  ： KEY1_IRQHandler
  * @ 功能说明： 中断服务函数
  * @ 参数    ： 无  
  * @ 返回值  ： 无
  ********************************************************************************/
void KEY2_IRQHandler(void)
{
  /* 进入中断 */
  rt_interrupt_enter();
	//确保是否产生了EXTI Line中断
	if(__HAL_GPIO_EXTI_GET_IT(KEY2_INT_GPIO_PIN) != RESET) 
	{
		
		//清除中断标志位
		__HAL_GPIO_EXTI_CLEAR_IT(KEY2_INT_GPIO_PIN);     
	}
  
  /* 离开中断 */
  rt_interrupt_leave();
}

/*********************************************************************************
  * @ 函数名  ： DEBUG_USART_IRQHandler
  * @ 功能说明： 串口中断服务函数
  * @ 参数    ： 无  
  * @ 返回值  ： 无
  ********************************************************************************/
void DEBUG_USART_IRQHandler(void)
{
  /* 进入中断 */
  rt_interrupt_enter();

	if(__HAL_UART_GET_FLAG(&UartHandle,UART_FLAG_IDLE)!=RESET)
	{
		//空闲中断产生，说明接收完成，停止串口接收，此函数会停止DMA的接收，并调用HAL_UART_AbortReceiveCpltCallback
		HAL_UART_AbortReceive_IT(&UartHandle);
		//清除空闲中断标志位
		__HAL_UART_CLEAR_IDLEFLAG(&UartHandle);
	}
	
  /* 离开中断 */
  rt_interrupt_leave();
}

void DMA2_Stream5_IRQHandler(void)
{
  HAL_DMA_IRQHandler(UartHandle.hdmarx);
}

void KEY3_IRQHandler(void)
{
	__HAL_GPIO_EXTI_CLEAR_IT(KEY3_INT_GPIO_PIN);
}
void KEY4_IRQHandler(void)
{
	__HAL_GPIO_EXTI_CLEAR_IT(KEY4_INT_GPIO_PIN);
}
void KEY5_IRQHandler(void)
{
	__HAL_GPIO_EXTI_CLEAR_IT(KEY5_INT_GPIO_PIN);
}
/**
* @brief This function handles System tick timer.
*/
//void SysTick_Handler(void)
//{
//  /* USER CODE BEGIN SysTick_IRQn 0 */

//  /* USER CODE END SysTick_IRQn 0 */
//  HAL_IncTick();
//  HAL_SYSTICK_IRQHandler();
//  /* USER CODE BEGIN SysTick_IRQn 1 */

//  /* USER CODE END SysTick_IRQn 1 */
//}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
