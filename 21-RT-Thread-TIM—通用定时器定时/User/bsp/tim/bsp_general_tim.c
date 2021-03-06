/**
  ******************************************************************************
  * @file    bsp_basic_tim.c
  * @author  STMicroelectronics
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   通用定时器定时范例
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火  STM32 H743 开发板  
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */
  
#include "./tim/bsp_general_tim.h"
#include "./led/bsp_led.h" 
TIM_HandleTypeDef TIM_Base;


 /**
  * @brief  通用定时器 TIMx,x[2-5,12-14,15-17]中断优先级配置
  * @param  无
  * @retval 无
  */
static void TIMx_NVIC_Configuration(void)
{
    HAL_NVIC_SetPriority(GENERAL_TIM_IRQ, 0, 0);
    HAL_NVIC_EnableIRQ(GENERAL_TIM_IRQ);
}

                         
/*
 * 注意：TIM_Base_InitTypeDef结构体里面有5个成员，TIM6和TIM7的寄存器里面只有
 * TIM_Prescaler和TIM_Period，所以使用TIM6和TIM7的时候只需初始化这两个成员即可，
 * 另外三个成员是通用定时器和高级定时器才有.
 *-----------------------------------------------------------------------------
 * Prescaler         都有
 * CounterMode			 TIMx,x[6,7]没有，其他都有（基本定时器）
 * Period            都有
 * ClockDivision     TIMx,x[6,7]没有，其他都有(基本定时器)
 * RepetitionCounter TIMx,x[1,8]才有(高级定时器)
 *-----------------------------------------------------------------------------
 */
static void TIM_Mode_Config(void)
{
    GENERAL_TIM_CLK_ENABLE();
     
    TIM_Base.Instance = GENERAL_TIM;
    /* 累计 TIM_Period个后产生一个更新或者中断*/		
    //当定时器从0计数到10000-1，即为10000次，为一个定时周期
    TIM_Base.Init.Period = 10000 - 1;
    //定时器时钟源TIMxCLK = 2 * PCLK1  
    //				PCLK1 = HCLK / 4 
    //				=> TIMxCLK=HCLK/2=SystemCoreClock/2=240MHz
    // 设定定时器频率为=TIMxCLK/(TIM_Prescaler+1)=10000Hz
    TIM_Base.Init.Prescaler =  24000 - 1;
		// 计数方式
	  TIM_Base.Init.CounterMode=TIM_COUNTERMODE_UP;
	  // 采样时钟分频
	  TIM_Base.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    // 初始化定时器TIM
    HAL_TIM_Base_Init(&TIM_Base);
    // 开启定时器更新中断
    HAL_TIM_Base_Start_IT(&TIM_Base);
}

/**
  * @brief  初始化基本定时器定时，1s产生一次中断
  * @param  无
  * @retval 无
  */
void TIMx_Configuration(void)
{
    TIMx_NVIC_Configuration(); 
	
  	TIM_Mode_Config();
}



/*********************************************END OF FILE**********************/

