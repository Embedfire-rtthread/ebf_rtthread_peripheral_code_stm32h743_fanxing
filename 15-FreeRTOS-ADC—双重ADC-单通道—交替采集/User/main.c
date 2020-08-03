/**
  *********************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2018-xx-xx
  * @brief   RT-Thread 3.0 + STM32 工程模版
  *********************************************************************
  * @attention
  *
  * 实验平台:野火 STM32H743 开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :https://fire-stm32.taobao.com
  *
  **********************************************************************
  */ 
 
/*
*************************************************************************
*                             包含的头文件
*************************************************************************
*/ 
#include "board.h"
#include "rtthread.h"


/*
*************************************************************************
*                               变量
*************************************************************************
*/
/* 定义线程控制块 */
static rt_thread_t led1_thread = RT_NULL;
static rt_thread_t key_thread = RT_NULL;

extern __IO uint32_t ADC_ConvertedValue;
__IO uint16_t ADC_ConvertedValueLocal[2];
float ADC_vol[2];
/*
*************************************************************************
*                             函数声明
*************************************************************************
*/
static void led1_thread_entry(void* parameter);
static void key_thread_entry(void* parameter);

/*
*************************************************************************
*                             main 函数
*************************************************************************
*/
/**
  * @brief  主函数
  * @param  无
  * @retval 无
  */
int main(void)
{
    /* 
	 * 开发板硬件初始化，RTT系统初始化已经在main函数之前完成，
	 * 即在component.c文件中的rtthread_startup()函数中完成了。
	 * 所以在main函数中，只需要创建线程和启动线程即可。
	 */
	rt_kprintf("这是一个[野火]-STM32-RTT-ADC—独立模式-多通道-DMA实验！\n");

	led1_thread =                          /* 线程控制块指针 */
    rt_thread_create( "led1",              /* 线程名字 */
                      led1_thread_entry,   /* 线程入口函数 */
                      RT_NULL,             /* 线程入口函数参数 */
                      512,                 /* 线程栈大小 */
                      3,                   /* 线程的优先级 */
                      20);                 /* 线程时间片 */
                   
    /* 启动线程，开启调度 */
   if (led1_thread != RT_NULL)
        rt_thread_startup(led1_thread);
    else
        return -1;

		key_thread =                          /* 线程控制块指针 */
    rt_thread_create( "key",              /* 线程名字 */
                      key_thread_entry,   /* 线程入口函数 */
                      RT_NULL,             /* 线程入口函数参数 */
                      512,                 /* 线程栈大小 */
                      4,                   /* 线程的优先级 */
                      20);                 /* 线程时间片 */
                   
    /* 启动线程，开启调度 */
   if (key_thread != RT_NULL)
        rt_thread_startup(key_thread);
    else
        return -1;
}

/*
*************************************************************************
*                             线程定义
*************************************************************************
*/

static void led1_thread_entry(void* parameter)
{	
    while (1)
    {
			//双ADC交替采样：ADC_MASTER的采样值存放在低16位；
			//              ADC_SLAVE的采样值存放在高16位；
			ADC_ConvertedValueLocal[0] = (uint16_t)ADC_ConvertedValue;
			ADC_ConvertedValueLocal[1] = (uint16_t)((ADC_ConvertedValue&0xFFFF0000)>>16);
			
			ADC_vol[0] =(float)((uint16_t)ADC_ConvertedValueLocal[0]*3.3/65536); 
			ADC_vol[1] =(float)((uint16_t)ADC_ConvertedValueLocal[1]*3.3/65536);    
			
			printf("\r\n The current AD value = 0x%08X \r\n", ADC_ConvertedValueLocal[0]); 
			printf("\r\n The current AD value = 0x%08X \r\n", ADC_ConvertedValueLocal[1]);     
			//读取转换的AD值
			printf("\r\n The current ADC1 value = %f V \r\n",ADC_vol[0]); 
			printf("\r\n The current ADC2 value = %f V \r\n",ADC_vol[1]); 
			printf("\r\n\r\n");

			rt_thread_delay(500);   /* 延时500个tick */	
    }
}

static void key_thread_entry(void* parameter)
{
	while (1)
	{
			if( Key_Scan(KEY1_GPIO_PORT, KEY1_PIN) != KEY_OFF)
			{
				LED2_TOGGLE;
				rt_kprintf("LED状态翻转\n");
			}
			rt_thread_delay(10);   /* 延时10个tick */		 	
	}
}

/********************************END OF FILE****************************/
