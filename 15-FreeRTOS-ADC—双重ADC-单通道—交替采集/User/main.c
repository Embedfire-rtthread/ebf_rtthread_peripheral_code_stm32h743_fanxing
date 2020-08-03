/**
  *********************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2018-xx-xx
  * @brief   RT-Thread 3.0 + STM32 ����ģ��
  *********************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ�� STM32H743 ������ 
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :https://fire-stm32.taobao.com
  *
  **********************************************************************
  */ 
 
/*
*************************************************************************
*                             ������ͷ�ļ�
*************************************************************************
*/ 
#include "board.h"
#include "rtthread.h"


/*
*************************************************************************
*                               ����
*************************************************************************
*/
/* �����߳̿��ƿ� */
static rt_thread_t led1_thread = RT_NULL;
static rt_thread_t key_thread = RT_NULL;

extern __IO uint32_t ADC_ConvertedValue;
__IO uint16_t ADC_ConvertedValueLocal[2];
float ADC_vol[2];
/*
*************************************************************************
*                             ��������
*************************************************************************
*/
static void led1_thread_entry(void* parameter);
static void key_thread_entry(void* parameter);

/*
*************************************************************************
*                             main ����
*************************************************************************
*/
/**
  * @brief  ������
  * @param  ��
  * @retval ��
  */
int main(void)
{
    /* 
	 * ������Ӳ����ʼ����RTTϵͳ��ʼ���Ѿ���main����֮ǰ��ɣ�
	 * ����component.c�ļ��е�rtthread_startup()����������ˡ�
	 * ������main�����У�ֻ��Ҫ�����̺߳������̼߳��ɡ�
	 */
	rt_kprintf("����һ��[Ұ��]-STM32-RTT-ADC������ģʽ-��ͨ��-DMAʵ�飡\n");

	led1_thread =                          /* �߳̿��ƿ�ָ�� */
    rt_thread_create( "led1",              /* �߳����� */
                      led1_thread_entry,   /* �߳���ں��� */
                      RT_NULL,             /* �߳���ں������� */
                      512,                 /* �߳�ջ��С */
                      3,                   /* �̵߳����ȼ� */
                      20);                 /* �߳�ʱ��Ƭ */
                   
    /* �����̣߳��������� */
   if (led1_thread != RT_NULL)
        rt_thread_startup(led1_thread);
    else
        return -1;

		key_thread =                          /* �߳̿��ƿ�ָ�� */
    rt_thread_create( "key",              /* �߳����� */
                      key_thread_entry,   /* �߳���ں��� */
                      RT_NULL,             /* �߳���ں������� */
                      512,                 /* �߳�ջ��С */
                      4,                   /* �̵߳����ȼ� */
                      20);                 /* �߳�ʱ��Ƭ */
                   
    /* �����̣߳��������� */
   if (key_thread != RT_NULL)
        rt_thread_startup(key_thread);
    else
        return -1;
}

/*
*************************************************************************
*                             �̶߳���
*************************************************************************
*/

static void led1_thread_entry(void* parameter)
{	
    while (1)
    {
			//˫ADC���������ADC_MASTER�Ĳ���ֵ����ڵ�16λ��
			//              ADC_SLAVE�Ĳ���ֵ����ڸ�16λ��
			ADC_ConvertedValueLocal[0] = (uint16_t)ADC_ConvertedValue;
			ADC_ConvertedValueLocal[1] = (uint16_t)((ADC_ConvertedValue&0xFFFF0000)>>16);
			
			ADC_vol[0] =(float)((uint16_t)ADC_ConvertedValueLocal[0]*3.3/65536); 
			ADC_vol[1] =(float)((uint16_t)ADC_ConvertedValueLocal[1]*3.3/65536);    
			
			printf("\r\n The current AD value = 0x%08X \r\n", ADC_ConvertedValueLocal[0]); 
			printf("\r\n The current AD value = 0x%08X \r\n", ADC_ConvertedValueLocal[1]);     
			//��ȡת����ADֵ
			printf("\r\n The current ADC1 value = %f V \r\n",ADC_vol[0]); 
			printf("\r\n The current ADC2 value = %f V \r\n",ADC_vol[1]); 
			printf("\r\n\r\n");

			rt_thread_delay(500);   /* ��ʱ500��tick */	
    }
}

static void key_thread_entry(void* parameter)
{
	while (1)
	{
			if( Key_Scan(KEY1_GPIO_PORT, KEY1_PIN) != KEY_OFF)
			{
				LED2_TOGGLE;
				rt_kprintf("LED״̬��ת\n");
			}
			rt_thread_delay(10);   /* ��ʱ10��tick */		 	
	}
}

/********************************END OF FILE****************************/
