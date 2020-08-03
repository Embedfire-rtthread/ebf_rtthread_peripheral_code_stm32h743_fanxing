#ifndef __ADC_H
#define	__ADC_H

#include "stm32h7xx.h"


//引脚定义
#define RHEOSTAT_ADC_PIN1                           GPIO_PIN_3  
#define RHEOSTAT_ADC_PIN2                           GPIO_PIN_4
#define RHEOSTAT_ADC_PIN3                           GPIO_PIN_5
#define RHEOSTAT_ADC_PIN4                           GPIO_PIN_6
#define RHEOSTAT_ADC_PIN5                           GPIO_PIN_7
#define RHEOSTAT_ADC_PIN6                           GPIO_PIN_8

#define RHEOSTAT_ADC_GPIO_PORT                      GPIOF                     
#define RHEOSTAT_ADC_GPIO_CLK_ENABLE()              __GPIOF_CLK_ENABLE()

// ADC 序号宏定义
#define RHEOSTAT_ADC1                        ADC3
#define RHEOSTAT_ADC1_CLK_ENABLE()           __ADC3_CLK_ENABLE()


#define RHEOSTAT_ADC_CHANNEL1                 ADC_CHANNEL_5
#define RHEOSTAT_ADC_CHANNEL2                 ADC_CHANNEL_9
#define RHEOSTAT_ADC_CHANNEL3                 ADC_CHANNEL_4
#define RHEOSTAT_ADC_CHANNEL4                 ADC_CHANNEL_8
#define RHEOSTAT_ADC_CHANNEL5                 ADC_CHANNEL_3
#define RHEOSTAT_ADC_CHANNEL6                 ADC_CHANNEL_7

#define Rheostat_ADC12_IRQ                    ADC_IRQn

void ADC_Init(void);
void Rheostat_ADC_NVIC_Config(void);
uint16_t ADC_GetValue(void);
#endif /* __ADC_H */
