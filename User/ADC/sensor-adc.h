/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32f10x_adc.h"
#include "stdio.h"
/** @addtogroup SingleChannelContinuous
  * @{
  */ 


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define USE_DMA_Transfer
#define ADC1_DR_Address    ((uint32_t)0x4001244C)
#define DEALDATA   1   //数据四舍五入  0可取消

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


//ErrorStatus HSEStartUpStatus;
    
/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);

void ADCconfig(void);                                //配置ADC
float ReadSensorData(uint16_t data);                 //将ADC采集的整数数据转换为小数
float Filter(float Original[],int num);              //对采集的数据进行滤波

