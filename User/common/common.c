/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   �����жϽ��ղ���
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ�� iSO-MINI STM32 ������ 
  * ��̳    :http://www.chuxue123.com
  * �Ա�    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */ 
#include "stm32f10x.h"
#include "misc.h"
#include "stm32f10x_it.h"
#include "bsp_usart.h"
#include "bsp_TiMbase.h" 
#include "bsp_SysTick.h"
#include "string.h"
#include "bsp_rtc.h"
#include "bsp_date.h"
#include "WatchDog.h"
#include "common.h"
#include "gprs.h"
#include "SPI_Flash.h"
#include "SensorInit.h"

extern struct liquid_set  DeviceConfig;     //������������Ϣ�ṹ��
extern struct SMS_Config_RegPara   ConfigData; //FLASH��ȡ�ṹ
/*******************************************************************************
* Function Name  : XX
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void gotoSleep(uint16_t SendCount)
{
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
	PWR_BackupAccessCmd(ENABLE);
	
  
	if(SendCount<1) 
	{
		SendCount = 1;                         //��ֹ��������������豸���ڴ�������״̬���޷����ѣ�Ŀǰ���������ʱ��Ϊ24h
	}
	//Sms_Consult();                          //���Ķ��ţ��������ò����������д�����
	//RTC_SetAlarm(RTC_GetCounter()+10);   //�ɼ�ʱ�䵽��ʼ����
	RTC_SetAlarm(RTC_GetCounter()+SendCount * 60 );      //���ߣ�����ʹ��
	
	RTC_WaitForLastTask();
  if(DeviceConfig.MessageSetFlag ==1)	     //�ж����޸Ĳ���ʱ��������д��洢��
	{
//    ParaUpdateCheck();                     //д��������
#if DEBUG_TEST	
		printf("\r\nDevice Set Upload Success According to Messages!!\r\n");                    //����ʹ��
#endif
  }

	Delay_ms(800);  
	PowerOFF_GPRS();                         //�ر�433ģ���Դ
	Delay_ms(100);
  PowerOFF_UltrasonicSensor();             //�رճ�����̽ͷ��Դ
	Delay_ms(100);
  PowerOFF_485();                          //�ر�485��Դ
	Delay_ms(100);
  PowerOFF_Flash();                        //�ر�Flash��Դ
	
	
 /*********************************�������Ĵ���д��FLASH***************************************************/			
	DeviceConfig.WorkNum  += 1;
	ConfigData.WorkNum_Byte [0]=DeviceConfig.WorkNum>>8;
	ConfigData.WorkNum_Byte [1]=DeviceConfig.WorkNum & 0x00FF;
	DataWrite_To_Flash(0,12,0,ConfigData.WorkNum_Byte,2);                             //���ѹ����Ĵ���д��FLASH			
	
	printf("\r\nCollectPeriod:%d-----SendCount:%d\r\n",DeviceConfig.CollectPeriod,DeviceConfig.SendCount);    //����ʹ��
	printf("SLEEP OK!\r\n%dmin later wakeup!!",SendCount);	        //Ӧ�ÿ������������˻�

	
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR , ENABLE);
//	PWR_WakeUpPinCmd(ENABLE); //ʹ��WAKE-UP�ܽ�
	PWR_EnterSTANDBYMode();	
}
