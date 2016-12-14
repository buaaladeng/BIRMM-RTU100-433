/**
  ******************************************************************************
  * @file    main.c
  * @author  casic 203
  * @version V1.0
  * @date    2015-07-27
  * @brief   
  * @attention
  *
  ******************************************************************************
  */ 
#include "stm32f10x.h"
#include "misc.h"
#include "stdlib.h"
#include "time.h"
#include "stm32f10x_it.h"
#include "bsp_usart.h"
#include "bsp_TiMbase.h" 
#include "modbus.h"
#include "bsp_SysTick.h"
#include "string.h"
#include "gprs.h"
#include "bsp_rtc.h"
#include "bsp_date.h"
#include "WatchDog.h"
#include "AiderProtocol.h"
#include "SPI_Flash.h"
#include "common.h"
#include "DS2780.h"
#include "433_Wiminet.h"
#include  "test.h"
#include  "sensor-adc.h"
#include  "SensorInit.h"

#define COLLECT_NUM 10                  //传感器一次工作采集的数据量
#define SNESOR_WAIT 180                  //传感器预热时间
/***********************************变量声明与定义***************************************************/ 
struct    rtc_time             systmtime;                  //RTC时钟设置结构体
struct    liquid_set           DeviceConfig ={0x00};       //配置信息结构体
struct    SMS_Config_RegPara   ConfigData ={0x00};       //与FLASH交互的结构体，方便在系统进入休眠模式之前写入FLASH

uint16_t  WWDOG_Feed =0x1FFF;                  //窗口看门狗复位周期为：XX*1.8s = 7.6min
char      PowerOffReset =0;                    //掉电重启标志位

u8        Send_Request_flag=0;                    //发送数据至上位机后受到反馈
uint8_t   LiquidDataSend_Flag =0;                 //数据发送完成标志位
uint8_t   SensorCollectNumber=0;                 //一次上报需要进行数据采集的数量         采集数量=上报周期/采集周期
uint8_t   DataCollectCount ;                 //数据采集计数器   每次初始化需要读取该值，采集完数据后写入FLASH

struct    SenserData   PerceptionData;             //传感器数据   “u16采集时间  float传感器数据    u8采集数量”
struct    SenserData   ReportUp[MAX_COLLECTNUM];   //上传传感器的数据

char      Usart1_recev_buff[300] ={'\0'};      //USART1接收缓存
uint16_t  Usart1_recev_count =0;               //USART1发送计数器

unsigned char   Usart2_send_buff[SENDBUFF_SIZE]={'\0'};          //433模块发送数组 
unsigned char   Usart2_recev_buff[RECEIVEBUFF_SIZE]={'\0'};      //433模块接收数组

unsigned char   Usart3_send_buff[300]={'\0'};    //3G模块发送数组

extern  char      Usart3_recev_buff[1000];       //3G模块与433模块共用接收数据组
extern  uint16_t  Usart3_recev_count;            //3G模块与433模块接收数据量

        uint8_t   DMA_UART3_RECEV_FLAG =0;                //USART3 DMA接收标志变量
extern  uint8_t   DMA_USART3_RecevBuff[RECEIVEBUFF_SIZE];    //3G模块与433模块共用接收数据组
extern  struct    DMA_USART3_RecevConfig   DMA_USART3_RecevIndicator;    //DMA指示

extern __IO uint16_t ADC1ConvertedValue;                    //ADC采集的数据
uint16_t Sensor_PowderOn;
uint16_t Sensor_PowderTime;

/***********************************函数申明***************************************************/  
u16   DMA_UART3_RecevDetect(u8* pDeviceID, u16 sNodeAddress);     //USART3接收数据监测与数据解析
int   DMA_UART3_RecevDataGet(void);                               //获取串口3的DMA数据
void  PeripheralInit( void);                                      //初始化外围设备
void  Display(void);                                              //进行信息打印
unsigned int Get_Rand_Num(void);                                 //获取随机数据


/***********************获取随机数据***********************************************/
unsigned int Get_Rand_Num(void)
{    
	    unsigned int rand_num;
      srand((unsigned)RTC_GetCounter());                                     //设置种子
		  rand_num = rand()%1000;                                          //获取0~1000内任意随机数
	    return(rand_num);
}
/*******************************************************************************
* Function Name  : int  DMA_UART3_RecevDataGet(void)
* Description    : 接收433传输的数据，Usart3_recev_buff[],
* Input          : None
* Output         : None
* Return         : 采集到数据的长度
*******************************************************************************/
int  DMA_UART3_RecevDataGet(void)
{
   int i=0,j=0;
	 u16 DMA_RecevLength =0;
	
	 memset(Usart3_recev_buff, 0x00, sizeof(Usart3_recev_buff));
	 DMA_USART3_RecevIndicator.CurrentDataStartNum = DMA_USART3_RecevIndicator.NextDataStartNum ;
	  
	 i = RECEIVEBUFF_SIZE - DMA_GetCurrDataCounter(DMA1_Channel6);
	 if(DMA_USART3_RecevIndicator.DMA_RecevCount <i)
	 {
     DMA_RecevLength =i -DMA_USART3_RecevIndicator.DMA_RecevCount;
   }
	 else
	 {
     DMA_RecevLength = RECEIVEBUFF_SIZE -DMA_USART3_RecevIndicator.DMA_RecevCount + i;
   }
   DMA_USART3_RecevIndicator.DMA_RecevCount = i;
	
	 if((DMA_USART3_RecevIndicator.CurrentDataStartNum + DMA_RecevLength-1) < RECEIVEBUFF_SIZE)
	 {
     DMA_USART3_RecevIndicator.CurrentDataEndNum =DMA_USART3_RecevIndicator.CurrentDataStartNum +DMA_RecevLength-1;     
   }
	 else
	 {
     DMA_USART3_RecevIndicator.CurrentDataEndNum =(DMA_USART3_RecevIndicator.CurrentDataStartNum +DMA_RecevLength-1) -RECEIVEBUFF_SIZE;  
   }
#if DEBUG_TEST 	 
	 printf("\r\nDMA UART2 Recev Data Start Num:%d----End Num: %d\r\n",DMA_USART3_RecevIndicator.CurrentDataStartNum,DMA_USART3_RecevIndicator.CurrentDataEndNum);    //数据起始位置与终止位置
#endif
	 if(DMA_USART3_RecevIndicator.CurrentDataEndNum ==(RECEIVEBUFF_SIZE-1))
	 {
	   DMA_USART3_RecevIndicator.NextDataStartNum = 0;
   }
	 else
	 {
		 DMA_USART3_RecevIndicator.NextDataStartNum = DMA_USART3_RecevIndicator.CurrentDataEndNum + 1;
   }	
   /*************************************Data Copy*********************************************************/
   if(DMA_RecevLength !=0)
	 {
     j =DMA_USART3_RecevIndicator.CurrentDataStartNum;
		 if(DMA_USART3_RecevIndicator.CurrentDataEndNum >DMA_USART3_RecevIndicator.CurrentDataStartNum)
		 {
			 for(i=0; i<DMA_RecevLength; i++,j++)
			 {
					Usart3_recev_buff[i] =DMA_USART3_RecevBuff[j];	
			 }
		 }
		 else
		 {
			 for(i=0; i<DMA_RecevLength; i++)
			 {
					if( j<(RECEIVEBUFF_SIZE-1) )
					{
						 Usart3_recev_buff[i] =DMA_USART3_RecevBuff[j];
						 j++;				
					}
					else if( j==(RECEIVEBUFF_SIZE-1) )
					{
						 Usart3_recev_buff[i] =DMA_USART3_RecevBuff[j];
						 j =0;				 
					}
			  } 
      }
    }
	  return DMA_RecevLength;
}
/*******************************************************************************
* Function Name  : u16  DMA_UART3_RecevDetect(u8* pDeviceID, u16 sNodeAddress)
* Description    : 检测DMA接收的数据
* Input          : 节点的ID与 地址
* Output         : 接收状态
* Return         : 分析结果
*******************************************************************************/
u16  DMA_UART3_RecevDetect(u8* pDeviceID, u16 sNodeAddress)
{
	int DataLength =0;
	int i=0;
	u16 StateFlag =0;                         //数据分析结果标志                        
	
  if(DMA_UART3_RECEV_FLAG==1)
  {

		 DataLength = DMA_UART3_RecevDataGet();             //获取DMA数据
		 if(DataLength>0)
		 {
#if DEBUG_TEST 
				printf("\r\nDataLength:%d\r\n", DataLength);             //将数据长度进行输出显示
#endif
			  for(i=0;i<DataLength;i++)
			  {
#if DEBUG_TEST 
             printf(" %.2x ",Usart3_recev_buff[i]);               //将接收的数据进行显示
#endif
        }

        StateFlag = Receive_Data_Analysis(pDeviceID, sNodeAddress);	      //进行433接收数据分析，并输出分析结果
        			
		 }
		 else
		 {
#if DEBUG_TEST 
        printf("\r\nNo data\r\n");
#endif
     }
		 DMA_Cmd(DMA1_Channel6, DISABLE);           //关闭DMA
		 memset(DMA_USART3_RecevBuff,0x00,RECEIVEBUFF_SIZE);   //将串口3的接收数组清空
     DMA_UART3_RECEV_FLAG =0;
		 DMA_Cmd(DMA1_Channel6, ENABLE);            //开启DMA
		 USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);   // 进行串口2配置
  } 
	return StateFlag;
}





/*******************************************************************************
* Function Name  : void SenserDataCollect(struct SenserData* pGetData, u8* pDevID, u16 NodeAddr)
* Description    : 传感器采集数据,并进行发送
* Input          : 传感器数据形式，节点ID，节点地址
* Output         : None
* Return         : None
*******************************************************************************/
void SenserDataCollect(struct SenserData* pGetData, u8* pDevID, u16 NodeAddr)
{
      
			u32       TimCount_Current =0;
	    uint8_t   sensordata[7]={0x00};                        //将传感器数据写入FLASH,数组包含“采集时间，传感器数据，采集数量”
   		u8        i=0;                                         //测试使用
			float sensor_collect[COLLECT_NUM];
			uint16_t WorkTime;
		  struct TagStruct AlarmTAG;
#if DEBUG_TEST 
      printf("\r\nthe sensor start collect data!\r\n");       //测试显示
#endif

/////////////////////////////////进行多次采集数据，采集完毕后，进行处理并输出存储

			Sensor_PowderTime=RTC_GetCounter();
			WorkTime=Sensor_PowderTime-Sensor_PowderOn;
#if DEBUG_TEST 
			printf("THE SENSOR HAS WORKED ALREADY %d S",WorkTime);
#endif
			if(WorkTime < SNESOR_WAIT)
			{
				Delay_ms((SNESOR_WAIT-WorkTime)*1000);                      //进行传感器预热
			}

			for(i=0;i<COLLECT_NUM;i++)
			{
				sensor_collect[i]=ReadSensorData(ADC1ConvertedValue);                     //每间隔1秒进行采集

				printf("\r\nNO:%d  sensor_collect: %0.2f",i,sensor_collect[i]);            //将采集的数据进行打印

				Delay_ms(1000);
			}
    	pGetData->Ch4Data.Data_Float  = Filter(sensor_collect,COLLECT_NUM);                                                 //
	    pGetData->CollectTime =(DeviceConfig.Time_Hour)*60 +(DeviceConfig.Time_Min);      //数据采集时间
    	pGetData->DataCount =  DataCollectCount+1;                                        //当前采集的个数
			Delay_ms(500);   
/*********************************将采集的数据个数写入FLASH***************************************************/
      DataCollectCount = pGetData->DataCount;
			ConfigData.CollectNum = DataCollectCount;
			DataWrite_To_Flash(0,9,0,&ConfigData.CollectNum,1);                           //将已采集的数据个数写入FLASH
			
/*********************************将采集的传感器数据写入FLASH***************************************************/
			sensordata[0]= pGetData->CollectTime >> 8;
			sensordata[1]= pGetData->CollectTime & 0xff;                                 //采集时间
      for(i=0;i<4;i++)
      sensordata[2+i] =	pGetData->Ch4Data.Data_Hex[i];                             //采集的传感器数据
      sensordata[6] = pGetData->DataCount;			                                   //采集的数量
      DataWrite_To_Flash(1,pGetData->DataCount,0,(uint8_t*)sensordata,sizeof(sensordata));       //数据从Sector1开始写入
			
			//printf("\r\nthe sensor data already send NO.%d to flash!\r\n",pGetData->DataCount);
      PowerOFF_UltrasonicSensor();

//////////////如果监测的气体浓度达到了报警阈值，立即报警，无需等到上报时间的到来///////////////////////		
				if((pGetData->Ch4Data.Data_Float >=DeviceConfig.HighAlarmLevel .Data_Float) ||(pGetData->Ch4Data.Data_Float>=DeviceConfig.LowAlarmLevel .Data_Float)  )
				{
					
					
					AlarmTAG.OID_Command=(DeviceConfig.CollectPeriod<<11)+pGetData->CollectTime  +((0xC0 + REPORTDATA )<<24);
					AlarmTAG.OID_Command=ntohl(AlarmTAG.OID_Command);
					AlarmTAG.Width=4;
					for(i=0;i<AlarmTAG.Width;i++)
					AlarmTAG.Value[i]=pGetData->Ch4Data.Data_Hex[i];                                 //组成报警的TAG
					SendDataToServ(TRAPREQUEST,&AlarmTAG,1,Usart3_send_buff,pDevID);                //433模块主动上传数据
		      //AlarmTrap(Usart3_send_buff, pDevID, NodeAddr, &PerceptionData);             //3G模块主动上传数据
		      Send_Request_flag = DMA_UART3_RecevDetect(pDevID, NodeAddr);       //每隔5秒发送一次，共发送3次，收到数据时
					if(Send_Request_flag == 1)
		      {
		      Send_Request_flag = 0;	
		    
		      }

				  LiquidDataSend_Flag=1;                 //数据成功发送到服务器标志变量
			 }		
///////////////////////////////////////////////////////////////////////////////////////////////////////			
			 if(DataCollectCount  < SensorCollectNumber)
			 {    
			      gotoSleep(DeviceConfig.CollectPeriod  );               //进入睡眠状态，等待下一次采集任务
        }                                                         //如果未达到上报时间，以采集时间睡眠 
			 else
				if( DataCollectCount  == SensorCollectNumber )      //采集完毕，进行数据上报 ;当采集次数到达时，先判断是否阈值上报过一次
				{
		        Delay_ms(100); 
				    DeviceConfig.BatteryCapacity = DS2780_Test(); //监测电池电量
				
				
		      SendDataToServ(TRAPREQUEST,NULL,0,Usart3_send_buff,pDevID);                //433模块主动上传数据
					
		      Send_Request_flag = DMA_UART3_RecevDetect(pDevID, NodeAddr);                //每隔5秒发送一次，共发送5次，收到数据时断开TCP连接
		      if(Send_Request_flag == 1)
		       {
		      Send_Request_flag = 0;	
		   		 }
 
				LiquidDataSend_Flag=1;                 //液位数据成功发送到服务器标志变量
	      
         }
}




/*******************************************************************************
* Function Name  : int main(void)
* Description    : 主函数
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int main( void )
{
	u8   i=0;
	u8   DeviceID[6] = SENSORID ;                                 //初始化设备ID号
  u16  NodeAddr =0x0000;                                        //提取设备ID号最后面两个字节作为节点地址
  NodeAddr=DeviceID[4]*256 +DeviceID[5];
  PeripheralInit();                            //硬件初始化
/***********************************对采集数量进行处理,一次最多传15个数*****************************************************/
	SensorCollectNumber = DeviceConfig.SendCount / DeviceConfig.CollectPeriod ;
	if(SensorCollectNumber>15)
	{ SensorCollectNumber=15;	}
  
   DataRead_From_Flash(0,9,0, &(DeviceConfig.CollectNum ) ,1);            //读取已采集的数据量 
   DataCollectCount = DeviceConfig.CollectNum ;
   if(DataCollectCount > SensorCollectNumber)                     //如果已采集的数量大于需要采集的数量，将已采集数量归零
	 {
		 DataCollectCount=0;
	 }
#if DEBUG_TEST 
	printf("\r\n The  Collect Number is %d \r\n",DeviceConfig.CollectNum );
#endif	
	SendDataToServ(STARTUPREQUEST,NULL,0,Usart3_send_buff, DeviceID);  //开机上报信息
  //DeviceStartupRequest(Usart3_send_buff, DeviceID, NodeAddr);      //开机上报信息
	Delay_ms(500);  
  Test_SendToServ(DeviceID);                                                  //测试接口
	while(1)
  {
 
		  WWDOG_Feed =0x1FFF;                            //窗口看门狗喂狗,定时4分20秒，第二字节约1秒变化一次，即0x09AF变到0x099F约耗时1秒

 	    for(i=0;i<5;i++)
		{
			 Delay_ms(200);                                //检测是否有数据传输过来
			 if(DMA_UART3_RECEV_FLAG==1)                   //查询数据接收情况
			 {
				 DMA_UART3_RecevDetect(DeviceID, NodeAddr);   
				 break;
			 }
    }	

			if(DataCollectCount < SensorCollectNumber)   //当已采集的数据小于共采集数量
			{
#if DEBUG_TEST 				
				printf("\r\nThe Sensor Has DataCollectCount is %d \r\n",DataCollectCount);  
#endif				
				SenserDataCollect(&PerceptionData, DeviceID, NodeAddr);        //获取传感器数据
      }
			else                                                             
			{	
				if(DataCollectCount == SensorCollectNumber)         //完成数据采集后，开始上传数据
				DataCollectCount=0;                                   //对数据进行清零，重新采集
		     
		  }
			
			if(LiquidDataSend_Flag ==1)            
			{ 
				gotoSleep(DeviceConfig.CollectPeriod );
			}
	}
	
}

/*******************************************************************************
* Function Name  : void PeripheralInit( void )
* Description    : 初始化端口及外设
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void PeripheralInit( void )
{

	WWDG_Init(0x7F,0X5F,WWDG_Prescaler_8); 	//首先开启窗口看门狗，计数器值为7f,窗口寄存器为5f,分频数为8	
	USART1_Config();      /* USART1 配置模式为 9600 8-N-1，  中断接收 */    //用于串口调试助手
	USART2_Config();      /* USART2 配置模式为 9600 8-N-1，中断接收 */    //用于433通信方式
	USART4_Config();      /* UART4  配置模式为 38400 8-N-1，  中断接收 */    //用于传感器通信
  UART_NVIC_Configuration();
	USART2_DMA_Config();
//USART3_DMA_Config();
//	TIM3_Configuration();       /* 定时器TIM3参数配置 */	
//	TIM3_NVIC_Configuration();  /* 设置定时器TIM3的中断优先级 */
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, DISABLE);     //暂时关闭定时器TIM3
	
	RTC_NVIC_Config();                 /* 配置RTC秒中断优先级 */
	RTC_CheckAndConfig(&systmtime);
  SysTick_Init();
	ADCconfig();                       //进行ADC初始化
	//PowerON_Flash();                   //打开Flash电源  
	PowerON_GPRS();                    //打开GPRS模块电源
	PowerON_UltrasonicSensor();       //打开传感器
	Sensor_PowderOn=RTC_GetCounter();   //记录传感器工作开始时间
	Delay_ms(500);
  Display();                          //进行出厂显示

  
  if (PowerOffReset ==1)     
  {
#if DEBUG_TEST
    printf("\r\n掉电重启，重新初始化库仑计\r\n");                 //测试使用
#endif

		Set_register_ds2780();    //掉电后对库仑计重新初始化
	  set_ACR(1000);           //掉电后对库仑计重新初始化
	  DS2780_CapacityInit();    //掉电后重新写电池容量
  	PowerOffReset =0;       //掉电重启标志变量复位
  }
#if CONFIG > 0
  SensorInit();
#endif
	ConfigData_Init(&DeviceConfig);


	Power_SX1278_Init();       //打开433模块
	
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
  GPIO_SetBits(GPIOC,GPIO_Pin_5);                  //433模块SET管脚拉高，切换到接收模式
	Delay_ms(100);
  GPIO_SetBits(GPIOB,GPIO_Pin_0);                //433模块EN管脚拉低，切换到高速模式
	Delay_ms(100);
	USART_GetFlagStatus(USART2,USART_FLAG_TC);       //串口硬件复位之后，发送首字节之前，先读一下USART_SR,防止数据发送时首字节被覆盖 
 

}

/*******************************************************************************
* Function Name  : void Display( void )
* Description    : 打印出厂信息
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Display(void)
{
	u32       TimCount_Current =0;
	u8   DeviceID[6] = SENSORID ; 
	TimCount_Current = RTC_GetCounter();
	printf("\r\n********* 北京航天科瑞电子有限公司 *********");
	printf("\r\n************* 燃气智能监测终端 ***********");
	printf("\r\n************* BIRMM-RTU100 ***********");
	printf("\r\n************* 设备ID号：%x %x %x %x %x %x***********",DeviceID[0],DeviceID[1],DeviceID[2],DeviceID[3],DeviceID[4],DeviceID[5]);
	printf("\r\n************** 硬件类型: 433版本 **********");
	printf("\r\n************** 硬件版本号: %x ***********",HARDVERSION);
	printf("\r\n************** 软件版本号: %x **************\r\n\r\n",SOFTVERSION);
	
	Time_Display(TimCount_Current,&systmtime); 
	DeviceConfig.Time_Sec  =systmtime.tm_sec;
	DeviceConfig.Time_Min  =systmtime.tm_min;
	DeviceConfig.Time_Hour =systmtime.tm_hour;
	DeviceConfig.Time_Mday =systmtime.tm_mday;		
	DeviceConfig.Time_Mon  =systmtime.tm_mon;
	DeviceConfig.Time_Year =systmtime.tm_year-2000; //对上传年份去基数修正				
		    

printf("**************当前时间:%0.4d-%0.2d-%0.2d %0.2d:%0.2d:%0.2d***********\r\n",systmtime.tm_year,systmtime.tm_mon,     //测试使用
				systmtime.tm_mday,systmtime.tm_hour,systmtime.tm_min,systmtime.tm_sec);     //测试使用
  Delay_ms(3000);
}




#ifdef  DEBUG
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(u8* file, u32 line)
{ 
  /* User can add his own implementation to report the file name and line number */
 
  printf("\n\r Wrong parameter value detected on\r\n");
  printf("       file  %s\r\n", file);
  printf("       line  %d\r\n", line);
    
  /* Infinite loop */
  /* while (1)
  {
  } */
}
#endif
/*********************************************END OF FILE**********************/
