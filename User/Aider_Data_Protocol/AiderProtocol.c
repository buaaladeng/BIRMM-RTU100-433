#include "stm32f10x.h"
#include "gprs.h"
#include "bsp_SysTick.h"
#include "modbus.h"
#include "string.h"
#include "AiderProtocol.h"
#include "433_Wiminet.h"
#include "bsp_rtc.h"
#include "SPI_Flash.h"
#include "API-Platform.h"

//#include  "bsp_date.h"
////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
1、由于数据帧中存在回车换行符，因此通过3G模块AT命令上传数据的长度有待确认。
2、关于一帧上传数据的数据采集方式有两种方案，分别是：
一、数据采集时间在两次数据上传时间之间平均分配，此时需要采集一次数据进行休眠，同时将采集到的数据存入外部存储区，
并用外存或者备份寄存器记录数据采集次数。
二、数据采集间隔远小于数据上传时间间隔，定时唤醒，唤醒之后立即采集数据，N组数据采集完成之后立即上传到服务器，
上传完成，并完成其他相关处理后，再次进入待机模式。
目前默认采用方案二
*/

/***********************************变量声明与定义***************************************************/ 
extern  struct liquid_set  DeviceConfig;          //设备配置信息结构体
extern struct    SMS_Config_RegPara   ConfigData;     //定义的下发配置参数，HEX格式，方便在系统进入休眠模式之前写入FLASH
struct   DataFrame         StartupSend;
struct   SpecialDataFrame  TrapSend;
struct   DataFrame         GetRespSend;
extern struct   SenserData  ReportUp[MAX_COLLECTNUM];
extern u8     DataCollectCount;                   //数据采集计数器

extern struct    rtc_time        systmtime;        //系统时间
extern uint8_t   Usart3_send_buff[300];
extern uint8_t   DMA_UART3_RECEV_FLAG ;      //USART3 DMA接收标志变量
extern unsigned char   Usart2_send_buff[SENDBUFF_SIZE];     //433发送数据组
extern unsigned int Get_Rand_Num(void);                                 //获取随机数据

/***********************************函数申明***************************************************/ 
extern u16  DMA_UART3_RecevDetect(u8* pDeviceID, u16 sNodeAddress);


/*******************************************************************************
* Function Name  : UploadFlash
* Description    : 进行FLASH写入
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint8_t  UploadFlash(char* pSetPara, uint8_t  InstructCode)
{
  uint8_t   j=0;
  uint8_t   Counter=0;
#if DEBUG_TEST	
	switch(InstructCode)
	{
		
		case 2:          //存储采集周期
		{   
			 for(j=0;j<strlen(pSetPara);j++)
			 printf("\r\nCollect Period Before Write Into Flash:-%x-\r\n",pSetPara[j]);
			 DataWrite_To_Flash(0,2,0,(uint8_t*)pSetPara,2);      //将采集周期写入Flash
			 DeviceConfig.MessageSetFlag =1;                    //标示已完成参数修改	
       //printf("\r\n collect period: %s\r\n",pSetPara);			          //测试使用						
			 return 1;
		}
		case 3:          //存储上传周期
		{  
			 for(j=0;j<strlen(pSetPara);j++)
			 printf("\r\nSend Period Before Write Into Flash:-%x-\r\n",pSetPara[j]);
			 DataWrite_To_Flash(0,3,0,(uint8_t*)pSetPara,2);      //将上传次数写入Flash
			 DeviceConfig.MessageSetFlag =1;                      //标示已完成参数修改	
       //printf("\r\n send period: %s\r\n",pSetPara);			          //测试使用						
			 return 1;
		}
		
		case 4:            //存储报警号码
		{
			  //手机号码的判别条件，后续需要增加。
			 DataWrite_To_Flash(0,4,0,(uint8_t*)pSetPara,strlen(pSetPara));   //将报警号码写入Flash 
       DeviceConfig.MessageSetFlag =1;                                  //标示当前液位仪参数通过短信修改		 
       printf("\r\n Alarm Phone Number:%s\r\n",pSetPara);			          //测试使用
			 return 1;
		}
		
		case 5:              //存储服务器IP
		{

			 for(j=0,Counter=0;j<strlen(pSetPara);j++)                       //调用函数已经做了防溢出处理,strlen(pSetPara)<=15
			 {
          if((pSetPara[j]>='0')&&(pSetPara[j]<='9'))
					{
//             CharTemp[j] =pSetPara[j];
						 ;
          }
				  else if(pSetPara[j]=='.')                                    //分隔符统计
					{
						 Counter++;
          }
					else
					{
              break;
          }
       }
			 if(Counter==3)
			 {
          DataWrite_To_Flash(0,5,0, (uint8_t*)pSetPara,strlen(pSetPara));   //将服务器IP写入Flash 
       }
			 else
			 {
          printf("\r\nInput Server IP ERROR!!\r\n");
       }
			 DeviceConfig.MessageSetFlag =1;     //标示当前液位仪参数通过短信修改		 
			  printf("\r\n srever ip: %s\r\n",pSetPara);			          //测试使用
			 return 1;
			 
		}
		
		case 6:           //存储服务器端口号
		{
			 DataWrite_To_Flash(0,6,0,(uint8_t*)pSetPara,strlen(pSetPara));      //将服务器端口号写入Flash
			 DeviceConfig.MessageSetFlag =1;     //标示当前液位仪参数通过短信修改	
        printf("\r\n server port: %s\r\n",pSetPara);			          //测试使用			
			 return 1;
			
		}
		case 7:            //存储低浓度报警阈值
		{
			 
		   DataWrite_To_Flash(0,7,0,(uint8_t*)pSetPara , 4);   //将报警阈值（Hex格式）写入Flash 
			 DeviceConfig.MessageSetFlag =1;                       //标示已完成参数修改	
			 printf("\r\n Low Alarm Threshold: %s\r\n",pSetPara);			          //测试使用
			 return 1;
		}
		
	
		case 8:          //存储第一次上报时间
		{  
			 for(j=0;j<strlen(pSetPara);j++)
			 printf("\r\n---------%x----------\r\n",pSetPara[j]);
		   DataWrite_To_Flash(0,8,0,(uint8_t*)pSetPara,2);      //将第一次上报时间写入Flash
			 DeviceConfig.MessageSetFlag =1;                     //标示已完成参数修改	
       printf("\r\n start collect time: %s\r\n",pSetPara);			          //测试使用						
			 return 1;
		}
		
		case 9:          //存储一次采集的数量
		{   
			 for(j=0;j<strlen(pSetPara);j++)
			 printf("\r\n---------%x----------\r\n",pSetPara[j]);
		   DataWrite_To_Flash(0,9,0,(uint8_t*)pSetPara,2);      //将一次采集数量写入Flash
			 DeviceConfig.MessageSetFlag =1;                     //标示已完成参数修改	
       printf("\r\n collect number: %s\r\n",pSetPara);			          //测试使用						
			 return 1;
		}
		case 10:          //存储重传次数
		{
		   DataWrite_To_Flash(0,10,0,(uint8_t*)pSetPara,1);      //将重传次数写入Flash
			 DeviceConfig.MessageSetFlag =1;     //标示当前液位仪参数通过短信修改		 
			 //printf("\r\nBefore Write Into Flash:-%x-\r\n",pSetPara[0]);			          //测试使用			
			 return 1;
		}
		
			case 11:            //存储高浓度报警阈值
		{
			 
		   DataWrite_To_Flash(0,11,0,(uint8_t*)pSetPara , 4);   //将报警阈值（Hex格式）写入Flash 
			 DeviceConfig.MessageSetFlag =1;     //标示当前液位仪参数通过短信修改		 
			 printf("\r\n High Alarm Threshold: %s\r\n",pSetPara);			          //测试使用
			 return 1;
		}
		
		default:
		{
			 printf("\r\nInstruct Code ERROR !!\r\n");
		   return 0;
		}
  }
#endif
}

/*******************************************************************************
* Function Name  : WakeupResponse
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void WakeupResponse(u8* pSendBuff, u8* pDeviceID, u16 NodeAddress)
{
			
	struct   DataFrame  WakeupSend;
	struct   DataFrame* pDataFrame =&WakeupSend;
	char*    pChar =NULL;
  u8       SendCounter =1;     //发送次数计数器
	u16      RecevFlag =0;       //服务器数据接收标志变量
	u16      CrcData=0;
	u8       i=0,j=0;
	u8       ValidLength =0;     //有效数据长度，指1个Tag空间中实际存储有效数据的长度。
	const u8 PreLength   =16;    //帧前缀长度，指从帧前导码到OID序列（或者Tag序列）的数据长度，包括帧前导码，而不包括OID序列（或者Tag序列）
	const u8 CoreLength  =12;    //关键信息长度，指数据净荷部分长度字段指示的数值，去除OID序列（或者Tag序列）后剩余数据的长度
	
	pChar =(char*)pDataFrame;
	memset(pChar,0x00,sizeof(struct DataFrame));   //初始化结构体
	WakeupSend.Preamble =0xA3;
	WakeupSend.Version  =0x20;
	for(i=0;i<6;i++)
	{
    WakeupSend.DeviceID[i] =pDeviceID[i];
  }
	WakeupSend.RouteFlag =0x01;
	WakeupSend.NodeAddr  =ntohs( NodeAddress);               //调整为网络序，即高字节在前，低字节在后
	WakeupSend.PDU_Type  =(0x0B<<8)+(1<<7)+4;
	WakeupSend.PDU_Type  =ntohs(WakeupSend.PDU_Type);        //调整为网络序，即高字节在前，低字节在后
	WakeupSend.Seq       =1;
	
	WakeupSend.TagList[0].OID_Command =ntohl(DEVICE_WAKEUP); //调整为网络序，即高字节在前，低字节在后
	WakeupSend.TagList[0].Width =1;
	WakeupSend.TagList[0].Value[0]=1;
	WakeupSend.Tag_Count =1;
	WakeupSend.Length =CoreLength + WakeupSend.TagList[0].Width +6;  
	WakeupSend.Length =ntohs(WakeupSend.Length );
	WakeupSend.CrcCode=0xffff;                                        //CRC字段赋初值

	memcpy(pSendBuff,pChar,PreLength);
	for(i=0,j=PreLength;i<WakeupSend.Tag_Count;i++)
	{
    
		pChar = (char*)&(WakeupSend.TagList[i].OID_Command);   
		ValidLength =WakeupSend.TagList[i].Width+6;            //计算1个Tag实际占用的字节空间
		if((j+ValidLength) >=(SENDBUFF_SIZE-3))                //防止指针溢出
		{
       break;
    }
		WakeupSend.TagList[i].Width =ntohs(WakeupSend.TagList[i].Width); //调整为网络序，即高字节在前，低字节在后
	  memcpy((pSendBuff+j),pChar,ValidLength); 
		j = j+ValidLength;
		
  }
	pSendBuff[j++] = WakeupSend.CrcCode &0xff;
	pSendBuff[j++] = WakeupSend.CrcCode>>8;
	
  CrcData = CRC16(pSendBuff, j );   // Update the CRC value
	WakeupSend.CrcCode =CrcData;
	pSendBuff[j-2] = CrcData&0xff;   //CRC低字节在前
	pSendBuff[j-1] = CrcData>>8;     //CRC高字节在后

	if(DeviceConfig.RetryNum>0)
	{
     SendCounter =DeviceConfig.RetryNum;
  }
	else
	{
     SendCounter =1;
  }
	for(i=0;i< SendCounter;i++)
	{
     RecevFlag = SendMessage(pSendBuff, j, pDeviceID, NodeAddress);
//		 Delay_ms(2000); 
		if(RecevFlag != 0)
			break;
		//	 Delay_ms(2000); 
  }
	printf("\r\n----Length:%d----\r\n",j);   //测试使用
	for(i=0;i<j;i++)
	{
    printf("%x ",pSendBuff[i]);   //测试使用
  }
}

/**************************************************************************
* Function Name  : SendDataToServ()
* Description    : 通信协议的实现
* Input          : 操作类型，发送数组，设备ID
* Output         : None
* Return         : None
**************************************************************************/
void SendDataToServ(CommunicateType CommType,struct TagStruct TagList[],uint8_t TagNum,u8* pSendBuff, u8* pDeviceID)
{
  DeviceType DevType 				= DEVICE;        //初始化设备类别
	ReportDataType	RDataType =	REPORTDATA;    //初始化上报类型
  SendType   SType = TYPE_433; 	            //通信方式为433通信
	struct   DataFrame  SendData;             //发送数据格式
	struct   DataFrame* pDataFrame =&SendData;  //发送数据指针
	struct   TagStruct* pTag=TagList;           //接收TAG指针
	
	char*    pChar =NULL;
  u16      RecevFlag =0;           //服务器数据接收标志变量
	uint16_t NodeAddress;            //设备地址
	uint8_t  SendCounter=1;          //发送次数
	u16      CrcData=0;
	u8       i=0,j=0;
	uint8_t  RevTagNum = 0;           //TAG的数量
	unsigned int rand_num;        //随机延迟变量 范围：0~1000
	uint16_t CollectTime;         //采集时间
	uint8_t  readdata[7];         //读取FLASH内数据

	u8       ValidLength =0;     //有效数据长度，Tag空间中实际存储有效数据的长度。
	const u8 PreLength   =16;    //帧前缀长度，指从帧前导码到OID序列（或者Tag序列）的数据长度，包括帧前导码，而不包括OID序列（或者Tag序列）
	const u8 CoreLength  =12;    //关键信息长度，指数据净荷部分长度字段指示的数值，去除OID序列（或者Tag序列）后剩余数据的长度
	const u8 TagPreLength=6;     //TAG的OID长度为4+TAG数据长度2
	uint8_t  TagLength=0;        //TAG的总长度
	uint8_t  Offset=0;           //指针偏移量
	u8       TotalLength;        //除去CRC的数据总长度
	NodeAddress = pDeviceID[4]*256 +pDeviceID[5];
	pChar =(char*)pDataFrame;
	
	memset(pChar,0x00,sizeof(struct DataFrame));   //初始化结构体
	SendData.Preamble =0xA3;
	SendData.Version  = SOFTVERSION;
	for(i=0;i<6;i++)
	{
   SendData.DeviceID[i] = pDeviceID[i];
  }
	SendData.RouteFlag =SType;                              //数据发送方式
	SendData.NodeAddr  =ntohs( NodeAddress);               //调整为网络序，即高字节在前，低字节在后
	SendData.PDU_Type  =(CommType<<8)+(1<<7)+DEVICE;
	SendData.PDU_Type  =ntohs(SendData.PDU_Type);        //调整为网络序，即高字节在前，低字节在后
	SendData.Seq       =1;
	
	
	if(pTag!=NULL)
		{
				RevTagNum=TagNum;                     //如果指针不空，则证明有数据传输过来，接收TAG的数量
				for(i=0;i<RevTagNum;i++)
				SendData.TagList[i] = TagList[i];
		} 
	else
		{
	  switch(CommType)
	 {
				
		case GETRESPONSE:
		{

			break;
		}
		
		case TRAPREQUEST:
		{ 
			
				SendData.TagList[0].OID_Command = ntohl(DEVICE_QTY);   //调整为网络序，即高字节在前，低字节在后
				SendData.TagList[0].Width =1;                         //
				SendData.TagList[0].Value[0] =DeviceConfig.BatteryCapacity;  //数据上传时的电池剩余电量		  
	
				SendData.TagList[1].OID_Command= ntohl(SYSTERM_DATA);    //调整为网络序，即高字节在前，低字节在后
				SendData.TagList[1].Width =3;                            //
				SendData.TagList[1].Value[0] =systmtime.tm_year-2000;       //系统日期，年
				SendData.TagList[1].Value[1] =systmtime.tm_mon ;            //系统日期，月
				SendData.TagList[1].Value[2] =systmtime.tm_mday ;           //系统日期，日
			
			  
/***********************************不同的设备需要进行修改上报TAG*********************************************/	
			
				for(i=0; i< DataCollectCount ;i++)    //对每一个上报数据Tag赋值
				{
	///////////////////////////////////////将FLASH内的数据读取出来进行上报////////////////////////////////
				DataRead_From_Flash(1,i+1,0, readdata ,sizeof(readdata));
				CollectTime = readdata[0] *256 + readdata[1];

				SendData.TagList[2+i].OID_Command = (DeviceConfig.CollectPeriod<<11)+CollectTime  +((0xC0 + RDataType )<<24);     //业务类型5，根据协议修改
				SendData.TagList[2+i].OID_Command = ntohl(SendData.TagList[2+i].OID_Command); //调整为网络序，即高字节在前，低字节在后
				SendData.TagList[2+i].Width =4;
				for(j=0;j<SendData.TagList[2+i].Width;j++)	
					{
						SendData.TagList[2+i].Value[j]=readdata[2+j];                             //采集到的气体浓度数据
					}
				}
/*****************************************************************************************************/	
				RevTagNum =2+DataCollectCount;                     //TAG的总数等于电量+日期+上报数据
			  
		 	  break;
				
		}			
		case ONLINEREQUEST:
		{
			
			break;
		}
			
		case WAKEUPRESPONSE:
		{
	  SendData.TagList[0].OID_Command =ntohl(DEVICE_WAKEUP); //调整为网络序，即高字节在前，低字节在后
	  SendData.TagList[0].Width =1;
	  SendData.TagList[0].Value[0]=1;
	  RevTagNum =1;
	  		
			break;
		}
			
		case STARTUPREQUEST:
		{
			
			SendData.TagList[0].OID_Command =ntohl(DEVICE_STATE); //调整为网络序，即高字节在前，低字节在后
			SendData.TagList[0].Width =1;
			SendData.TagList[0].Value[0]=1;
			RevTagNum =1;

			break;
		}
			
		default:
		{
			RevTagNum =0;
		}

	}	
	}
	 
  		for(i=0; i<RevTagNum;i++)
			{
			TagLength = SendData.TagList[i].Width +TagPreLength;
			ValidLength += TagLength;
		  
			}
			SendData.Length =CoreLength + ValidLength;  
			SendData.Length =ntohs(SendData.Length );
			memcpy(pSendBuff,pChar,PreLength );                             //进行除TAG的数据复制
			
			SendData.CrcCode =0xffff;                                        //CRC字段赋初值
			TotalLength = PreLength + ValidLength;
			
			Offset=	PreLength;		
			for(i=0; i<RevTagNum;i++)
			{
			
		  pChar = (char*)&(SendData.TagList [i].OID_Command); 
      TagLength = SendData.TagList[i].Width +TagPreLength;				//进行TAG的复制 
			SendData.TagList[i].Width=ntohs(SendData.TagList[i].Width);
     	memcpy((pSendBuff+Offset),pChar,TagLength );
			
			Offset+=TagLength;
			
			}
			
			CrcData = CRC16(pSendBuff, TotalLength);   // Update the CRC value
	    SendData.CrcCode =CrcData;
	    pSendBuff[TotalLength+1] = CrcData&0xff;   //CRC低字节在前
	    pSendBuff[TotalLength+2] = CrcData>>8;     //CRC高字节在后
			
		
	if(DeviceConfig.RetryNum>0)
	{
     SendCounter =DeviceConfig.RetryNum;
  }
	else
	{
     SendCounter =1;
  }
	for(i=0;i<SendCounter;i++)
	{
     RecevFlag =SendMessage(pSendBuff, TotalLength+2, pDeviceID, NodeAddress);
		 if(RecevFlag != 0)     //成功发送
		 {
#if DEBUG_TEST 
				printf("\r\nSend  Data Success!\r\n"); //测试使用
#endif
			  break;     
     }
		  rand_num=Get_Rand_Num();          //获取随机延迟时间
#if DEBUG_TEST 
		 	printf("the rand_num is %d",rand_num);  
#endif
		   Delay_ms(3000+2*rand_num);                           //延迟随机发送
		  }
	
	
}


/*******************************************************************************
* Function Name  : DeviceStartupRequest
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

void DeviceStartupRequest(u8* pSendBuff, u8* pDeviceID, u16 NodeAddress)
{
	DeviceType DevType = BIRMM_RTU100;		
//	struct   DataFrame  StartupSend;
	struct   DataFrame* pDataFrame =&StartupSend;
	char*    pChar =NULL;
	unsigned int rand_num;
	u16      RecevFlag =0;       //服务器数据接收标志变量
	u16      CrcData=0;
	u8       i=0,j=0;
	u8       SendCounter =1;     //发送次数计数器
	u8       ValidLength =0;     //有效数据长度，指1个Tag空间中实际存储有效数据的长度。
	const u8 PreLength   =16;    //帧前缀长度，指从帧前导码到OID序列（或者Tag序列）的数据长度，包括帧前导码，而不包括OID序列（或者Tag序列）
	const u8 CoreLength  =12;    //关键信息长度，指数据净荷部分长度字段指示的数值，去除OID序列（或者Tag序列）后剩余数据的长度
	
	pChar =(char*)pDataFrame;
	memset(pChar,0x00,sizeof(struct DataFrame));   //初始化结构体
	StartupSend.Preamble =0xA3;
	StartupSend.Version  =0x20;
	for(i=0;i<6;i++)
	{
    StartupSend.DeviceID[i] =pDeviceID[i];
  }
	StartupSend.RouteFlag =0x01;
	StartupSend.NodeAddr  =ntohs( NodeAddress);              //调整为网络序，即高字节在前，低字节在后
	StartupSend.PDU_Type  =(8<<8)+(1<<7)+4;
	StartupSend.PDU_Type  =ntohs(StartupSend.PDU_Type);      //调整为网络序，即高字节在前，低字节在后
	StartupSend.Seq       =1;
	StartupSend.TagList[0].OID_Command =ntohl(DEVICE_STATE); //调整为网络序，即高字节在前，低字节在后
	StartupSend.TagList[0].Width =1;
	StartupSend.TagList[0].Value[0]=1;
	StartupSend.Tag_Count =1;
	StartupSend.Length =CoreLength + StartupSend.TagList[0].Width +6;  
	StartupSend.Length =ntohs(StartupSend.Length );
	StartupSend.CrcCode=0xffff;                                        //CRC字段赋初值

	memcpy(pSendBuff,pChar,PreLength);
	for(i=0,j=PreLength;i<StartupSend.Tag_Count;i++)
	{
    
		pChar = (char*)&(StartupSend.TagList[i].OID_Command);   
		ValidLength =StartupSend.TagList[i].Width+6;           //计算1个Tag实际占用的字节空间
		if((j+ValidLength) >=(SENDBUFF_SIZE-3))                //防止指针溢出
		{
       break;
    }
		StartupSend.TagList[i].Width =ntohs(StartupSend.TagList[i].Width); //调整为网络序，即高字节在前，低字节在后
	  memcpy((pSendBuff+j),pChar,ValidLength); 
		j = j+ValidLength;
		
  }
	pSendBuff[j++] = StartupSend.CrcCode &0xff;
	pSendBuff[j++] = StartupSend.CrcCode>>8;
	
  CrcData = CRC16(pSendBuff, j );   // Update the CRC value
	StartupSend.CrcCode =CrcData;
	pSendBuff[j-2] = CrcData&0xff;   //CRC低字节在前
	pSendBuff[j-1] = CrcData>>8;     //CRC高字节在后

	if(DeviceConfig.RetryNum>0)
	{
     SendCounter =DeviceConfig.RetryNum;
  }
	else
	{
     SendCounter =1;
  }
	for(i=0;i<SendCounter;i++)
	{
     RecevFlag =SendMessage(pSendBuff, j, pDeviceID, NodeAddress);
		 if(RecevFlag ==0x0984)     //成功接收到TrapResponse
		 {
        #if DEBUG_TEST	 
				printf("\r\nReceive TrapResponse success!\r\n"); //测试使用
				#endif
			  break;     
     }
		   rand_num=Get_Rand_Num();
		 	printf("the rand_num is %d",rand_num);
		   Delay_ms(2000+2*rand_num);                        //随机发送
		 //Delay_ms(2000+2*Get_Rand_Num());                                 //获取随机数据); 
  }
//	printf("\r\n----Length:%d----\r\n",j);   //测试使用
//	for(i=0;i<j;i++)
//	{
//     printf("-%x-",pSendBuff[i]);   //测试使用
//  }
}

/*******************************************************************************
* Function Name  : TrapRequest
* Description    : 主动上传请求
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void  TrapRequest(u8* pSendBuff, u8* pDeviceID, u16 NodeAddress, struct SenserData* pObtainData)
{
		
//	struct   SpecialDataFrame  TrapSend;
	struct   SpecialDataFrame* pDataFrame  =&TrapSend;
	char*    pChar =NULL;
	u16      RecevFlag =0;       //服务器数据接收标志变量
	uint8_t  readdata[7];        //读取FLASH内数据
	unsigned int rand_num;       //0~1000之间的随机数据
	u16      CrcData=0;
	u8       SendCounter =1;     //发送次数计数器
	u8       i=0,j=0;
	u8       Offset=0;           //发送缓存地址偏移变量
	u8       ValidLength =0;     //有效数据长度，指1个Tag空间中实际存储有效数据的长度。
	u8       LengthKey =0;       //数据净荷部分长度字段指示的数值
	const u8 PreLength   =16;    //帧前缀长度，指从帧前导码到OID序列（或者Tag序列）的数据长度，包括帧前导码，而不包括OID序列（或者Tag序列）
	const u8 CoreLength  =12;    //关键信息长度，指数据净荷部分长度字段指示的数值，去除Tag序列后剩余数据的长度
	
  printf("\r\nFUNCTION: TrapRequest start! \r\n");                    //

	pChar =(char*)pDataFrame;
	memset(pChar,0x00,sizeof(struct SpecialDataFrame));   //初始化结构体
	TrapSend.Preamble =0xA3;
	TrapSend.Version  =0x20;
	for(i=0;i<6;i++)
	{
    TrapSend.DeviceID[i] =pDeviceID[i];
  }
	TrapSend.RouteFlag =0x01;
	TrapSend.NodeAddr  =ntohs( NodeAddress);              //调整为网络序，即高字节在前，低字节在后
	TrapSend.PDU_Type  =(4<<8)+(1<<7)+4;
	TrapSend.PDU_Type  =ntohs(TrapSend.PDU_Type);         //调整为网络序，即高字节在前，低字节在后
	TrapSend.Seq       =1;
	LengthKey = CoreLength;
	
	TrapSend.BattEnergy.OID_Command = ntohl(DEVICE_QTY);   //调整为网络序，即高字节在前，低字节在后
	TrapSend.BattEnergy.Width =1;                         //
	TrapSend.BattEnergy.Value[0] =DeviceConfig.BatteryCapacity;  //数据上传时的电池剩余电量
	LengthKey = LengthKey+6+TrapSend.BattEnergy.Width;
	
	TrapSend.SysTime.OID_Command= ntohl(SYSTERM_DATA);    //调整为网络序，即高字节在前，低字节在后
	TrapSend.SysTime.Width =3;                            //
	TrapSend.SysTime.Value[0] =systmtime.tm_year-2000;       //系统日期，年
	TrapSend.SysTime.Value[1] =systmtime.tm_mon ;            //系统日期，月
	TrapSend.SysTime.Value[2] =systmtime.tm_mday ;           //系统日期，日
  LengthKey = LengthKey+6+TrapSend.SysTime.Width;

	for(i=0,TrapSend.Tag_Count=0,TrapSend.Length =LengthKey ;i< DataCollectCount ;i++)    //对每一个数据Tag赋值
	{
		///////////////////////////////////////将FLASH内的数据读取出来进行上报////////////////////////////////
		DataRead_From_Flash(1,i+1,0, readdata ,sizeof(readdata));
		pObtainData[i].CollectTime = readdata[0] *256 + readdata[1];
		for(j=0;j<4;j++)
		pObtainData[i].Ch4Data .Data_Hex [j] = readdata[2+j];
		
    TrapSend.TagList[i].OID_Data = (DeviceConfig.CollectPeriod<<11)+pObtainData[i].CollectTime  +(0xC5<<24);     //业务类型5，根据协议修改
		TrapSend.TagList[i].OID_Data = ntohl(TrapSend.TagList[i].OID_Data); //调整为网络序，即高字节在前，低字节在后
		TrapSend.TagList[i].Width =4;
		TrapSend.TagList[i].PerceptionData.Data_F   =pObtainData[i].Ch4Data .Data_Float  ;  //采集到的气体浓度数据
    
		TrapSend.Tag_Count++ ;                                             //对传感器数据Tag进行计数，不包括采集日期Tag
		TrapSend.Length =TrapSend.Length + TrapSend.TagList[i].Width +6;  
  }

	/////////////////////////////////////////////////////////////////////////////////////////////
	TrapSend.Length =ntohs(TrapSend.Length );              //计算长度字段
	memcpy(pSendBuff,pChar,PreLength);                     //复制Tag之前的数据到发送Buff
	Offset=PreLength;                                           //指针偏移地址
	
	pChar = (char*)&(TrapSend.BattEnergy.OID_Command);     //电池电量Tag
	ValidLength =TrapSend.BattEnergy.Width+6;                    //计算1个Tag实际占用的字节空间 
	TrapSend.BattEnergy.Width =ntohs(TrapSend.BattEnergy.Width); //调整为网络序，即高字节在前，低字节在后
	memcpy((pSendBuff+Offset),pChar,ValidLength);                     //复制电池电量Tag数据到发送Buff
	Offset = Offset+ValidLength;
	
	pChar = (char*)&(TrapSend.SysTime.OID_Command);        //系统日期Tag
	ValidLength =TrapSend.SysTime.Width+6;                 //计算1个Tag实际占用的字节空间 
	TrapSend.SysTime.Width =ntohs(TrapSend.SysTime.Width); //调整为网络序，即高字节在前，低字节在后
	memcpy((pSendBuff+Offset),pChar,ValidLength);       //复制系统时间Tag数据到发送Buff
	Offset = Offset+ValidLength;
	
	for(i=0;i<TrapSend.Tag_Count;i++)      //
	{
		pChar = (char*)&(TrapSend.TagList[i].OID_Data);   
		ValidLength =TrapSend.TagList[i].Width+6;            //计算1个Tag实际占用的字节空间
		if((Offset+ValidLength) >=(SENDBUFF_SIZE-3))              //防止指针溢出
		{
       break;
    }
		TrapSend.TagList[i].Width =ntohs(TrapSend.TagList[i].Width); //调整为网络序，即高字节在前，低字节在后
	  memcpy((pSendBuff+Offset),pChar,ValidLength);                //复制每一个数据Tag到发送Buff
		Offset = Offset+ValidLength;	
  }
	
	TrapSend.CrcCode=0xffff;                               //CRC字段赋初值
	pSendBuff[Offset++] = TrapSend.CrcCode &0xff;
	pSendBuff[Offset++] = TrapSend.CrcCode>>8;
	
  CrcData = CRC16(pSendBuff, Offset );   // Update the CRC value
	TrapSend.CrcCode =CrcData;
	pSendBuff[Offset-2] = CrcData&0xff;   //CRC低字节在前
	pSendBuff[Offset-1] = CrcData>>8;     //CRC高字节在后

	if(DeviceConfig.RetryNum>0)
	{
     SendCounter =DeviceConfig.RetryNum;
  }
	else
	{
     SendCounter =1;
  }
	for(i=0; i<SendCounter; i++)
	{
     RecevFlag =SendMessage(pSendBuff, Offset, pDeviceID, NodeAddress);
//		 printf("\r\nReceive TrapResponse success:%4x!\r\n", RecevFlag); //测试使用
		 if(RecevFlag ==0x0584)     //成功接收到TrapResponse
		 {
        #if DEBUG_TEST	 
				printf("\r\nReceive TrapResponse success!\r\n"); //测试使用
				#endif
			  break;     
     }
		 rand_num=Get_Rand_Num();
		 printf("the rand_num is %d",rand_num);
		 Delay_ms(2000+2*rand_num);                        //随机发送
  }
//	printf("\r\n----Length:%d----\r\n",Offset);   //测试使用
//	for(i=0;i<Offset;i++)
//	{
//    printf("-%x-",pSendBuff[i]);   //测试使用
//  }
}

/*******************************************************************************
* Function Name  : AlarmTrap
* Description    : 报警主动上传请求
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void  AlarmTrap(u8* pSendBuff, u8* pDeviceID, u16 NodeAddress, struct SenserData* pObtainData)
{
		
//	struct   SpecialDataFrame  TrapSend;
	struct   SpecialDataFrame* pDataFrame  =&TrapSend;
	char*    pChar =NULL;
	u16      RecevFlag =0;       //服务器数据接收标志变量
	uint8_t  readdata[7];        //读取FLASH内数据
	unsigned int rand_num;       //0~1000之间的随机数据
	u16      CrcData=0;
	u8       SendCounter =1;     //发送次数计数器
	u8       i=0,j=0;
	u8       Offset=0;           //发送缓存地址偏移变量
	u8       ValidLength =0;     //有效数据长度，指1个Tag空间中实际存储有效数据的长度。
	u8       LengthKey =0;       //数据净荷部分长度字段指示的数值
	const u8 PreLength   =16;    //帧前缀长度，指从帧前导码到OID序列（或者Tag序列）的数据长度，包括帧前导码，而不包括OID序列（或者Tag序列）
	const u8 CoreLength  =12;    //关键信息长度，指数据净荷部分长度字段指示的数值，去除Tag序列后剩余数据的长度
	
  printf("\r\nFUNCTION: AlarmTrap start! \r\n");                    //

	pChar =(char*)pDataFrame;
	memset(pChar,0x00,sizeof(struct SpecialDataFrame));   //初始化结构体
	TrapSend.Preamble =0xA3;
	TrapSend.Version  =0x20;
	for(i=0;i<6;i++)
	{
    TrapSend.DeviceID[i] =pDeviceID[i];
  }
	TrapSend.RouteFlag =0x01;
	TrapSend.NodeAddr  =ntohs( NodeAddress);              //调整为网络序，即高字节在前，低字节在后
	TrapSend.PDU_Type  =(4<<8)+(1<<7)+4;
	TrapSend.PDU_Type  =ntohs(TrapSend.PDU_Type);         //调整为网络序，即高字节在前，低字节在后
	TrapSend.Seq       =1;
	LengthKey = CoreLength;
	
	TrapSend.BattEnergy.OID_Command = ntohl(DEVICE_QTY);   //调整为网络序，即高字节在前，低字节在后
	TrapSend.BattEnergy.Width =1;                         //
	TrapSend.BattEnergy.Value[0] =DeviceConfig.BatteryCapacity;  //数据上传时的电池剩余电量
	LengthKey = LengthKey+6+TrapSend.BattEnergy.Width;
	
	TrapSend.SysTime.OID_Command= ntohl(SYSTERM_DATA);    //调整为网络序，即高字节在前，低字节在后
	TrapSend.SysTime.Width =3;                            //
	TrapSend.SysTime.Value[0] =systmtime.tm_year-2000;       //系统日期，年
	TrapSend.SysTime.Value[1] =systmtime.tm_mon ;            //系统日期，月
	TrapSend.SysTime.Value[2] =systmtime.tm_mday ;           //系统日期，日
  LengthKey = LengthKey+6+TrapSend.SysTime.Width;

	for(i=0,TrapSend.Tag_Count=0,TrapSend.Length =LengthKey ;i< 1 ;i++)    //对每一个数据Tag赋值
	{
		///////////////////////////////////////将FLASH内的数据读取出来进行上报////////////////////////////////
		DataRead_From_Flash(1,DataCollectCount,0, readdata ,sizeof(readdata));
		pObtainData[i].CollectTime = readdata[0] *256 + readdata[1];
		for(j=0;j<4;j++)
		pObtainData[i].Ch4Data .Data_Hex [j] = readdata[2+j];
		
    TrapSend.TagList[i].OID_Data = (DeviceConfig.CollectPeriod<<11)+pObtainData[i].CollectTime  +(0xC5<<24);     //业务类型5，根据协议修改
		TrapSend.TagList[i].OID_Data = ntohl(TrapSend.TagList[i].OID_Data); //调整为网络序，即高字节在前，低字节在后
		TrapSend.TagList[i].Width =4;
		TrapSend.TagList[i].PerceptionData.Data_F   =pObtainData[i].Ch4Data .Data_Float  ;  //采集到的气体浓度数据
    
		TrapSend.Tag_Count++ ;                                             //对传感器数据Tag进行计数，不包括采集日期Tag
		TrapSend.Length =TrapSend.Length + TrapSend.TagList[i].Width +6;  
  }

	/////////////////////////////////////////////////////////////////////////////////////////////
	TrapSend.Length =ntohs(TrapSend.Length );              //计算长度字段
	memcpy(pSendBuff,pChar,PreLength);                     //复制Tag之前的数据到发送Buff
	Offset=PreLength;                                           //指针偏移地址
	
	pChar = (char*)&(TrapSend.BattEnergy.OID_Command);     //电池电量Tag
	ValidLength =TrapSend.BattEnergy.Width+6;                    //计算1个Tag实际占用的字节空间 
	TrapSend.BattEnergy.Width =ntohs(TrapSend.BattEnergy.Width); //调整为网络序，即高字节在前，低字节在后
	memcpy((pSendBuff+Offset),pChar,ValidLength);                     //复制电池电量Tag数据到发送Buff
	Offset = Offset+ValidLength;
	
	pChar = (char*)&(TrapSend.SysTime.OID_Command);        //系统日期Tag
	ValidLength =TrapSend.SysTime.Width+6;                 //计算1个Tag实际占用的字节空间 
	TrapSend.SysTime.Width =ntohs(TrapSend.SysTime.Width); //调整为网络序，即高字节在前，低字节在后
	memcpy((pSendBuff+Offset),pChar,ValidLength);       //复制系统时间Tag数据到发送Buff
	Offset = Offset+ValidLength;
	
	for(i=0;i<TrapSend.Tag_Count;i++)      //
	{
		pChar = (char*)&(TrapSend.TagList[i].OID_Data);   
		ValidLength =TrapSend.TagList[i].Width+6;            //计算1个Tag实际占用的字节空间
		if((Offset+ValidLength) >=(SENDBUFF_SIZE-3))              //防止指针溢出
		{
       break;
    }
		TrapSend.TagList[i].Width =ntohs(TrapSend.TagList[i].Width); //调整为网络序，即高字节在前，低字节在后
	  memcpy((pSendBuff+Offset),pChar,ValidLength);                //复制每一个数据Tag到发送Buff
		Offset = Offset+ValidLength;	
  }
	
	TrapSend.CrcCode=0xffff;                               //CRC字段赋初值
	pSendBuff[Offset++] = TrapSend.CrcCode &0xff;
	pSendBuff[Offset++] = TrapSend.CrcCode>>8;
	
  CrcData = CRC16(pSendBuff, Offset );   // Update the CRC value
	TrapSend.CrcCode =CrcData;
	pSendBuff[Offset-2] = CrcData&0xff;   //CRC低字节在前
	pSendBuff[Offset-1] = CrcData>>8;     //CRC高字节在后

	if(DeviceConfig.RetryNum>0)
	{
     SendCounter =DeviceConfig.RetryNum;
  }
	else
	{
     SendCounter =1;
  }
	for(i=0; i<SendCounter; i++)
	{
     RecevFlag =SendMessage(pSendBuff, Offset, pDeviceID, NodeAddress);
//		 printf("\r\nReceive TrapResponse success:%4x!\r\n", RecevFlag); //测试使用
		 if(RecevFlag ==0x0584)     //成功接收到TrapResponse
		 {
        #if DEBUG_TEST	 
				printf("\r\nReceive TrapResponse success!\r\n"); //测试使用
				#endif
			  break;     
     }
		 rand_num=Get_Rand_Num();
		 printf("the rand_num is %d",rand_num);
		 Delay_ms(2000+2*rand_num);                        //随机发送
  }
//	printf("\r\n----Length:%d----\r\n",Offset);   //测试使用
//	for(i=0;i<Offset;i++)
//	{
//    printf("-%x-",pSendBuff[i]);   //测试使用
//  }
}

/*******************************************************************************
* Function Name  : GetResponse
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void GetResponse(struct CommandFrame RequestPara, u8* pSendBuff, u8* pDeviceID, u16 NodeAddress)
{
  struct   rtc_time   SystTime;           //RTC时钟设置结构体
//	struct   DataFrame  GetRespSend;
	struct   DataFrame* pDataFrame = & GetRespSend;
	u16      RecevFlag =0;       //服务器数据接收标志变量
	char*    pChar =NULL;

	u8       SendCounter =1;     //发送次数计数器
	u16      CrcData=0;
	u8       i=0,j=0;
	u8       k=0;
	u8       ValidLength =0;     //有效数据长度，指1个Tag空间中实际存储有效数据的长度。
	const u8 PreLength   =16;    //帧前缀长度，指从帧前导码到OID序列（或者Tag序列）的数据长度，包括帧前导码，而不包括OID序列（或者Tag序列）
	const u8 CoreLength  =12;    //关键信息长度，指数据净荷部分长度字段指示的数值，去除OID序列（或者Tag序列）后剩余数据的长度
	
	pChar =(char*)pDataFrame;
	memset(pChar,0x00,sizeof(struct DataFrame));   //初始化结构体
	GetRespSend.Preamble =0xA3;
	GetRespSend.Version  =0x20;
	for(i=0;i<6;i++)
	{
    GetRespSend.DeviceID[i] =pDeviceID[i];
  }
	GetRespSend.RouteFlag =0x01;
	GetRespSend.NodeAddr  =ntohs( NodeAddress);              //调整为网络序，即高字节在前，低字节在后
	GetRespSend.PDU_Type  =(2<<8)+(1<<7)+4;
	GetRespSend.PDU_Type  =ntohs(GetRespSend.PDU_Type);      //调整为网络序，即高字节在前，低字节在后
	GetRespSend.Seq       =1;
	GetRespSend.Length    =CoreLength ;                       //
	for(i=0,GetRespSend.Tag_Count=0; i<RequestPara.OID_Count; i++)
	{
		 switch(RequestPara.OID_List[i])
	   {
        case DEF_NR:            //重传次数
				{
            GetRespSend.TagList[i].OID_Command =DEF_NR;
					  GetRespSend.TagList[i].Width =1;
					  GetRespSend.TagList[i].Value[0] =DeviceConfig.RetryNum;
					  GetRespSend.Tag_Count++;
					  GetRespSend.Length =GetRespSend.Length  + GetRespSend.TagList[i].Width +6; 
					  break;
        }	
				case SYSTERM_TIME:     //系统时间
				{
      
						Time_Display(RTC_GetCounter(),&SystTime);     //获取系统当前时间			
						DeviceConfig.Time_Sec  =SystTime.tm_sec;
						DeviceConfig.Time_Min  =SystTime.tm_min;
						DeviceConfig.Time_Hour =SystTime.tm_hour;
						DeviceConfig.Time_Mday =SystTime.tm_mday;		
						DeviceConfig.Time_Mon  =SystTime.tm_mon;
						DeviceConfig.Time_Year =SystTime.tm_year-2000; //对上传年份去基数修正
					
					  GetRespSend.TagList[i].OID_Command =SYSTERM_TIME;
					  GetRespSend.TagList[i].Width =6;
					  GetRespSend.TagList[i].Value[0] =DeviceConfig.Time_Year;   //系统日期，年
					  GetRespSend.TagList[i].Value[1] =DeviceConfig.Time_Mon;    //系统日期，月
					  GetRespSend.TagList[i].Value[2] =DeviceConfig.Time_Mday;   //系统日期，日
					  GetRespSend.TagList[i].Value[3] =DeviceConfig.Time_Hour;   //系统时间，小时
					  GetRespSend.TagList[i].Value[4] =DeviceConfig.Time_Min;    //系统时间，分
					  GetRespSend.TagList[i].Value[5] =DeviceConfig.Time_Sec;    //系统时间，秒
					  GetRespSend.Tag_Count++;
					  GetRespSend.Length =GetRespSend.Length  + GetRespSend.TagList[i].Width +6; 
					  break;
        }	
        case CLT1_ITRL1:       //一时区采集间隔
				{
            GetRespSend.TagList[i].OID_Command =CLT1_ITRL1;
					  GetRespSend.TagList[i].Width =2;
					  GetRespSend.TagList[i].Value[0] =DeviceConfig.CollectPeriod >>8;   //数据采集间隔，高字节在前
					  GetRespSend.TagList[i].Value[1] =DeviceConfig.CollectPeriod &0xff; //数据采集间隔，低字节在后
					  GetRespSend.Tag_Count++;
					  GetRespSend.Length =GetRespSend.Length  + GetRespSend.TagList[i].Width +6; 
					  break;
        }	
				case CLT1_CNT1:        //一时区采集次数
				{
            GetRespSend.TagList[i].OID_Command =CLT1_CNT1;
					  GetRespSend.TagList[i].Width =2;
					  GetRespSend.TagList[i].Value[0] =DeviceConfig.CollectNum >>8;   //数据采集间隔，高字节在前
					  GetRespSend.TagList[i].Value[1] =DeviceConfig.CollectNum &0xff; //数据采集间隔，低字节在后
					  GetRespSend.Tag_Count++;
					  GetRespSend.Length =GetRespSend.Length  + GetRespSend.TagList[i].Width +6; 
					  break;
        }
				
				case CLT1_STIME1 :    //一时区采集开始时间
				{
            GetRespSend.TagList[i].OID_Command =CLT1_STIME1;
					  GetRespSend.TagList[i].Width =2;
					  GetRespSend.TagList[i].Value[0] = DeviceConfig.CollectStartTime>>8;    //启动时间，高字节在前 
					  GetRespSend.TagList[i].Value[1] = DeviceConfig.CollectStartTime&0xff;  //启动时间，低字节在后  					
					  GetRespSend.Tag_Count++;
					  GetRespSend.Length =GetRespSend.Length  + GetRespSend.TagList[i].Width +6; 
					  break;
        }	
//				
				case UPLOAD_CYCLE:     //数据上报周期
				{
            GetRespSend.TagList[i].OID_Command =UPLOAD_CYCLE;
					  GetRespSend.TagList[i].Width =2;
					
					  GetRespSend.TagList[i].Value[0] =DeviceConfig.SendCount  >>8;   //数据采集间隔，高字节在前
					  GetRespSend.TagList[i].Value[1] =DeviceConfig.SendCount  &0xff; //数据采集间隔，低字节在后
					  GetRespSend.Tag_Count++;
					  GetRespSend.Length =GetRespSend.Length  + GetRespSend.TagList[i].Width +6; 
					  break;
        }	

////////////////////////////////////////////////////////////////////////////////////////////////////////////
			case LOW_LEVEL:     //报警低浓度阈值
				{
            GetRespSend.TagList[i].OID_Command =LOW_LEVEL;
					  GetRespSend.TagList[i].Width =4;
					  for(k=0;k<4;k++)
					{
					   GetRespSend.TagList[i].Value[k] = DeviceConfig.LowAlarmLevel .Data_Hex[k]   ;   //数据采集间隔，高字节在前
					 // GetRespSend.TagList[i].Value[1] =DeviceConfig.AlarmData  &0xff; //数据采集间隔，低字节在后
					}
					  GetRespSend.Tag_Count++;
					  GetRespSend.Length =GetRespSend.Length  + GetRespSend.TagList[i].Width +6; 
					  break;
        }			
				case HIGH_LEVEL:     //报警高浓度阈值
				{
            GetRespSend.TagList[i].OID_Command =HIGH_LEVEL;
					  GetRespSend.TagList[i].Width =4;
					  for(k=0;k<4;k++)
					{
					   GetRespSend.TagList[i].Value[k] = DeviceConfig.HighAlarmLevel .Data_Hex[k]   ;   //数据采集间隔，高字节在前
					 // GetRespSend.TagList[i].Value[1] =DeviceConfig.AlarmData  &0xff; //数据采集间隔，低字节在后
					}
					  GetRespSend.Tag_Count++;
					  GetRespSend.Length =GetRespSend.Length  + GetRespSend.TagList[i].Width +6; 
					  break;
        }		
				
//       case SMS_PHONE:     //报警电话号码
//				{
//            GetRespSend.TagList[i].OID_Command =SMS_PHONE;
//					  GetRespSend.TagList[i].Width =phone_number_next;
//					for(j=0;j<phone_number_next;j++)
//					{
//					  GetRespSend.TagList[i].Value[j] =DeviceConfig.AlarmPhoneNum  [j];   //报警电话号码
//					 
//					}
//					  GetRespSend.Tag_Count++;
//					  GetRespSend.Length =GetRespSend.Length  + GetRespSend.TagList[i].Width +6; 
//					  break;
//        }	
//			  case BSS_IP:     //网络服务IP地址
//				{
//            GetRespSend.TagList[i].OID_Command =BSS_IP;
//					  GetRespSend.TagList[i].Width =ip_number_next;
//					for(j=0;j<ip_number_next;j++)
//					{
//					  GetRespSend.TagList[i].Value[j] =DeviceConfig.ServerIP  [j];   //数据采集间隔，高字节在前
//					
//					}
//					  GetRespSend.Tag_Count++;
//					  GetRespSend.Length =GetRespSend.Length  + GetRespSend.TagList[i].Width +6; 
//					  break;
//        }	
//				  case BSS_PORT:     //网络服务端口号
//				{
//            GetRespSend.TagList[i].OID_Command =BSS_PORT;
//					  GetRespSend.TagList[i].Width =port_number_next;
//					  for(j=0;j<port_number_next;j++)
//					{
//					  GetRespSend.TagList[i].Value[j] =DeviceConfig.ServerPort  [j];   //数据采集间隔，高字节在前
//					 // GetRespSend.TagList[i].Value[1] =DeviceConfig.AlarmData  &0xff; //数据采集间隔，低字节在后
//					}
//					  GetRespSend.Tag_Count++;
//					  GetRespSend.Length =GetRespSend.Length  + GetRespSend.TagList[i].Width +6; 
//					  break;
//        }	
				

////////////////////////////////////////////////////////////////////////////////////////////////////////////				
        default:
				{
					 #if DEBUG_TEST	
           printf("\r\nWarning!!Tag OID not recognition!\r\n"); //测试使用
					 #endif
					 break;
        }
		}
  }
	
	GetRespSend.Length =ntohs(GetRespSend.Length );                //调整为网络序，即高字节在前，低字节在后
	GetRespSend.CrcCode=0xffff;                                    //CRC字段赋初值
	
	memcpy(pSendBuff,pChar,PreLength);
	
	for(i=0,j=PreLength;i<GetRespSend.Tag_Count;i++)
	{
    GetRespSend.TagList[i].OID_Command =ntohl(GetRespSend.TagList[i].OID_Command);  //将OID序列调整为高字节在前，低字节在后  //有待测试
		pChar = (char*)&(GetRespSend.TagList[i].OID_Command);   
		ValidLength =GetRespSend.TagList[i].Width+6;           //计算1个Tag实际占用的字节空间
		if((j+ValidLength) >=(SENDBUFF_SIZE-3))                //防止指针溢出
		{
       break;
    }
		GetRespSend.TagList[i].Width =ntohs(GetRespSend.TagList[i].Width); //调整为网络序，即高字节在前，低字节在后
	  memcpy((pSendBuff+j),pChar,ValidLength); 
		j = j+ValidLength;
		
  }
	pSendBuff[j++] = GetRespSend.CrcCode &0xff;
	pSendBuff[j++] = GetRespSend.CrcCode>>8;
	
  CrcData = CRC16(pSendBuff, j );   // Update the CRC value
	GetRespSend.CrcCode =CrcData;
	pSendBuff[j-2] = CrcData&0xff;   //CRC低字节在前
	pSendBuff[j-1] = CrcData>>8;     //CRC高字节在后
  
	if(DeviceConfig.RetryNum>0)
	{
     SendCounter =DeviceConfig.RetryNum;
  }
	else
	{
     SendCounter =1;
  }
	for(i=0;i<SendCounter;i++)
	{
     RecevFlag =SendMessage(pSendBuff, j, pDeviceID, NodeAddress);
		 printf("\r\nReceive GetResponse success:%4x!\r\n", RecevFlag); //测试使用
		 if(RecevFlag ==0x0384)     //成功接收到GetResponse
		 {
        #if DEBUG_TEST	 
				printf("\r\nReceive GetResponse success!\r\n"); //测试使用
				#endif
			  break;     
     }
//		 Delay_ms(3000); 
  }
  
//	printf("\r\n----Length:%d----\r\n",j);   //测试使用
//	for(i=0;i<j;i++)
//	{
//    printf(",0x%x",pSendBuff[i]);   //测试使用
//  }

}
/*******************************************************************************
* Function Name  : 二院协议数据分析
* Description    : 用于接收服务器配置，
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Treaty_Data_Analysis(u8* pTreatyBuff, u16* pFlag, u8* pDeviceID, u16 NodeAddress)	
{
	u8   DataLen =0;          //Tag序列或者OID序列的长度
	u16  PduType =0;
  
	u8   i=0,j=0,k=0;
	u8*  pChar =NULL;
	u32* pOid  =NULL;
	struct CommandFrame ParaRequest;
	struct TagStruct RecTagList[MAX];           //接收到的TAG
	uint8_t RecTagNum;                          //接收到TAG的数量
	
  PduType =pTreatyBuff[13]*256 +pTreatyBuff[14];   //结算接收数据PDU编码
  
	switch(PduType)
	{
/******************************************服务器请求********************************************/	     
        case ((GETREQUEST<<8)+(1<<8)+DEVICE):
				{
#if DEBUG_TEST	 
						printf("\r\nReceive Get Request Command from Server.\r\n");    //监测到服务器发送的请求命令
#endif
						*pFlag =PduType;
						DataLen =pTreatyBuff[2]*256 +pTreatyBuff[3]-12; //接收的OID的总长度
					  ParaRequest.OID_Count =DataLen/4;               //接收的OID的数量
					  if((ParaRequest.OID_Count >0)&&(ParaRequest.OID_Count <= MAX))   //限定OID的数量，控制在20个
						{
							  pOid = (u32*)&pTreatyBuff[16];
							  j    =ParaRequest.OID_Count;
							  RecTagNum = j;                            //接收TAG的总数
								for(i=0;i<ParaRequest.OID_Count;i++)
								{ 
									  ParaRequest.OID_List[i] =(CommandType)*pOid;                    //进行OID采集   
									  ParaRequest.OID_List[i] =ntohl( ParaRequest.OID_List[i] );      //进行网络序列的变化
									
									  RecTagList[i].OID_Command= ParaRequest.OID_List[i];             //采集TAG的OID
									  RecTagList[i].Width =0;                                         //长度为0,一次无value
									
									  printf("\r\n--Cycle--%4x----.\r\n", ParaRequest.OID_List[i]);   //显示OID
										j--;
									  if(j==0)   
										{
                       break;
                    }
									  pOid++;
								}
								printf("\r\n--i:%d--j:%d--%4x----.\r\n",i,j, ParaRequest.OID_List[i]);    //显示OID
								
            }
					  else
						{
#if DEBUG_TEST	 
							  printf("\r\nReceive Command OID not correct.\r\n");    //????
#endif
            }

						DMA_UART3_RECEV_FLAG =0;     //清接收标志变量
						SendDataToServ(GETRESPONSE,RecTagList,RecTagNum,Usart3_send_buff,pDeviceID);    //进行响应
					  //GetResponse( ParaRequest, Usart3_send_buff, pDeviceID,  NodeAddress);
					  break;
        }	
/******************************************设置设备参数请求********************************************/	
        case ((SETREQUEST<<8)+(1<<8)+DEVICE):                  //接收服务器下发的配置
				{
            #if DEBUG_TEST	 
						printf("\r\nReceive Set Request Command from Server.\r\n");    //测试使用
						#endif
						*pFlag =PduType;
					  DataLen =pTreatyBuff[2]*256 +pTreatyBuff[3]-12;    //接收到的Tag序列的总长度
					  if(DataLen >6)           //至少存在一个合法的配置参数
						{
                pOid = (u32*)(pTreatyBuff+16);    //服务器下发的第一个OID
							  i=0;
							  while( DataLen >6 )
								{
									 printf("\r\n--Cycle--%4x----.\r\n",ntohl(*pOid));         //测试使用
									 pChar = (u8*)pOid;
                   switch(ntohl(*pOid))
									 {
											
											case SYSTERM_TIME:     //系统时间
											{
													DeviceConfig.Time_Year =*(pChar+6);
												  DeviceConfig.Time_Mon  =*(pChar+7);
												  DeviceConfig.Time_Mday =*(pChar+8);
												  DeviceConfig.Time_Hour =*(pChar+9);
												  DeviceConfig.Time_Min  =*(pChar+10);
												  DeviceConfig.Time_Sec  =*(pChar+11);
												  printf("\r\n-年-月-日-时-分-秒：-%d--%d--%d--%d--%d--%d--.\r\n",DeviceConfig.Time_Year,DeviceConfig.Time_Mon,DeviceConfig.Time_Mday,
												                 DeviceConfig.Time_Hour,DeviceConfig.Time_Min, DeviceConfig.Time_Sec  );         //测试使用
												  if((DeviceConfig.Time_Mon<=12)&&(DeviceConfig.Time_Mday<=31)&&(DeviceConfig.Time_Hour<=23)&&(DeviceConfig.Time_Min<=60)&&(DeviceConfig.Time_Sec<=60)) //参数合法性判定
													{
                            Time_Auto_Regulate(&DeviceConfig);             //通过服务器下发参数进行RTC时钟校准，
                          }
												  ParaRequest.OID_List[i] =SYSTERM_TIME; 
/**********************************************************************************************************/														
                          RecTagList[i].OID_Command=ntohl(SYSTERM_TIME);          //进行TAG填充 OID
													RecTagList[i].Width=6;                                  // //进行TAG填充 WIDTH
													for(k=0;k<6;k++)
													RecTagList[i].Value[k]=pChar[6+k];                      // 进行TAG填充 VALUE
/**********************************************************************************************************/	                          													
												  pOid =(u32*)(pChar+12);                          //指针后移1个Tag
												  DataLen = DataLen-12;
												 	i++;        //合法OID计数器
													break;							
											}	
											case CLT1_ITRL1:       //一时区采集间隔
											{
													DeviceConfig.CollectPeriod =pChar[6]*256+pChar[7];
												   
												  ConfigData.CollectPeriod_Byte[0]= *(pChar+6);
												  ConfigData.CollectPeriod_Byte[1]= *(pChar+7);                           											 
												  printf("\r\n---CollectPeriod:-%d---.\r\n", DeviceConfig.CollectPeriod );         //测试使用
												  if(0<DeviceConfig.CollectPeriod<=60)              //参数合法性判定
													{
														 Delay_ms(100); 
												UploadFlash((char *)ConfigData.CollectPeriod_Byte,2);  //参数存入Flash
														 Delay_ms(100); 
                          }
												  ParaRequest.OID_List[i] =CLT1_ITRL1;   
/**********************************************************************************************************/													
                          RecTagList[i].OID_Command=ntohl(CLT1_ITRL1);          //进行TAG填充 OID
													RecTagList[i].Width=2;                                  // //进行TAG填充 WIDTH
													for(k=0;k<2;k++)
													RecTagList[i].Value[k]=pChar[6+k];                      // 进行TAG填充 VALUE
/**********************************************************************************************************/	
													
												  pOid =(u32*)(pChar+8);                         //指针后移1个Tag
												  DataLen = DataLen-8;
												 	i++;        //合法OID计数器
													break;
											}	
											case CLT1_CNT1:        //一时区采集次数
											{
													DeviceConfig.CollectNum =pChar[6]*256+pChar[7];

													printf("\r\n-CollectNum:-%2x---.\r\n", DeviceConfig.CollectNum );         //测试使用
												  if(DeviceConfig.CollectNum<=1440)              //参数合法性判定
													{
                             //UploadFlash((char*)&(DeviceConfig.CollectNum), 9);  //参数存入Flash
                          }
												  ParaRequest.OID_List[i] =CLT1_CNT1; 
/**********************************************************************************************************/													
                          RecTagList[i].OID_Command=ntohl(CLT1_CNT1);          //进行TAG填充 OID
													RecTagList[i].Width=2;                                  // //进行TAG填充 WIDTH
													for(k=0;k<2;k++)
													RecTagList[i].Value[k]=pChar[6+k];                      // 进行TAG填充 VALUE
/**********************************************************************************************************/	

											
												  pOid =(u32*)(pChar+8);                         //指针后移1个Tag
												  DataLen = DataLen-8;
												 	i++;        //合法OID计数器
													break;
											}
											
//									
											case UPLOAD_CYCLE:     //数据上报周期
											{
													DeviceConfig.SendCount  =pChar[6]*256+pChar[7];
														   //for(k=0;k<1;k++)
												  ConfigData.SendCount_Byte [0]= *(pChar+6);
												  ConfigData.SendCount_Byte [1]= *(pChar+7);
				
												  if(0<DeviceConfig.SendCount <=1440)              //参数合法性判定
													{
														 Delay_ms(100); 
														 //DataWrite_To_Flash(0,3,0,(uint8_t*)ConfigData.SendCount_Byte ,2);      //将采集周期写入Flash
                             UploadFlash((char*)ConfigData.SendCount_Byte, 3);  //参数存入Flash
														 Delay_ms(100); 
                          }
											  	printf("\r\n-UploadCycle:-%d---.\r\n", DeviceConfig.SendCount  ); //测试使用
												  ParaRequest.OID_List[i] =UPLOAD_CYCLE;   
/**********************************************************************************************************/													
                          RecTagList[i].OID_Command=ntohl(UPLOAD_CYCLE);          //进行TAG填充 OID
													RecTagList[i].Width=2;                                  // //进行TAG填充 WIDTH
													for(k=0;k<2;k++)
													RecTagList[i].Value[k]=pChar[6+k];                      // 进行TAG填充 VALUE
/**********************************************************************************************************/													
												  pOid =(u32*)(pChar+8);                           //指针后移1个Tag
												  DataLen = DataLen-8;
											  	i++;        //合法OID计数器
													break;
											}	
									
							
											    case LOW_LEVEL:     //报警低浓度阈值
											{   
												  for(k=0;k<4;k++)
													DeviceConfig.LowAlarmLevel .Data_Hex[k]    =pChar[6+k];
												  if(DeviceConfig.LowAlarmLevel  .Data_Float  <=25.0)              //参数合法性判定
													{
                             UploadFlash((char*)(DeviceConfig.LowAlarmLevel .Data_Hex   ), 7);  //参数存入Flash
                          }
													for(k=0;k<4;k++)
											  	printf("\r\n-AlarmData:-%2x---.\r\n", DeviceConfig.LowAlarmLevel  .Data_Hex[k]  ); //测试使用
												  ParaRequest.OID_List[i] =LOW_LEVEL;       
/**********************************************************************************************************/													
                          RecTagList[i].OID_Command=ntohl(LOW_LEVEL);          //进行TAG填充 OID
													RecTagList[i].Width=4;                                  // //进行TAG填充 WIDTH
													for(k=0;k<4;k++)
													RecTagList[i].Value[k]=pChar[6+k];                      // 进行TAG填充 VALUE
/**********************************************************************************************************/
												  pOid =(u32*)(pChar+10);                           //指针后移1个Tag
												  DataLen = DataLen-10;
											  	i++;        //合法OID计数器
													break;
											}
											case CLT1_STIME1  :    //一时区采集起始时间
											{
													
												  DeviceConfig.CollectStartTime =pChar[6]*256+pChar[7];
													printf("\r\n-Collect Start Time:-%2x---.\r\n", DeviceConfig.CollectStartTime );        //测试使用
												  if(DeviceConfig.CollectStartTime<=1440)              //参数合法性判定
													{
                             UploadFlash((char*)&(DeviceConfig.CollectStartTime), 8);  //参数存入Flash
                          }
												  ParaRequest.OID_List[i] =CLT1_STIME1;  
/**********************************************************************************************************/													
                          RecTagList[i].OID_Command=ntohl(CLT1_STIME1);          //进行TAG填充 OID
													RecTagList[i].Width=2;                                  // //进行TAG填充 WIDTH
													for(k=0;k<2;k++)
													RecTagList[i].Value[k]=pChar[6+k];                      // 进行TAG填充 VALUE
/**********************************************************************************************************/													
												  pOid =(u32*)(pChar+8);                              //指针后移1个Tag
												  DataLen = DataLen-8;
												 	i++;        //合法OID计数器
													break;
											}	
											case DEF_NR:            //重传次数
											{
													DeviceConfig.RetryNum =*(pChar+6);
												  if(1<= DeviceConfig.RetryNum <10)                         //参数合法性判定
													{
														 Delay_ms(100); 
                             UploadFlash((char*)&(DeviceConfig.RetryNum), 10);  //参数存入Flash
														 Delay_ms(100); 
                          }
												  printf("\r\n-Retry Num-%x---.\r\n", DeviceConfig.RetryNum);   //测试使用
												  ParaRequest.OID_List[i] =DEF_NR;    
/**********************************************************************************************************/													
                          RecTagList[i].OID_Command=ntohl(DEF_NR);          //进行TAG填充 OID
													RecTagList[i].Width=1;                                  // //进行TAG填充 WIDTH
													for(k=0;k<1;k++)
													RecTagList[i].Value[k]=pChar[6+k];                      // 进行TAG填充 VALUE
/**********************************************************************************************************/														
												  pOid =(u32*)(pChar+7);                     //指针后移1个Tag
												  DataLen = DataLen-7; 
 	                        i++;               //合法OID计数器												
													break;
											}	
											
											case HIGH_LEVEL:     //报警高浓度阈值
											{   
												  for(k=0;k<4;k++)
													DeviceConfig.HighAlarmLevel .Data_Hex[k]    =pChar[6+k];
												  if( 25.0<DeviceConfig.LowAlarmLevel  .Data_Float < DeviceConfig.HighAlarmLevel  .Data_Float  <=50.0)              //参数合法性判定
													{
                             UploadFlash((char*)(DeviceConfig.HighAlarmLevel .Data_Hex   ), 11);  //参数存入Flash
                          }
													for(k=0;k<4;k++)
											  	printf("\r\n-AlarmData:-%2x---.\r\n", DeviceConfig.HighAlarmLevel  .Data_Hex[k]  ); //测试使用
												  ParaRequest.OID_List[i] =HIGH_LEVEL;  
/**********************************************************************************************************/													
                          RecTagList[i].OID_Command=ntohl(HIGH_LEVEL);          //进行TAG填充 OID
													RecTagList[i].Width=4;                                  // //进行TAG填充 WIDTH
													for(k=0;k<4;k++)
													RecTagList[i].Value[k]=pChar[6+k];                      // 进行TAG填充 VALUE
/**********************************************************************************************************/													
												  pOid =(u32*)(pChar+10);                           //指针后移1个Tag
												  DataLen = DataLen-10;
											  	i++;        //合法OID计数器
													break;
											}
									
											default:
											{
												 #if DEBUG_TEST	
												 printf("\r\nWarning!!Tag OID not recognition!\r\n"); //测试使用
												 #endif
												 pOid =(u32*)(pChar+1);  //指针后移一个字节，查询后续有无合法OID
												 DataLen = DataLen-1;    //指针后移一个字节，查询后续有无合法OID
												 break;
											}
									 }
                }
						}
						if((i>0)&&(i<=20))
						{
               ParaRequest.OID_Count =i;           //对OID序列进行计数	
							 RecTagNum=i;                        //TAG的数量
            }
            else
						{
               ParaRequest.OID_Count =7;           //当OID计数器值超过20时,将计数器值重置成默认值（默认7）
            }	
		
            DMA_UART3_RECEV_FLAG =0;     //接收标志变量复位	
						
            SendDataToServ(GETRESPONSE,RecTagList,RecTagNum,Usart3_send_buff,pDeviceID);    //进行响应						
					  //GetResponse( ParaRequest, Usart3_send_buff, pDeviceID,  NodeAddress);        //利用串口3进行通信
					  break;
        }	
				
/******************************************设备数据上报信息响应*********************************************/	
				case ((TRAPRESPONSE<<8)+(1<<8)+DEVICE):                  //收到上报数据的响应
				{
#if DEBUG_TEST	 
						printf("\r\nReceive Trap Response Command from Server.\r\n");    //测试使用
#endif
					  /////////////////////////
					  //服务器接收完整性验证，对于一次上传一帧数据的情况，只需通过PDUType进行验证，接收到应答帧即代表接收完整
						*pFlag =PduType;
					  break;
        }	
/******************************************设备检查链接响应*********************************************/	
				case ((ONLINERESPONSE<<8)+(1<<8)+DEVICE):                      //设备检查链接响应
				{
#if DEBUG_TEST	 
						printf("\r\nReceive Online Response Command from Server.\r\n");    //测试使用
#endif
					
						*pFlag =PduType;
					  break;
        }	
/******************************************设备开机上报信息响应*********************************************/				
				case ((STARTUPRESPONSE<<8)+(1<<8)+DEVICE):                       //收到设备开机上报信息响应
				{
            #if DEBUG_TEST	 
						printf("\r\nReceive Startup Response Command from Server.\r\n");    //测试使用
						#endif
						*pFlag =PduType;
					  break;
        }	
/******************************************服务器唤醒设备请求*********************************************/				
				case ((WAKEUPREQUEST<<8)+(1<<8)+DEVICE):
				{
            #if DEBUG_TEST	 
						printf("\r\nReceive Wakeup Request Command from Server.\r\n");    //测试使用
						#endif
					
						*pFlag =PduType;
					
						if(DMA_UART3_RECEV_FLAG==1)                      //查询数据接收情况
						{
							DMA_UART3_RecevDetect(pDeviceID, NodeAddress);     //查询是否有数据传输
						}
					  SendDataToServ(WAKEUPRESPONSE,NULL,0,Usart3_send_buff, pDeviceID);
					  break;
					
						
					  //break;
        }	

        default:
				{
					 #if DEBUG_TEST	
           printf("\r\nWarning!!PDU Type not recognition!\r\n"); //测试使用
					 #endif
					 break;
        }
   }      
}

